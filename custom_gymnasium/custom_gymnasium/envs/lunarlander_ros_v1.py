import math
import warnings
from typing import TYPE_CHECKING, Optional
import numpy as np
import gymnasium as gym
from gymnasium import error, spaces
from gymnasium.error import DependencyNotInstalled
from gymnasium.utils import EzPickle, colorize
from gymnasium.utils.step_api_compatibility import step_api_compatibility

from gymnasium.envs.box2d.lunar_lander import LunarLander
from gymnasium.envs.box2d.lunar_lander import ContactDetector 
from gymnasium.envs.box2d.lunar_lander import VIEWPORT_W, VIEWPORT_H, SCALE, INITIAL_RANDOM, LEG_AWAY, LEG_DOWN, LEG_W, LEG_H, LEG_SPRING_TORQUE, LANDER_POLY, FPS, MAIN_ENGINE_POWER, SIDE_ENGINE_AWAY, SIDE_ENGINE_HEIGHT, SIDE_ENGINE_POWER, SCALE

try:
    import Box2D
    from Box2D.b2 import (
        circleShape,
        contactListener,
        edgeShape,
        fixtureDef,
        polygonShape,
        revoluteJointDef,
    )
except ImportError as e:
    raise DependencyNotInstalled(
        "Box2D is not installed, run `pip install gymnasium[box2d]`"
    ) from e

import rospy
import math
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Polygon, Point32

class LunarLander_ROS_v1(LunarLander):
    
    def __init__(self,
        render_mode: Optional[str] = None,
        continuous: bool = False,
        gravity: float = -10.0,
        enable_wind: bool = False,
        wind_power: float = 15.0,
        turbulence_power: float = 1.5,):
        super().__init__(
            render_mode=render_mode,
            continuous=continuous,
            gravity=gravity,
            enable_wind=enable_wind,
            wind_power=wind_power,
            turbulence_power=turbulence_power,
        )
        # self.gravity = gravity
        # self.turbulence_power = turbulence_power
        # self.render_mode = render_mode
        # rest of the initialization code
        self.terrain_pub = rospy.Publisher('terrain', Float32MultiArray, queue_size=1)
        self.lunarlander_pub = rospy.Publisher('lunarlander', Polygon, queue_size=1)
        self.thruster_pub = rospy.Publisher('thruster', Int32, queue_size=1)
        rospy.init_node('lunarlander_node', anonymous=True)
        self.terrain = None
        self.lunarlander = None
        self.thruster = None

    def _publish_msgs(self, terrain_msg, lunarlander_msg, thruster_msg):
        self.terrain_pub.publish(terrain_msg)

        self.lunarlander_pub.publish(lunarlander_msg)

        self.thruster_pub.publish(thruster_msg)

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ):
        super().reset(seed=seed)
        self._destroy()

        # Bug's workaround for: https://github.com/Farama-Foundation/Gymnasium/issues/728
        # Not sure why the self._destroy() is not enough to clean(reset) the total world environment elements, need more investigation on the root cause,
        # we must create a totally new world for self.reset(), or the bug#728 will happen
        self.world = Box2D.b2World(gravity=(0, self.gravity))
        self.world.contactListener_keepref = ContactDetector(self)
        self.world.contactListener = self.world.contactListener_keepref
        self.game_over = False
        self.prev_shaping = None

        W = VIEWPORT_W / SCALE
        H = VIEWPORT_H / SCALE

        # Create Terrain
        CHUNKS = 11
        height = self.np_random.uniform(0, H / 2, size=(CHUNKS + 1,))
        chunk_x = [W / (CHUNKS - 1) * i for i in range(CHUNKS)]
        self.helipad_x1 = chunk_x[CHUNKS // 2 - 1]
        self.helipad_x2 = chunk_x[CHUNKS // 2 + 1]
        self.helipad_y = H / 4
        height[CHUNKS // 2 - 2] = self.helipad_y
        height[CHUNKS // 2 - 1] = self.helipad_y
        height[CHUNKS // 2 + 0] = self.helipad_y
        height[CHUNKS // 2 + 1] = self.helipad_y
        height[CHUNKS // 2 + 2] = self.helipad_y
        smooth_y = [
            0.33 * (height[i - 1] + height[i + 0] + height[i + 1])
            for i in range(CHUNKS)
        ]

        self.terrain = smooth_y

        self.moon = self.world.CreateStaticBody(
            shapes=edgeShape(vertices=[(0, 0), (W, 0)])
        )
        self.sky_polys = []
        for i in range(CHUNKS - 1):
            p1 = (chunk_x[i], smooth_y[i])
            p2 = (chunk_x[i + 1], smooth_y[i + 1])
            self.moon.CreateEdgeFixture(vertices=[p1, p2], density=0, friction=0.1)
            self.sky_polys.append([p1, p2, (p2[0], H), (p1[0], H)])

        self.moon.color1 = (0.0, 0.0, 0.0)
        self.moon.color2 = (0.0, 0.0, 0.0)

        # Create Lander body
        initial_y = VIEWPORT_H / SCALE
        initial_x = VIEWPORT_W / SCALE / 2
        self.lander: Box2D.b2Body = self.world.CreateDynamicBody(
            position=(initial_x, initial_y),
            angle=0.0,
            fixtures=fixtureDef(
                shape=polygonShape(
                    vertices=[(x / SCALE, y / SCALE) for x, y in LANDER_POLY]
                ),
                density=5.0,
                friction=0.1,
                categoryBits=0x0010,
                maskBits=0x001,  # collide only with ground
                restitution=0.0,
            ),  # 0.99 bouncy
        )
        self.lander.color1 = (128, 102, 230)
        self.lander.color2 = (77, 77, 128)

        # Apply the initial random impulse to the lander
        self.lander.ApplyForceToCenter(
            (
                self.np_random.uniform(-INITIAL_RANDOM, INITIAL_RANDOM),
                self.np_random.uniform(-INITIAL_RANDOM, INITIAL_RANDOM),
            ),
            True,
        )

        if self.enable_wind:  # Initialize wind pattern based on index
            self.wind_idx = self.np_random.integers(-9999, 9999)
            self.torque_idx = self.np_random.integers(-9999, 9999)

        # Create Lander Legs
        self.legs = []
        for i in [-1, +1]:
            leg = self.world.CreateDynamicBody(
                position=(initial_x - i * LEG_AWAY / SCALE, initial_y),
                angle=(i * 0.05),
                fixtures=fixtureDef(
                    shape=polygonShape(box=(LEG_W / SCALE, LEG_H / SCALE)),
                    density=1.0,
                    restitution=0.0,
                    categoryBits=0x0020,
                    maskBits=0x001,
                ),
            )
            leg.ground_contact = False
            leg.color1 = (128, 102, 230)
            leg.color2 = (77, 77, 128)
            rjd = revoluteJointDef(
                bodyA=self.lander,
                bodyB=leg,
                localAnchorA=(0, 0),
                localAnchorB=(i * LEG_AWAY / SCALE, LEG_DOWN / SCALE),
                enableMotor=True,
                enableLimit=True,
                maxMotorTorque=LEG_SPRING_TORQUE,
                motorSpeed=+0.3 * i,  # low enough not to jump back into the sky
            )
            if i == -1:
                rjd.lowerAngle = (
                    +0.9 - 0.5
                )  # The most esoteric numbers here, angled legs have freedom to travel within
                rjd.upperAngle = +0.9
            else:
                rjd.lowerAngle = -0.9
                rjd.upperAngle = -0.9 + 0.5
            leg.joint = self.world.CreateJoint(rjd)
            self.legs.append(leg)

        self.drawlist = [self.lander] + self.legs

        if self.render_mode == "human":
            self.render()
        return self.step(np.array([0, 0]) if self.continuous else 0)[0], {}

    def step(
        self,
        action: np.ndarray,
    ):
        def transform_vertex(x, y, angle, position):
            # Rotate the point around the origin (0, 0) and then translate it
            x_rotated = x * math.cos(angle) - y * math.sin(angle)
            y_rotated = x * math.sin(angle) + y * math.cos(angle)
            # Translate the point by the position of the body
            x_translated = x_rotated + position.x
            y_translated = y_rotated + position.y
            return x_translated, y_translated
        """
        The step for
        1. publish the action to the thruster
        2. update the uav lander
        3. publish the uav lander state

        Args:

        Returns:
            
        """
        statet, reward, terminated, truncated, info = super().step(action=action)
        terrain_msg = Float32MultiArray(data=self.terrain)
        scaled_vertices = None
        for fixture in self.lander.fixtures:
            shape = fixture.shape
            # Check if the shape is a polygon
            if isinstance(shape, Box2D.b2PolygonShape):
                # Access the vertices of the polygon shape
                vertices = shape.vertices
                # Scale the vertices back up if necessary (assuming you have a SCALE variable)
                scaled_vertices = [(vertex[0] * SCALE, vertex[1] * SCALE) for vertex in vertices]
        lander_vertices = [transform_vertex(x, y, self.lander.angle, self.lander.position) for x, y in scaled_vertices]
        lunarlander_msg=Polygon()
        for x,y in lander_vertices:
            lunarlander_msg.points.append(Point32(x=x, y=y))
        # lunarlander_msg=Polygon(points=[Point32(x=lander_vertices[0][0], y=lander_vertices[0][1]), Point32(x=lander_vertices[1][0], y=lander_vertices[1][1]), Point32(x=lander_vertices[2][0], y=lander_vertices[2][1])])
        self._publish_msgs(thruster_msg=Int32(data=action), terrain_msg=terrain_msg, lunarlander_msg=lunarlander_msg)
        return statet, reward, terminated, truncated, info


