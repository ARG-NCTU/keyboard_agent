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
from gymnasium.envs.box2d.lunar_lander import VIEWPORT_W, VIEWPORT_H, SCALE, INITIAL_RANDOM, LEG_AWAY, LEG_DOWN, LEG_W, LEG_H, LEG_SPRING_TORQUE, LANDER_POLY

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

class UavLander(LunarLander):
    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ):
        super().reset(seed=seed)
        self._destroy()
        self.world.contactListener_keepref = ContactDetector(self)
        self.world.contactListener = self.world.contactListener_keepref
        self.game_over = False
        self.prev_shaping = None

#=========================================terrain==============================================
        W = VIEWPORT_W / SCALE
        H = VIEWPORT_H / SCALE
        CHUNKS = 30 # originally 11
        width_para = 6

        height = self.np_random.uniform(0, H / 2, size=(CHUNKS + 1,)) # ground height, random points(CHUNKS + 1), 
                                                                      # from 0 to H / 2
        chunk_x = [W / (CHUNKS - 1) * i for i in range(CHUNKS)] # ground got CHUNKS point distributed evenly

        # helipad is not in the middle of the ground, it is in the middle of the screen
        

        # self.helipad_y = H / 4 # originally H / 4
        # height[CHUNKS // 2 - 2] = self.helipad_y
        # height[CHUNKS // 2 - 1] = self.helipad_y
        # height[CHUNKS // 2 + 0] = self.helipad_y
        # height[CHUNKS // 2 + 1] = self.helipad_y
        # height[CHUNKS // 2 + 2] = self.helipad_y
        smooth_y = [
            0.33 * (height[i - 1] + height[i + 0] + height[i + 1])
            for i in range(CHUNKS)
        ]
        height[CHUNKS // 2 + 0] = self.np_random.uniform(0, H *3 / 5, size = 1)[0]
        self.helipad_y = height[CHUNKS // 2 + 0]

        width_node = (CHUNKS // width_para - 1) // 2
        self.helipad_x1 = chunk_x[CHUNKS // 2 - width_node]
        self.helipad_x2 = chunk_x[CHUNKS // 2 + width_node]

        smooth_y[CHUNKS // 2 + 0] = self.helipad_y
        for i in range(width_node):
            smooth_y[CHUNKS // 2 - (i+1)] = self.helipad_y
            smooth_y[CHUNKS // 2 + (i+1)] = self.helipad_y

        self.moon = self.world.CreateStaticBody(
            shapes=edgeShape(vertices=[(0, 0), (W, 0)])
        )
        self.sky_polys = []
        for i in range(CHUNKS - 1):
            p1 = (chunk_x[i], smooth_y[i])
            p2 = (chunk_x[i + 1], smooth_y[i + 1])
            self.moon.CreateEdgeFixture(vertices=[p1, p2], density=0, friction=0.1)
            self.sky_polys.append([p1, p2, (p2[0], H), (p1[0], H)])
#===============================================================================================
        self.moon.color1 = (0.0, 0.0, 0.0)
        self.moon.color2 = (0.0, 0.0, 0.0)

        initial_y = VIEWPORT_H / SCALE
        self.lander: Box2D.b2Body = self.world.CreateDynamicBody(
            position=(VIEWPORT_W / SCALE / 2, initial_y),
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
        self.lander.ApplyForceToCenter(
            (
                self.np_random.uniform(-INITIAL_RANDOM, INITIAL_RANDOM),
                self.np_random.uniform(-INITIAL_RANDOM, INITIAL_RANDOM),
            ),
            True,
        )

        self.legs = []
        for i in [-1, +1]:
            leg = self.world.CreateDynamicBody(
                position=(VIEWPORT_W / SCALE / 2 - i * LEG_AWAY / SCALE, initial_y),
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