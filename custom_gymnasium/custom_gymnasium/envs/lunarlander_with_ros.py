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
from gymnasium.envs.box2d.lunar_lander import VIEWPORT_W, VIEWPORT_H, SCALE, INITIAL_RANDOM, LEG_AWAY, LEG_DOWN, LEG_W, LEG_H, LEG_SPRING_TORQUE, LANDER_POLY, FPS, MAIN_ENGINE_POWER, SIDE_ENGINE_AWAY, SIDE_ENGINE_HEIGHT, SIDE_ENGINE_POWER

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



class LunarLander_ROS(LunarLander):
    
    def __init__(self):
        super().__init__()

    
    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ):
        """
        The reset for
        1. generate the terrain
        2. generate the uav lander

        Args:

        Returns:
            
        """
        super().reset(seed=seed)

