import gym
from gym.utils.play import play, display_arr, PlayPlot
import sys, gym, time
import pygame
import os
import numpy as np
import warnings
warnings.filterwarnings("ignore")

env = gym.make('LunarLander-v2', gravity= -5.0, turbulence_power = 1, render_mode="rgb_array")
env.reset()
env.render()
def callback(obs_t, obs_tp1, action, rew, terminated, truncated, info):
    if terminated and rew==100:
        print("\x1b[6;30;42mSuccess!!\x1b[0m")
    elif terminated: 
        print("Crashed!!")   
    return [rew]
plotter = PlayPlot(callback, 150, ["reward"])
play(env,callback = plotter.callback
                        ,keys_to_action={
                          "w": 2,
                          "a": 1,
                          "d": 3,
                          }, noop=0)

