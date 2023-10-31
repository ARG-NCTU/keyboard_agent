from gymnasium.envs.registration import registry, register, make, spec

register(
    id="UavLander-v0",
    entry_point="uav_gym.envs.box2d:UavLander",
    max_episode_steps=4096,
    # reward_threshold=200,
)