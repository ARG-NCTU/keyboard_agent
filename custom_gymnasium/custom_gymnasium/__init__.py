from gymnasium.envs.registration import registry, register, make, spec

register(
    id="UavLander-v1",
    entry_point="custom_gymnasium.envs.box2d:UavLander_v1",
    max_episode_steps=4096,
    # reward_threshold=200,
)