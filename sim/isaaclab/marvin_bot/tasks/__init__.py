"""Marvin Bot tasks, registered with Isaac Lab through the ``isaaclab.tasks`` entry point."""

import gymnasium as gym

gym.register(
    id="MarvinBot-Velocity-Flat",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.velocity_env_cfg:MarvinBotFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents:MarvinBotFlatPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="MarvinBot-Velocity-Rough",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.velocity_env_cfg:MarvinBotRoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents:MarvinBotRoughPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
