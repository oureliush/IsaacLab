# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Humanoid locomotion environment.
"""

import gymnasium as gym

from . import agents
#from .leg_env import Leg2LiteEnv, Leg2LiteEnvCfg

##
# Register Gym environments.
##

gym.register(
    id="Isaac-DLeg-Direct-v0",
    entry_point=f"{__name__}.walking_env_test:Dual_Leg_Test_Env",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.walking_env_test:Dual_Leg_test_1_EnvCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DLegPPORunnerCfg",
    },
)
