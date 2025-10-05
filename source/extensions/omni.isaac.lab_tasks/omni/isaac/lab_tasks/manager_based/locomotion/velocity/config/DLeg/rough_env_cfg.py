from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.utils import configclass

import omni.isaac.lab_tasks.manager_based.locomotion.velocity.mdp as mdp
from omni.isaac.lab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
)

from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm

from omni.isaac.lab_assets.custom_robots import DUAL_LEGS_CFG, DUAL_LEGS_CFG_UNREALISTIC, DUAL_LEGS_C_CFG, STRAIGHT_DUAL_LEGS_CFG, STRAIGHT_DUAL_LEGS_CFG_SMALL_FEET


@configclass
class DLegTerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    body_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=["base_link", "L_Hip_1", "R_Hip_1"]), "threshold": 1.0},
    )

    '''
    body_contact = DoneTerm(
    func=mdp.illegal_contact,
    params={
        "sensor_cfg": SceneEntityCfg(
            "contact_forces",
            body_names=[r"^(?!L_Foot_1$)(?!R_Foot_1$).+"],  # Regex: match any body name except L_Foot_1 and R_Foot_1
        ),
        "threshold": 1.0,
    },
)
    '''

@configclass
class DLegRewards(RewardsCfg):
    """Reward terms for the MDP."""
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    lin_vel_z_l2 = None
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": 0.5},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp, weight=1.0, params={"command_name": "base_velocity", "std": 0.5}
    )
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.25,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*Foot_1"]),
            "threshold": 0.4,
        },
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.5,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["R_Foot_1", "L_Foot_1"]),
            "asset_cfg": SceneEntityCfg("robot", body_names=["R_Foot_1", "L_Foot_1"]),
        },
    )
    
    leg_roll_deviation = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-1.25e-14,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["Revolute_5", "Revolute_13"]),
        },
    )
    
    

@configclass
class DLegRewards2(RewardsCfg):
    """Reward terms for the MDP."""
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=2.5,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*Foot_1"]),
            "threshold": 0.4,
        },
    )
    '''
    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["hip_abduction_.*", "hip_rotation_.*"])},
    )
    joint_deviation_toes = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["toe_joint_.*"])},
    )
    # penalize toe joint limits
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names="toe_joint_.*")},
    )
    '''

@configclass
class DLegRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: DLegRewards = DLegRewards()
    terminations: DLegTerminationsCfg = DLegTerminationsCfg()

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        self.scene.robot = DUAL_LEGS_C_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.events.physics_material = None
        self.events.add_base_mass = None 
        self.events.base_external_force_torque.params["asset_cfg"].body_names = [".*base_link"]
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }
        self.events.push_robot = None

        self.events.base_external_force_torque = None
        # Rewards
        self.rewards.undesired_contacts = None
        self.rewards.flat_orientation_l2.weight = -1.0
        self.rewards.dof_torques_l2.weight = 0.0
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.25e-7

        self.rewards.flat_orientation_l2 = None
        self.rewards.dof_acc_l2 = None


        '''
        # rewards
        self.rewards.undesired_contacts = None
        self.rewards.dof_torques_l2.weight = -5.0e-6
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 1.0
        self.rewards.action_rate_l2.weight *= 1.5
        self.rewards.dof_acc_l2.weight *= 1.5
        '''
        
        # Commands
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)

@configclass
class DLegRoughEnvCfg_PLAY(DLegRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None
