from omni.isaac.lab.envs import DirectRLEnv
from omni.isaac.lab.envs import DirectRLEnvCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.actuators import ImplicitActuatorCfg, IdealPDActuatorCfg, DelayedPDActuatorCfg

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation, ArticulationCfg
from omni.isaac.lab.envs import DirectRLEnv, DirectRLEnvCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.math import sample_uniform

from omni.isaac.lab.managers import EventTermCfg as EventTerm
import omni.isaac.lab.envs.mdp as mdp
from omni.isaac.lab.managers import SceneEntityCfg 
from omni.isaac.lab.utils.noise import NoiseModelWithAdditiveBiasCfg, GaussianNoiseCfg

from omni.isaac.lab.sensors import CameraCfg, ContactSensorCfg, RayCasterCfg, patterns, ContactSensor, ImuCfg, Imu

from omni.isaac.lab_assets.custom_robots import TWOJLRGEN2_CFG 

from collections.abc import Sequence
import torch
import numpy as np

import sys
import os


@configclass
class EventCfg:
  robot_physics_material = EventTerm(
      func=mdp.randomize_rigid_body_material,
      mode="startup",
      params={
          "asset_cfg": SceneEntityCfg("a_JLR_2", body_names=".*"),
          "static_friction_range": (0.7, 2.0),
          "dynamic_friction_range": (0.7, 2.0),
          "restitution_range": (1.0, 1.0),
          "num_buckets": 250,
      },
  )

  robot_mass = EventTerm(
      func=mdp.randomize_rigid_body_mass,
      mode="startup",
      params={
          "asset_cfg": SceneEntityCfg("a_JLR_2",  body_names=".*"),
          "mass_distribution_params": (0.75, 1.25),
          "operation": "scale",
          "distribution": "log_uniform",
      },
  )

  #robot_joint_stiffness_and_damping = EventTerm(
  #    func=mdp.randomize_actuator_gains,
  #   mode="reset",
  #    params={
  #       "asset_cfg": SceneEntityCfg("Leg_2Lite", joint_names=".*"),
  #       "stiffness_distribution_params": (0.3, 2.5),
  #        "damping_distribution_params": (0.3, 2.5),
  #        "operation": "scale",
  #        "distribution": "log_uniform",
  #    },
  #)
   
  knee_joint_properties = EventTerm(
      func=mdp.randomize_joint_parameters,
      mode="startup",
      params={
          "asset_cfg": SceneEntityCfg("a_JLR_2", joint_names="Revolute_23"),
#          "upper_limit_distribution_params": (-0.25, 0.25),
#          "lower_limit_distribution_params": (-0.25, 0.25),
          "friction_distribution_params": (0.0, 0.2),
          "operation": "add",
          "distribution": "gaussian",
      },
  )

  torque_rando = EventTerm(
      func=mdp.randomize_joint_parameters,
      mode="startup",
      params={
          "asset_cfg": SceneEntityCfg("a_JLR_2", joint_names=".*"),
          "torque_limit_distribution_params": (0.5, 0.85),
          "operation": "scale",
          "distribution": "log_uniform",
      },
  )
  
  foot_joint_properties = EventTerm(
      func=mdp.randomize_joint_parameters,
      mode="startup",
      params={
          "asset_cfg": SceneEntityCfg("a_JLR_2", joint_names="Revolute_26"),
#          "upper_limit_distribution_params": (-0.25, 0.25),
#          "lower_limit_distribution_params": (-0.25, 0.25),
          "friction_distribution_params": (0.0, 0.2),
          "operation": "add",
          "distribution": "gaussian",
      },
  )


  add_all_joint_default_pos = EventTerm(
    func=mdp.randomize_joint_default_pos,
    mode="startup",
    params={
        "asset_cfg": SceneEntityCfg("a_JLR_2", joint_names=[".*"]), 
        "pos_distribution_params": (-0.05, 0.05),
        "operation": "add"
      },
  )

#official isaaclab way
'''
  add_all_joint_default_pos = EventTerm(
    func=mdp.reset_joints_by_offset,
    mode="startup",
    params={
        "asset_cfg": SceneEntityCfg("Leg_2Lite", joint_names=[".*"]), 
        "position_range": (-0.05, 0.05),
        "velocity_range": (0, 0),
      },
  )
'''

  #reset_gravity = EventTerm(
  #    func=mdp.randomize_physics_scene_gravity,
  #    mode="interval",
  #    is_global_time=True,
  #    interval_range_s=(36.0, 36.0),  # time_s = num_steps * (decimation * dt)
  #    params={
  #        "gravity_distribution_params": ([0.0, 0.0, 0.0], [0.0, 0.0, 0.4]),
  #        "operation": "add",
  #        "distribution": "gaussian",
  #    },
  #)

@configclass 
class TWOJLREnvCfg(DirectRLEnvCfg):

    decimation = 4
    episode_length_s = 20.0
    action_space = 2
    observation_space = 11
    state_space = 0

    # Simulation configuration
    sim: SimulationCfg = SimulationCfg(
        #device="cuda:0",
        dt=1 / 200,
        render_interval=decimation,
        gravity=(0.0, 0.0, -9.81),
    )

    # Scene configuration
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=8192,
        env_spacing=2.0,
        replicate_physics=True
    )

    # robot
    robot_cfg: ArticulationCfg = TWOJLRGEN2_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    
    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*", history_length=3, update_period=0.005, track_air_time=True
    )
    
    baseimu: ImuCfg = ImuCfg(
        prim_path="/World/envs/env_.*/Robot/BNO085_IMU_v1_1", gravity_bias=(0, 0, 0),
    )

    footimu: ImuCfg = ImuCfg(
        prim_path="/World/envs/env_.*/Robot/Foot_IMU_1", gravity_bias=(0, 0, 0),
    )

    knee_dof_name = "Revolute_23"
    foot_dof_name = "Revolute_26"
  
    events: EventCfg = EventCfg() 

    # at every time-step add gaussian noise + bias. The bias is a gaussian sampled at reset
    #action_noise_model: NoiseModelWithAdditiveBiasCfg = NoiseModelWithAdditiveBiasCfg(
    #  noise_cfg=GaussianNoiseCfg(mean=0.0, std=0.05, operation="add"),
    #  bias_noise_cfg=GaussianNoiseCfg(mean=0.0, std=0.015, operation="abs"),
    #)

    # at every time-step add gaussian noise + bias. The bias is a gaussian sampled at reset
    observation_noise_model: NoiseModelWithAdditiveBiasCfg = NoiseModelWithAdditiveBiasCfg(
     noise_cfg=GaussianNoiseCfg(mean=0.0, std=0.002, operation="add"),
      bias_noise_cfg=GaussianNoiseCfg(mean=0.0, std=0.0001, operation="abs"),
    )




class TWOJLREnv(DirectRLEnv):
    cfg: TWOJLREnvCfg

    def __init__(self, cfg: TWOJLREnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        
        self.robot_dof_lower_limits = self.leg.data.soft_joint_pos_limits[0, :, 0].to(device=self.device) 
        self.robot_dof_upper_limits = self.leg.data.soft_joint_pos_limits[0, :, 1].to(device=self.device)

        print(self.robot_dof_lower_limits)
        print(self.robot_dof_upper_limits)

        self.torque_limits = torch.tensor([34.74, 23.16], device=self.device)  # Torque limits for "Knee" and "Foot" actuators
        
        self.motor_torque_limits = torch.tensor([3.86, 3.86], device=self.device)  # Torque limits for "Knee" and "Foot" motors
        self.gearbox_ratio = torch.tensor([12.00, 8.00], device=self.device)

        self.actions_high = torch.tensor([1.0, 1.0], device=self.device)
        self.actions_low = torch.tensor([-1.0, -1.0], device=self.device)

        self._knee_dof_idx, _ = self.leg.find_joints(self.cfg.knee_dof_name)
        self._foot_dof_idx, _ = self.leg.find_joints(self.cfg.foot_dof_name)

        self.joint_pos = self.leg.data.joint_pos.to(device=self.device)
        self.joint_vel = self.leg.data.joint_vel.to(device=self.device)

        self.fallen_over = torch.zeros(self.cfg.scene.num_envs, dtype=torch.bool, device=self.sim.device)

        self.is_knee_joint_outside_20deg = torch.zeros(self.cfg.scene.num_envs, dtype=torch.bool, device=self.sim.device)
        self.is_foot_joint_outside_20deg = torch.zeros(self.cfg.scene.num_envs, dtype=torch.bool, device=self.sim.device)
        self.is_base_outside_20deg = torch.zeros(self.cfg.scene.num_envs, dtype=torch.bool, device=self.sim.device)

        # — new: sample gear ratios once at startup —
        n_envs = self.cfg.scene.num_envs
        base_ratio = torch.tensor([12.0, 8.0], device=self.device)            # nominal [knee, foot]
        ineff   = torch.rand((n_envs, 2), device=self.device) * 0.2 + 0.8  # uniform in [0.8,1.0]
        self.gearbox_ratio = base_ratio.unsqueeze(0) * ineff              # shape [n_envs,2]

        print("Contact sensor body names:", self._contact_sensor.body_names) #actually trained well with only base_link and foot_1 as contact reporters

        self._otherthanfoot_ids, _ = self._contact_sensor.find_bodies("^(?!Foot_1$).*$")
        self._foot_id, _ = self._contact_sensor.find_bodies("Foot_1")
        self._baseimu_id, _ = self._contact_sensor.find_bodies("Foot_1")
        
        self.scene.update(dt=self.physics_dt) # Add this line

        print("test")


    def _setup_scene(self):
        self.leg = Articulation(self.cfg.robot_cfg)
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        
        self._contact_sensor = ContactSensor(self.cfg.contact_sensor)
        self._baseimu = Imu(self.cfg.baseimu)
        self._footimu = Imu(self.cfg.footimu)

        self.scene.sensors["contact_sensor"] = self._contact_sensor
        self.scene.sensors["baseimu"] = self._baseimu
        self.scene.sensors["footimu"] = self._footimu
        
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[])
        self.scene.articulations["a_JLR_2"] = self.leg
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def rescale_actions(self, low, high, action):
        d = (high - low) / 2.0
        m = (high + low) / 2.0
        scaled_action = action * d + m
        return scaled_action

    def _pre_physics_step(self, actions: torch.Tensor):
      self.actions = actions.clone().to(device=self.device)
      self.clamped_actions = torch.clamp(self.actions, -1.0, 1.0)
      self.rescaled_actions = self.rescale_actions(self.actions_low, self.actions_high, self.clamped_actions)

      #print(self.actions)

      self.idealtorques = self.clamped_actions * self.torque_limits

      motor_torque = self.clamped_actions * self.motor_torque_limits    # [n,2]
      self.current_torques = motor_torque * self.gearbox_ratio  # applies per-env ratio

      #print(self.actions)
      #print(self.current_torques)

      self.current_targets = (
        0.5 * (self.actions + 1.0) * (self.robot_dof_upper_limits - self.robot_dof_lower_limits)
        + self.robot_dof_lower_limits
      )

    def _apply_action(self):
      self.leg.set_joint_effort_target(self.current_torques)

    def _get_observations(self) -> dict:
        joint_pos = self.joint_pos.to(device=self.device)
        joint_vel = self.joint_vel.to(device=self.device)

        base_imu_orient = self._baseimu.data.quat_w

        #print(base_imu_orient)

        qr = base_imu_orient[:, [0]]
        qi = base_imu_orient[:, [1]]
        qj = base_imu_orient[:, [2]]
        qk = base_imu_orient[:, [3]]

        sqr = torch.square(qr)
        sqi = torch.square(qi)
        sqj = torch.square(qj)
        sqk = torch.square(qk)
        
        pitch = torch.atan2(2.0 * (qj * qk + qi * qr),  (-sqi - sqj + sqk + sqr))
        self.pitch = torch.rad2deg(pitch)

        #print(self.pitch)

        imu_lin_acc = self._footimu.data.lin_acc_b
        imu_ang_vel = self._footimu.data.ang_vel_b

        # Round the IMU observations to 2 decimal places
        imu_lin_acc = torch.round(imu_lin_acc * 100) / 100
        imu_ang_vel = torch.round(imu_ang_vel * 100) / 100
        
        #print(joint_pos[:, self._knee_dof_idx[0]].unsqueeze(dim=1))
        #print(imu_ang_vel)
        
        obs = torch.cat(
            (
                joint_pos[:, self._knee_dof_idx[0]].unsqueeze(dim=1), 
                joint_vel[:, self._knee_dof_idx[0]].unsqueeze(dim=1),
                joint_pos[:, self._foot_dof_idx[0]].unsqueeze(dim=1), 
                joint_vel[:, self._foot_dof_idx[0]].unsqueeze(dim=1),
                imu_lin_acc,
                imu_ang_vel,
                self.pitch,
            ),
            dim=-1,
        )

        test_input = torch.tensor([0.6789, -1.2345, 4.5678, -3.2109, 
                                  2.3456, -0.9876, 1.2345, 0.4321, 
                                  -2.3456, 3.1415, -1.6180, 2.7182]).to(device=self.device)
        # If your models expect a batch dimension, add one:
        test_input = test_input.unsqueeze(0)  # Shape: [1, 12]
        
        observations = {"policy": obs}
        return observations

    def quat_rotate(self, q, v):
        q_conj = torch.cat([-q[:, :3], q[:, 3:]], dim=1).to(device=self.device)
        v_q = torch.cat([v.unsqueeze(0).repeat(q.shape[0], 1), torch.zeros((q.shape[0], 1), device=self.device)], dim=1).to(device=self.device)
        result = self.quaternion_multiply(self.quaternion_multiply(q, v_q), q_conj)
        return result[:, :3]


    def quaternion_multiply(self, q, r):
        x1, y1, z1, w1 = q.unbind(-1)
        x2, y2, z2, w2 = r.unbind(-1)
        return torch.stack((
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        ), dim=-1).to(device=self.device)
        
    
    def _get_rewards(self) -> torch.Tensor:
        contact_threshold = 30.0
        degree_to_rad = torch.pi / 180.0



        # Convert degree thresholds to radians
        base_10deg = 10.0 * degree_to_rad
        base_20deg = 20.0 * degree_to_rad
        joint_10deg = 10.0 * degree_to_rad
        joint_20deg = 20.0 * degree_to_rad

        joint_pos = self.leg.data.joint_pos.to(device=self.device)   # shape: [num_envs, 2]
        joint_vel = self.leg.data.joint_vel.to(device=self.device)   # shape: [num_envs, 2]
        
        knee_pos = joint_pos[:, self._knee_dof_idx[0]]
        foot_pos = joint_pos[:, self._foot_dof_idx[0]]

        base_orientation = self.pitch[:, 0]
        #print(base_orientation)
        base_orientation = torch.abs(base_orientation)


        
        forces = self._contact_sensor.data.net_forces_w_history[:, -1, self._foot_id, :]
        forces = forces.squeeze(1)  # remove singleton dimension if present
        force_magnitude = torch.norm(forces, dim=-1)




        # Conditions needed to be met for positive rewards
        is_base_within_10deg = base_orientation < base_10deg
        is_knee_joint_within_10deg = torch.abs(knee_pos) <= joint_10deg
        is_foot_joint_within_10deg = torch.abs(foot_pos) <= joint_10deg
        foot_contact = force_magnitude > contact_threshold
        eligible_for_rewards = (
            is_base_within_10deg 
            & is_knee_joint_within_10deg 
            & is_foot_joint_within_10deg 
            & foot_contact
        )
        
        maximum_reward = 5.0
        good_robot = 5.0 + ((base_orientation/10) * maximum_reward)
        bad_robot = -5.0

        # The conditions that reset robot
        self.is_knee_joint_outside_20deg = torch.abs(knee_pos) > joint_20deg
        self.is_foot_joint_outside_20deg = torch.abs(foot_pos) > joint_20deg
        self.is_base_outside_20deg = base_orientation > base_20deg
        
        # using scalars here preserves the 1-D shape automatically
        reward = torch.where(eligible_for_rewards, good_robot, bad_robot)

        # ensure it’s on the right device & dtype
        reward = reward.to(device=self.device, dtype=force_magnitude.dtype)

        # Give a bonus when the foot is in contact, and a penalty if it’s not.
        # contact_reward = torch.where(foot_contact, torch.ones_like(up_cos), -torch.ones_like(up_cos)).to(device=self.device)

        #print("Reward shape:", reward.shape)

        # placeholder reward: zeros for each env
        #reward = torch.zeros_like(force_magnitude)

        return reward


    def _reset_idx(self, env_ids: torch.Tensor | None):
        
        if env_ids is None:
            env_ids = self.leg._ALL_INDICES
        super()._reset_idx(env_ids)
        self.leg.reset(env_ids)

        joint_pos = self.leg.data.default_joint_pos[env_ids].to(device=self.device)
        joint_vel = self.leg.data.default_joint_vel[env_ids].to(device=self.device)

        default_root_state = self.leg.data.default_root_state[env_ids].to(device=self.device)
        default_root_state[:, :3] += self.scene.env_origins[env_ids].to(device=self.device)

        self.joint_pos[env_ids] = joint_pos
        self.joint_vel[env_ids] = joint_vel

        self.leg.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.leg.write_root_com_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.leg.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        #print("reset")

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        time_outs = self.episode_length_buf >= self.cfg.episode_length_s * (1 / self.cfg.sim.dt)
        net_contact_forces = self._contact_sensor.data.net_forces_w_history
        illegal_contact = torch.any(torch.max(torch.norm(net_contact_forces[:, :, self._otherthanfoot_ids], dim=-1), dim=1)[0] > 1.0, dim=1)
        died = torch.any(torch.max(torch.norm(net_contact_forces[:, :, self._foot_id], dim=-1), dim=1)[0] < 30.0, dim=1)
        die = (
            illegal_contact
            | died
            | self.is_base_outside_20deg
            | self.is_foot_joint_outside_20deg
            | self.is_knee_joint_outside_20deg
        )
        
        return die, time_outs
