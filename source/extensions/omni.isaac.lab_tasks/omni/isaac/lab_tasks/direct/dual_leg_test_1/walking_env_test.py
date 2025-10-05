from omni.isaac.lab.envs import DirectRLEnv
from omni.isaac.lab.envs import DirectRLEnvCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.terrains import TerrainImporterCfg

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

from omni.isaac.lab_assets.custom_robots import DUAL_LEGS_CFG, DUAL_LEGS_CFG_UNREALISTIC

import gymnasium as gym

from collections.abc import Sequence
import torch
import numpy as np

import sys
import os



@configclass 
class Dual_Leg_test_1_EnvCfg(DirectRLEnvCfg):

    decimation = 4
    episode_length_s = 20.0
    action_space = 10
    observation_space = 41
    state_space = 0
    action_scale = 0.5

    x_vel_reward_scale: float = 1.0
    yaw_rate_reward_scale: float = 1.0

    # Simulation configuration
    sim: SimulationCfg = SimulationCfg(
        #device="cuda:0",
        dt=1 / 200,
        render_interval=decimation,
        gravity=(0.0, 0.0, -9.81),
    )

    # Scene configuration
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096,
        env_spacing=2.0,
        replicate_physics=True
    )

    # robot
    robot_cfg: ArticulationCfg = DUAL_LEGS_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*", history_length=3, update_period=0.005, track_air_time=True
    )

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

class Dual_Leg_Test_Env(DirectRLEnv):
    cfg: Dual_Leg_test_1_EnvCfg

    def __init__(self, cfg: Dual_Leg_test_1_EnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        
        self.robot_dof_lower_limits = self.legs.data.soft_joint_pos_limits[0, :, 0].to(device=self.device) 
        self.robot_dof_upper_limits = self.legs.data.soft_joint_pos_limits[0, :, 1].to(device=self.device)

        print(self.robot_dof_lower_limits)
        print(self.robot_dof_upper_limits)

        self._actions = torch.zeros(self.num_envs, gym.spaces.flatdim(self.single_action_space), device=self.device)
        self._previous_actions = torch.zeros(
            self.num_envs, gym.spaces.flatdim(self.single_action_space), device=self.device
        )

        self.actions_high = torch.tensor([1.0, 1.0], device=self.device)
        self.actions_low = torch.tensor([-1.0, -1.0], device=self.device)

        self.joint_pos = self.legs.data.joint_pos.to(device=self.device)
        self.joint_vel = self.legs.data.joint_vel.to(device=self.device)

        # X/Y linear velocity and yaw angular velocity commands
        self._commands = torch.zeros(self.num_envs, 2, device=self.device)

        self.fallen_over = torch.zeros(self.cfg.scene.num_envs, dtype=torch.bool, device=self.sim.device)

        self._base_id, _ = self._contact_sensor.find_bodies("base_link")
        #self._hip_id, _ = self._contact_sensor.find_bodies("base_link")

        # Logging
        self._episode_sums = {
            key: torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
            for key in [
                "track_x_vel_exp",
                "track_yaw_rate_exp",
                "lin_vel_z_l2",
                "ang_vel_xy_l2",
            ]
        }

        #print("test")

    def _setup_scene(self):
        print("test2")
        self.legs = Articulation(self.cfg.robot_cfg)
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        
        self.scene.articulations["Dual_Leg_URDF_Test_1"] = self.legs

        self._contact_sensor = ContactSensor(self.cfg.contact_sensor)
        self.scene.sensors["contact_sensor"] = self._contact_sensor
        
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[])
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)
        print("test3")

    def rescale_actions(self, low, high, action):
        d = (high - low) / 2.0
        m = (high + low) / 2.0
        scaled_action = action * d + m
        return scaled_action

    def _pre_physics_step(self, actions: torch.Tensor):
      self._actions = actions.clone().to(device=self.device)
      #self.clamped_actions = torch.clamp(self.actions, -1.0, 1.0)
      #self.rescaled_actions = self.rescale_actions(self.actions_low, self.actions_high, self.clamped_actions)

      #print(self.actions)\
      #print(self.current_torques)

      self.current_targets = (
        0.5 * (self.actions + 1.0) * (self.robot_dof_upper_limits - self.robot_dof_lower_limits)
        + self.robot_dof_lower_limits
      )

      self._processed_actions = self.cfg.action_scale * self.actions + self.legs.data.default_joint_pos

    def _apply_action(self):
      self.legs.set_joint_effort_target(self._processed_actions)

    def _get_observations(self) -> dict:
        self._previous_actions = self.actions.clone()
        obs = torch.cat(
            (
                self.legs.data.root_com_lin_vel_b,
                self.legs.data.root_com_ang_vel_b,
                self.legs.data.projected_gravity_b,
                self.legs.data.joint_pos,
                self.legs.data.joint_vel,
                self.actions,
                self._commands,
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

        base_quats = self.legs.data.root_quat_w.to(device=self.device)
        # In this simulation the world “up” is defined as the x-axis.
        up_vec = torch.tensor([1, 0, 0], dtype=torch.float32, device=self.sim.device)
        # quat_rotate rotates the up_vec by the base quaternion.
        robot_up = self.quat_rotate(base_quats, up_vec)  # shape: [num_envs, 3]
        
        # Compute the cosine of the angle between robot_up and world up.
        # (Since both are unit vectors, dot product equals cosine.)
        # Here we assume alignment is best when robot_up is [1, 0, 0] so we use the first component.
        up_cos = robot_up[:, 0]

        # Track commanded x velocity and yaw rate

        # x velocity tracking: compare commanded x velocity (self._commands[:, 0])
        # to measured x velocity (self.legs.data.root_com_lin_vel_b[:, 0])
        x_vel_error = torch.square(self._commands[:, 0] - self.legs.data.root_com_lin_vel_b[:, 0])
        x_vel_reward = torch.exp(-x_vel_error / 0.25) * self.cfg.x_vel_reward_scale * self.step_dt

        # yaw rate tracking: compare commanded yaw (self._commands[:, 1])
        # to measured yaw angular velocity (self.legs.data.root_com_ang_vel_b[:, 2])
        yaw_rate_error = torch.square(self._commands[:, 1] - self.legs.data.root_com_ang_vel_b[:, 2])
        yaw_rate_reward = torch.exp(-yaw_rate_error / 0.25) * self.cfg.yaw_rate_reward_scale * self.step_dt

        ang_vel_error = torch.sum(torch.square(self.legs.data.root_com_ang_vel_b[:, :2]), dim=1)

        # z velocity tracking
        z_vel_error = torch.square(self.legs.data.root_com_lin_vel_b[:, 2])

        rewards = {
            "track_x_vel_exp": x_vel_reward,
            "track_yaw_rate_exp": yaw_rate_reward,
            #"lin_vel_z_l2": z_vel_error * -2.0 * self.step_dt,
            "ang_vel_xy_l2": ang_vel_error * -0.05 * self.step_dt,
        }

        total_reward = torch.sum(torch.stack(list(rewards.values())), dim=0)
        
        # Logging
        for key, value in rewards.items():
            self._episode_sums[key] += value
            
        '''
        reward = (
            x_vel_reward,
            + yaw_rate_reward,
        ).to(device=self.device)
        '''
        #total_reward[self.died] = -200.0
        self.fallen_over = up_cos < 0.5

        return total_reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        net_contact_forces = self._contact_sensor.data.net_forces_w_history
        self.died = torch.any(torch.max(torch.norm(net_contact_forces[:, :, self._base_id], dim=-1), dim=1)[0] > 1.0, dim=1)
        
        self.died = self.died | torch.any(torch.max(torch.norm(net_contact_forces[:, :, self._hip_id], dim=-1), dim=1)[0] > 1.0, dim=1)

        return self.died, time_out

    def _reset_idx(self, env_ids: torch.Tensor | None): 
        if env_ids is None:
            env_ids = self.legs._ALL_INDICES
        super()._reset_idx(env_ids)
        self.legs.reset(env_ids)

        if len(env_ids) == self.num_envs:
            # Spread out the resets to avoid spikes in training when many environments reset at a similar time
            self.episode_length_buf[:] = torch.randint_like(self.episode_length_buf, high=int(self.max_episode_length))

        self._actions[env_ids] = 0.0
        self._previous_actions[env_ids] = 0.0

        self._commands[:, 0].uniform_(0.0, 1.0).to(device=self.device)
        self._commands[:, 1].uniform_(-1.0, 1.0).to(device=self.device)

        joint_pos = self.legs.data.default_joint_pos[env_ids].to(device=self.device)
        joint_vel = self.legs.data.default_joint_vel[env_ids].to(device=self.device)

        default_root_state = self.legs.data.default_root_state[env_ids].to(device=self.device)
        default_root_state[:, :3] += self.scene.env_origins[env_ids].to(device=self.device)

        self.joint_pos[env_ids] = joint_pos
        self.joint_vel[env_ids] = joint_vel

        self.legs.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.legs.write_root_com_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.legs.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        #print("reset")

        # Logging
        extras = dict()
        for key in self._episode_sums.keys():
            episodic_sum_avg = torch.mean(self._episode_sums[key][env_ids])
            extras["Episode_Reward/" + key] = episodic_sum_avg / self.max_episode_length_s
            self._episode_sums[key][env_ids] = 0.0
        self.extras["log"] = dict()
        self.extras["log"].update(extras)
        extras = dict()
        extras["Episode_Termination/base_contact"] = torch.count_nonzero(self.reset_terminated[env_ids]).item()
        extras["Episode_Termination/time_out"] = torch.count_nonzero(self.reset_time_outs[env_ids]).item()
        self.extras["log"].update(extras)
