import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg, IdealPDActuatorCfg, DelayedPDActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.sensors import RayCasterCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR


DUAL_LEGS_CFG = ArticulationCfg(
  spawn=sim_utils.UsdFileCfg(
    usd_path=f"/home/ourelius/Desktop/DL_Test_1.usd",
    activate_contact_sensors=True,
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
      disable_gravity=False,
      retain_accelerations=False,
      max_linear_velocity=1000.0,
      max_angular_velocity=1000.0,
      max_depenetration_velocity=1.0,
    ),
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
      enabled_self_collisions=False,
      solver_position_iteration_count=4,
      solver_velocity_iteration_count=4,
    ),
  ),
  init_state=ArticulationCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.88281565037),
    joint_pos={
        "L_Hip_Joint": -0.7, 
        "L_Thigh_Joint": -0.14,
        "Revolute_5": 0.49, 
        "L_Knee_Joint": 1.082,
        "L_Foot_Joint": 0.54,

        "R_Hip_Joint": 0.7, 
        "R_Thigh_Joint": -0.14,
        "Revolute_13": -0.49, 
        "R_Knee_Joint": 1.082,
        "R_Foot_Joint": 0.54,
        
        }
  ),
  actuators={
    "L_Hip": DelayedPDActuatorCfg(
       joint_names_expr=["L_Hip_Joint"],
       effort_limit=40.0,
       velocity_limit=15.7079692679,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Thigh": DelayedPDActuatorCfg(
       joint_names_expr=["L_Thigh_Joint"], 
       effort_limit=40.0,
       velocity_limit=15.7079692679, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Z_R": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_5"],
       effort_limit=23.16,
       velocity_limit=36.6519142919,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Knee": DelayedPDActuatorCfg(
       joint_names_expr=["L_Knee_Joint"], 
       effort_limit=34.74,
       velocity_limit=20.9439510239, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Foot": DelayedPDActuatorCfg(
       joint_names_expr=["L_Foot_Joint"], 
       effort_limit=23.16,
       velocity_limit=36.6519142919, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),


    "R_Hip": DelayedPDActuatorCfg(
       joint_names_expr=["R_Hip_Joint"],
       effort_limit=40.0,
       velocity_limit=15.7079692679,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Thigh": DelayedPDActuatorCfg(
       joint_names_expr=["R_Thigh_Joint"], 
       effort_limit=40.0,
       velocity_limit=15.7079692679, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Z_R": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_13"],
       effort_limit=23.16,
       velocity_limit=36.6519142919,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Knee": DelayedPDActuatorCfg(
       joint_names_expr=["R_Knee_Joint"], 
       effort_limit=34.74,
       velocity_limit=20.9439510239, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Foot": DelayedPDActuatorCfg(
       joint_names_expr=["R_Foot_Joint"], 
       effort_limit=23.16,
       velocity_limit=36.6519142919, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
  },
)

DUAL_LEGS_C_CFG = ArticulationCfg(
  spawn=sim_utils.UsdFileCfg(
    usd_path=f"/home/ourelius/Desktop/DLeg_C.usd",
    activate_contact_sensors=True,
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
      disable_gravity=False,
      retain_accelerations=False,
      max_linear_velocity=1000.0,
      max_angular_velocity=1000.0,
      max_depenetration_velocity=1.0,
    ),
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
      enabled_self_collisions=False,
      solver_position_iteration_count=4,
      solver_velocity_iteration_count=4,
    ),
  ),
  init_state=ArticulationCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.88281565037),
    joint_pos={
        "L_Hip_Joint": -0.7, 
        "L_Thigh_Joint": -0.14,
        "Revolute_5": 0.49, 
        "L_Knee_Joint": 1.082,
        "L_Foot_Joint": 0.54,

        "R_Hip_Joint": 0.7, 
        "R_Thigh_Joint": -0.14,
        "Revolute_13": -0.49, 
        "R_Knee_Joint": 1.082,
        "R_Foot_Joint": 0.54,
        
        }
  ),
  actuators={
    "L_Hip": DelayedPDActuatorCfg(
       joint_names_expr=["L_Hip_Joint"],
       effort_limit=28.0,
       velocity_limit=15.7079692679,
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Thigh": DelayedPDActuatorCfg(
       joint_names_expr=["L_Thigh_Joint"], 
       effort_limit=28.0,
       velocity_limit=15.7079692679, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Z_R": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_5"],
       effort_limit=17.0,
       velocity_limit=36.6519142919,
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Knee": DelayedPDActuatorCfg(
       joint_names_expr=["L_Knee_Joint"], 
       effort_limit=24.318,
       velocity_limit=20.9439510239, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Foot": DelayedPDActuatorCfg(
       joint_names_expr=["L_Foot_Joint"], 
       effort_limit=17.0,
       velocity_limit=36.6519142919, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),


    "R_Hip": DelayedPDActuatorCfg(
       joint_names_expr=["R_Hip_Joint"],
       effort_limit=28.0,
       velocity_limit=15.7079692679,
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Thigh": DelayedPDActuatorCfg(
       joint_names_expr=["R_Thigh_Joint"], 
       effort_limit=28.0,
       velocity_limit=15.7079692679, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Z_R": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_13"],
       effort_limit=17.0,
       velocity_limit=36.6519142919,
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Knee": DelayedPDActuatorCfg(
       joint_names_expr=["R_Knee_Joint"], 
       effort_limit=24.318,
       velocity_limit=20.9439510239, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Foot": DelayedPDActuatorCfg(
       joint_names_expr=["R_Foot_Joint"], 
       effort_limit=17.0,
       velocity_limit=36.6519142919, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
  },
)



DUAL_LEGS_CFG_UNREALISTIC = ArticulationCfg(
  spawn=sim_utils.UsdFileCfg(
    usd_path=f"/home/ourelius/Desktop/DL_Test_1.usd",
    activate_contact_sensors=True,
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
      rigid_body_enabled=True,
      max_linear_velocity=1000.0,
      max_angular_velocity=1000.0,
      max_depenetration_velocity=100.0,
      enable_gyroscopic_forces=True,
    ),
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
      enabled_self_collisions=False,
      solver_position_iteration_count=4,
      solver_velocity_iteration_count=0,
      sleep_threshold=0.005,
      stabilization_threshold=0.001,
    ),
  ),
  init_state=ArticulationCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.88281565037),
    joint_pos={
        "L_Hip_Joint": -0.7, 
        "L_Thigh_Joint": -0.14,
        "Revolute_5": 0.49, 
        "L_Knee_Joint": 1.082,
        "L_Foot_Joint": 0.54,

        "R_Hip_Joint": 0.7, 
        "R_Thigh_Joint": -0.14,
        "Revolute_13": -0.49, 
        "R_Knee_Joint": 1.082,
        "R_Foot_Joint": 0.54,
        
        }
  ),
   actuators={
    "L_Hip": DelayedPDActuatorCfg(
       joint_names_expr=["L_Hip_Joint"],
       effort_limit=40.0,
       velocity_limit=100.00,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
    "L_Thigh": DelayedPDActuatorCfg(
       joint_names_expr=["L_Thigh_Joint"], 
       effort_limit=40.0,
       velocity_limit=100.00, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
    "L_Z_R": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_5"],
       effort_limit=40.00,
       velocity_limit=100.00,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
    "L_Knee": DelayedPDActuatorCfg(
       joint_names_expr=["L_Knee_Joint"], 
       effort_limit=40.00,
       velocity_limit=100.00, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
    "L_Foot": DelayedPDActuatorCfg(
       joint_names_expr=["L_Foot_Joint"], 
       effort_limit=40.00,
       velocity_limit=100.00, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),


    "R_Hip": DelayedPDActuatorCfg(
       joint_names_expr=["R_Hip_Joint"],
       effort_limit=40.0,
       velocity_limit=100.00,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
    "R_Thigh": DelayedPDActuatorCfg(
       joint_names_expr=["R_Thigh_Joint"], 
       effort_limit=40.0,
       velocity_limit=100.00, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
    "R_Z_R": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_13"],
       effort_limit=40.0,
       velocity_limit=100.00,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
    "R_Knee": DelayedPDActuatorCfg(
       joint_names_expr=["R_Knee_Joint"], 
       effort_limit=40.00,
       velocity_limit=100.0, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
    "R_Foot": DelayedPDActuatorCfg(
       joint_names_expr=["R_Foot_Joint"], 
       effort_limit=40.00,
       velocity_limit=100.0, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
  },
)

STRAIGHT_DUAL_LEGS_CFG = ArticulationCfg(
  spawn=sim_utils.UsdFileCfg(
    usd_path=f"/home/ourelius/Desktop/S_DLeg.usd",
    activate_contact_sensors=True,
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
      rigid_body_enabled=True,
      max_linear_velocity=1000.0,
      max_angular_velocity=1000.0,
      max_depenetration_velocity=100.0,
      enable_gyroscopic_forces=True,
    ),
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
      enabled_self_collisions=False,
      solver_position_iteration_count=4,
      solver_velocity_iteration_count=0,
      sleep_threshold=0.005,
      stabilization_threshold=0.001,
    ),
  ),
  init_state=ArticulationCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.76011515024),
    joint_pos={
        "L_Hip": -0.54, 
        "L_Thigh": 0.0,
        "Revolute_20": 0.0, 
        "L_Knee": 1.082,
        "L_Foot": 0.54,

        "R_Hip": 0.54, 
        "R_Thigh": 0.0,
        "Revolute_13": 0.0, 
        "R_Knee": 1.082,
        "R_Foot": 0.54,
        
        }
  ),
  actuators={
    "L_Hip": DelayedPDActuatorCfg(
       joint_names_expr=["L_Hip"],
       effort_limit=40.0,
       velocity_limit=15.7079692679,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Thigh": DelayedPDActuatorCfg(
       joint_names_expr=["L_Thigh"], 
       effort_limit=40.0,
       velocity_limit=15.7079692679, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Z_R": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_20"],
       effort_limit=23.16,
       velocity_limit=36.6519142919,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Knee": DelayedPDActuatorCfg(
       joint_names_expr=["L_Knee"], 
       effort_limit=34.74,
       velocity_limit=20.9439510239, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "L_Foot": DelayedPDActuatorCfg(
       joint_names_expr=["L_Foot"], 
       effort_limit=23.16,
       velocity_limit=36.6519142919, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),


    "R_Hip": DelayedPDActuatorCfg(
       joint_names_expr=["R_Hip"],
       effort_limit=40.0,
       velocity_limit=15.7079692679,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Thigh": DelayedPDActuatorCfg(
       joint_names_expr=["R_Thigh"], 
       effort_limit=40.0,
       velocity_limit=15.7079692679, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Z_R": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_13"],
       effort_limit=23.16,
       velocity_limit=36.6519142919,
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Knee": DelayedPDActuatorCfg(
       joint_names_expr=["R_Knee"], 
       effort_limit=34.74,
       velocity_limit=20.9439510239, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
    "R_Foot": DelayedPDActuatorCfg(
       joint_names_expr=["R_Foot"], 
       effort_limit=23.16,
       velocity_limit=36.6519142919, 
       stiffness=100000.0,
       damping=10000000.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=0,
    ),
  },
)

STRAIGHT_DUAL_LEGS_CFG_SMALL_FEET = ArticulationCfg(
  spawn=sim_utils.UsdFileCfg(
    usd_path=f"/home/ourelius/Desktop/small_feet_S_DLeg.usd",
    activate_contact_sensors=True,
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
      rigid_body_enabled=True,
      max_linear_velocity=1000.0,
      max_angular_velocity=1000.0,
      max_depenetration_velocity=100.0,
      enable_gyroscopic_forces=True,
    ),
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
      enabled_self_collisions=False,
      solver_position_iteration_count=4,
      solver_velocity_iteration_count=0,
      sleep_threshold=0.005,
      stabilization_threshold=0.001,
    ),
  ),
  init_state=ArticulationCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.76011515024),
    joint_pos={
        "L_Hip": -0.54, 
        "L_Thigh": 0.0,
        "Revolute_20": 0.0, 
        "L_Knee": 1.082,
        "L_Foot": 0.54,

        "R_Hip": 0.54, 
        "R_Thigh": 0.0,
        "Revolute_13": 0.0, 
        "R_Knee": 1.082,
        "R_Foot": 0.54,
        
        }
  ),
  actuators={
    "L_Hip": DelayedPDActuatorCfg(
       joint_names_expr=["L_Hip"],
       effort_limit=40.0,
       velocity_limit=15.7079692679,
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,
       max_delay=0,
    ),
    "L_Thigh": DelayedPDActuatorCfg(
       joint_names_expr=["L_Thigh"], 
       effort_limit=40.0,
       velocity_limit=15.7079692679, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,
       max_delay=0,
    ),
    "L_Z_R": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_20"],
       effort_limit=23.16,
       velocity_limit=36.6519142919,
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,
       max_delay=0,
    ),
    "L_Knee": DelayedPDActuatorCfg(
       joint_names_expr=["L_Knee"], 
       effort_limit=34.74,
       velocity_limit=20.9439510239, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,
       max_delay=0,
    ),
    "L_Foot": DelayedPDActuatorCfg(
       joint_names_expr=["L_Foot"], 
       effort_limit=23.16,
       velocity_limit=36.6519142919, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,
       max_delay=0,
    ),
    "R_Hip": DelayedPDActuatorCfg(
       joint_names_expr=["R_Hip"],
       effort_limit=40.0,
       velocity_limit=15.7079692679,
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,
       max_delay=0,
    ),
    "R_Thigh": DelayedPDActuatorCfg(
       joint_names_expr=["R_Thigh"], 
       effort_limit=40.0,
       velocity_limit=15.7079692679, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,
       max_delay=0,
    ),
    "R_Z_R": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_13"],
       effort_limit=23.16,
       velocity_limit=36.6519142919,
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,
       max_delay=0,
    ),
    "R_Knee": DelayedPDActuatorCfg(
       joint_names_expr=["R_Knee"], 
       effort_limit=34.74,
       velocity_limit=20.9439510239, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,
       max_delay=0,
    ),
    "R_Foot": DelayedPDActuatorCfg(
       joint_names_expr=["R_Foot"], 
       effort_limit=23.16,
       velocity_limit=36.6519142919, 
       stiffness=10000000.0,
       damping=100000.0,
       min_delay=0,
       max_delay=0,
    ),
  },
)

LEG_CFG = ArticulationCfg(
  spawn=sim_utils.UsdFileCfg(
    activate_contact_sensors=True,
    usd_path=f"/home/ourelius/Desktop/leg2lite.usd",
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
      disable_gravity=False,
      retain_accelerations=False,
      max_linear_velocity=1000.0,
      max_angular_velocity=1000.0,
      max_depenetration_velocity=1.0,
    ),
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
      enabled_self_collisions=False,
      solver_position_iteration_count=4,
      solver_velocity_iteration_count=4,
    ),
  ),
  init_state=ArticulationCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.9331608),
    joint_pos={"Revolute_3": 0.0, "Revolute_5": 0.0}
  ),
  actuators={
    "Knee_actuator": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_3"],
       effort_limit=23.16,
       velocity_limit=10.471976, # 10*4 RPS Limit (ODrive Speed Unit)
       stiffness=0.0,
       damping=0.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
    "Foot_actuator": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_5"], 
       effort_limit=3.98,
       velocity_limit=15.707963, # 5*4 RPS Limit (ODrive Speed Unit)
       stiffness=0.0,
       damping=0.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
  },
)

'''
      rigid_body_enabled=True,
      max_linear_velocity=1000.0,
      max_angular_velocity=1000.0,
      max_depenetration_velocity=100.0,
      enable_gyroscopic_forces=True,
    ),
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
      enabled_self_collisions=False,
      solver_position_iteration_count=4,
      solver_velocity_iteration_count=4,
      sleep_threshold=0.005,
      stabilization_threshold=0.001,
    ),
  ),
'''

TWOJLR_CFG = ArticulationCfg(
  spawn=sim_utils.UsdFileCfg(
    activate_contact_sensors=True,
    usd_path=f"/home/ourelius/Desktop/2JLR_I.usd",
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
      disable_gravity=False,
      retain_accelerations=False,
      max_linear_velocity=1000.0,
      max_angular_velocity=1000.0,
      max_depenetration_velocity=1.0,
    ),
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
      enabled_self_collisions=False,
      solver_position_iteration_count=4,
      solver_velocity_iteration_count=4,
    ),
  ),
  init_state=ArticulationCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.0),
    joint_pos={"Revolute_14": 0.0, "Revolute_17": 0.0}
  ),
  actuators={
    "Knee_actuator": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_14"],
       effort_limit=34.74,
       velocity_limit=20.9439510239, # 10*4 RPS Limit (ODrive Speed Unit)
       stiffness=0.0,
       damping=0.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
    "Foot_actuator": DelayedPDActuatorCfg(
       joint_names_expr=["Revolute_17"], 
       effort_limit=23.16,
       velocity_limit=36.6519142919, # 5*4 RPS Limit (ODrive Speed Unit)
       stiffness=0.0,
       damping=0.0,
       min_delay=0,  # physics time steps (min: 2.0*0=0.0ms)
       max_delay=1,
    ),
  },
)
