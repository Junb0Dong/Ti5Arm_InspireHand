# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Allegro Hand robots from Wonik Robotics.

The following configurations are available:

* :obj:`ALLEGRO_HAND_CFG`: Allegro Hand with implicit actuator model.

Reference:

* https://www.wonikrobotics.com/robot-hand

"""


import math

import isaaclab.sim as sim_utils
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

##
# Configuration
##

TI5_INSPIRE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/junbo/IsaacLab/mycode/assets/left_arm_hand.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            # retain_accelerations=False,
            # enable_gyroscopic_forces=False,
            # angular_damping=0.01,
            # max_linear_velocity=1000.0,
            # max_angular_velocity=64 / math.pi * 180.0,
            max_depenetration_velocity=5.0,
            # max_contact_impulse=1e32,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            # sleep_threshold=0.005,
            # stabilization_threshold=0.0005,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "A": 0,
            "B": 0.0,
            "C": 0.0,
            "D": 0.0,
            "E": 0.0,
            "F": 0.0,
            "L_thumb_proximal_pitch_joint":0.4,
            "L_thumb_proximal_yaw_joint":0.4,
            "L_index_proximal_joint":0.4,
            "L_middle_proximal_joint":0.4,
            "L_ring_proximal_joint":0.4,
            "L_pinky_proximal_joint":0.4,
        },
        # pos=(0.0, 0.0, 0.5),
        # rot=(0.257551, 0.283045, 0.683330, -0.621782),
    ),
    
    actuators={
        "arm_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["A", "B", "C"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=100.0,
            damping=40.0,
        ),
        "arm_forearm": ImplicitActuatorCfg(
            joint_names_expr=["D", "E", "F"],
            effort_limit=12.0,
            velocity_limit=2.61,
            stiffness=40.0,
            damping=20.0,
        ),
        "thumb_fingers": ImplicitActuatorCfg(
            joint_names_expr=["L_thumb_proximal_pitch_joint", "L_thumb_proximal_yaw_joint"],
            effort_limit=200.0,
            velocity_limit=0.2,
            stiffness=2e3,
            damping=2e2,
            # friction=0.01,
        ),
        "other_fingers": ImplicitActuatorCfg(
            joint_names_expr=["L_index_proximal_joint", "L_middle_proximal_joint", "L_pinky_proximal_joint", "L_ring_proximal_joint"],
            effort_limit=0,
            velocity_limit=0.0,
            stiffness=1e3,
            damping=1e2,
            # friction=0.1,
        ),
        "passvie_joints": ImplicitActuatorCfg(
            joint_names_expr=["L_index_intermediate_joint", "L_middle_intermediate_joint", "L_pinky_intermediate_joint", "L_ring_intermediate_joint", "L_thumb_distal_joint", "L_thumb_intermediate_joint"],
            effort_limit=0,
            velocity_limit=0.0,
            stiffness=1e3,
            damping=1e2,
            friction=1.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Allegro Hand robot."""
