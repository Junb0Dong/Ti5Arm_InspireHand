# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""获取 TI5 机械臂初始关节信息的演示脚本"""

"""首先启动 Isaac Sim 仿真器"""
import argparse
import torch
from omni.isaac.lab.app import AppLauncher

# 添加 AppLauncher 命令行参数
parser = argparse.ArgumentParser(description="获取 TI5 机械臂初始关节信息")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# 启动 Omniverse 应用
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation, ArticulationCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.actuators.actuator_cfg import ImplicitActuatorCfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR



"""以下是主要逻辑部分"""

def design_scene():
    """构建仿真场景"""
    # Ground-plane
    # cfg = sim_utils.GroundPlaneCfg()
    # cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # TI5 机械臂
    TI5_ARM_CFG = ArticulationCfg(
    # nedd to be modified
    prim_path="/World/ti5_Robot",
    spawn=sim_utils.UsdFileCfg(
            usd_path=f"source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/direct/ti5_hand/arm2_assets/original_hand.usd",   # revlative path to the bash/terminal
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=12, 
                solver_velocity_iteration_count=1,
                fix_root_link=True
            ),
        ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            # "A": 0,
            # "B": 0,
            # "C": 0,
            # "D": 0,
            # "E": 0,
            # "F": 0,
            # "A": 1.157,
            # "B": 0.000,
            # "C": -0.155,
            # "D": -2.239,
            # "E": 0.000,
            # "F": 1.003,
            "L_index_proximal_joint": 0.1,
            "L_middle_proximal_joint": 0.000,
            "L_pinky_proximal_joint": 0.000,
            "L_ring_proximal_joint": 0.000,
            "L_thumb_proximal_yaw_joint": 0.300,
            "L_thumb_proximal_pitch_joint": 0.000,
            # "index_proximal_joint": 0.010,
            # "middle_proximal_joint": 0.0000,
            # "pinky_proximal_joint": 0.000,
            # "ring_proximal_joint": 0.000,
            # "thumb_proximal_yaw_joint": 0.300,
            # "thumb_proximal_pitch_joint": 0.000,
        },
        pos=(1.0, 0.0, 0.0),
        rot=(0.0, 0.0, 0.0, 1.0),
    ),
    actuators={
        # "arm_shoulder": ImplicitActuatorCfg(
        #     joint_names_expr=["A","B","C"],
        #     effort_limit=150.0,
        #     velocity_limit=2.5,
        #     stiffness=80.0,
        #     damping=40.0,
        # ),
        # "arm_forearm": ImplicitActuatorCfg(
        #     joint_names_expr=["D","E","F"],
        #     effort_limit=150.0,
        #     velocity_limit=2.5,
        #     stiffness=80.0,
        #     damping=40.0,
        # ),
        "active_fingers": ImplicitActuatorCfg(
            joint_names_expr=["L_index_proximal_joint", "L_middle_proximal_joint", "L_pinky_proximal_joint", "L_ring_proximal_joint", "L_thumb_proximal_yaw_joint", "L_thumb_proximal_pitch_joint"],
            effort_limit=0.0,
            velocity_limit=0,
            stiffness=0,
            damping=0,
        ),
        # "active_fingers": ImplicitActuatorCfg(
        #     joint_names_expr=["index_proximal_joint", "middle_proximal_joint", "pinky_proximal_joint", "ring_proximal_joint", "thumb_proximal_yaw_joint", "thumb_proximal_pitch_joint"],
        #     effort_limit=200.0,
        #     velocity_limit=0.2,
        #     stiffness=2e3,
        #     damping=1e2,
        # ),
    },
    )

    # #franka test
    # TI5_ARM_CFG = ArticulationCfg(
    #     prim_path="/World/envs/Robot",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path=f"source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/direct/ti5_hand/arm2_assets/franka.usd",
    #         activate_contact_sensors=False,
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             disable_gravity=False,
    #             max_depenetration_velocity=5.0,
    #         ),
    #         articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    #             enabled_self_collisions=False, solver_position_iteration_count=12, solver_velocity_iteration_count=1
    #         ),
    #     ),
    #     init_state=ArticulationCfg.InitialStateCfg(
    #         joint_pos={
    #             "panda_joint1": 0,
    #             "panda_joint2": 0,
    #             "panda_joint3": 0,
    #             "panda_joint4": 0,
    #             "panda_joint5": 0,
    #             "panda_joint6": 0,
    #             "panda_joint7": 0,
    #             "panda_finger_joint.*": 0.035,
    #         },
    #         pos=(1.0, 0.0, 0.0),
    #         rot=(0.0, 0.0, 0.0, 1.0),
    #     ),
    #     actuators={
    #         "panda_shoulder": ImplicitActuatorCfg(
    #             joint_names_expr=["panda_joint[1-4]"],
    #             effort_limit=0,
    #             velocity_limit=0,
    #             stiffness=0.0,
    #             damping=0.0,
    #         ),
    #         "panda_forearm": ImplicitActuatorCfg(
    #             joint_names_expr=["panda_joint[5-7]"],
    #             effort_limit=0,
    #             velocity_limit=0,
    #             stiffness=0.0,
    #             damping=0.0,
    #         ),
    #         "panda_hand": ImplicitActuatorCfg(
    #             joint_names_expr=["panda_finger_joint.*"],
    #             effort_limit=0,
    #             velocity_limit=0,
    #             stiffness=0.0,
    #             damping=0.0,
    #         ),
    #     },
    # )
    


    # 创建机械臂实例
    ti5_arm = Articulation(cfg=TI5_ARM_CFG)
    return {"ti5_arm": ti5_arm}

def main():
    """主函数"""
    # 初始化仿真上下文
    # sim_cfg = {"device": args_cli.device}  # 使用命令行指定的计算设备
    # sim = SimulationContext(sim_cfg)
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # 构建场景
    scene_entities = design_scene()
    
    # 重置仿真（关键步骤！）
    sim.reset()

    # 获取并打印关节信息
    robot = scene_entities["ti5_arm"]

    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    while simulation_app.is_running():
        # reset
        if count % 300 == 0:
            # reset counters
            sim_time = 0.0
            count = 0
            # root state
            root_state = robot.data.default_root_state.clone()
            robot.write_root_state_to_sim(root_state)
            # set joint positions
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            # joint_pos = torch.clamp(joint_pos, joint_limits[:, 0], joint_limits[:, 1])

            print("====================================")
            print("joint_pos:", joint_pos)
            print("====================================")
            print("joint_vel:", joint_vel)
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            robot.reset()
            print("[INFO]: Resetting robots state...")

        # apply random actions to the robots and generate random joint positions
        joint_pos_target = robot.data.default_joint_pos + torch.randn_like(robot.data.joint_pos) * 0.1
        joint_pos_target = joint_pos_target.clamp_(
            robot.data.soft_joint_pos_limits[..., 0], robot.data.soft_joint_pos_limits[..., 1]
        )

        # apply action to the robot
        robot.set_joint_position_target(joint_pos_target)

        # write data to sim
        robot.write_data_to_sim()

        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers

        robot.update(sim_dt)


if __name__ == "__main__":
    main()
    simulation_app.close()