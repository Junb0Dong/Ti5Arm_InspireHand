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



"""以下是主要逻辑部分"""

def design_scene():
    """构建仿真场景"""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # TI5_ARM_CFG = ArticulationCfg(
    # prim_path="/World/Robot",
    # spawn=sim_utils.UsdFileCfg(
    #         usd_path=f"source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/direct/ti5_hand/arm2_assets/arm2.usd", # revlative path to the bash/terminal
    #         activate_contact_sensors=False,
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             disable_gravity=False,
    #             max_depenetration_velocity=5.0,
    #         ),
    #         articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    #             enabled_self_collisions=False, solver_position_iteration_count=12, solver_velocity_iteration_count=1
    #         ),
    #     ),
    # actuators={
    #     "default": ImplicitActuatorCfg(
    #         joint_names_expr=[".*"],
    #         effort_limit=150.0,    # 根据实际电机参数调整
    #         velocity_limit=12.0,
    #         stiffness=300.0,       # 适当提高刚度
    #         damping=25.0           # 适当提高阻尼
    #     )
    # }
    # )

    TI5_ARM_CFG = ArticulationCfg(
    # nedd to be modified
    prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(
            usd_path=f"source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/direct/ti5_hand/arm2_assets/arm2.usd",   # revlative path to the bash/terminal
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=12, solver_velocity_iteration_count=1
            ),
        ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "A": 1.157,
            "B": -1.066,
            "C": -0.155,
            "D": -2.239,
            "E": -1.841,
            "F": 1.003,
            "L_index_proximal_joint": 0.0000,
            "L_middle_proximal_joint": 0.0000,
            "L_pinky_proximal_joint": 0.0001,
            "L_ring_proximal_joint": 0.0001,
            "L_thumb_proximal_yaw_joint": 0.0002,
            "L_index_intermediate_joint": 0.0000,
            "L_middle_intermediate_joint": -0.0000,
            "L_pinky_intermediate_joint": -0.0000,
            "L_ring_intermediate_joint": -0.0000,
            "L_thumb_proximal_pitch_joint": 0.0001,
            "L_thumb_intermediate_joint": 0.0023,
            "L_thumb_distal_joint": 0.0000
        },
        pos=(1.0, 0.0, 0.0),
        rot=(0.0, 0.0, 0.0, 1.0),
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["A", "B", "C", "D", "E", "F"],
            effort_limit=150.0,
            velocity_limit=2.5,
            stiffness=80.0,
            damping=4.0,
        ),
        "active_fingers": ImplicitActuatorCfg(
            joint_names_expr=["L_index_proximal_joint", "L_middle_proximal_joint", "L_pinky_proximal_joint", "L_ring_proximal_joint", "L_thumb_proximal_yaw_joint", "L_thumb_proximal_pitch_joint"],
            effort_limit=200.0,
            velocity_limit=0.2,
            stiffness=2e3,
            damping=1e2,
        ),
        "passive_fingers": ImplicitActuatorCfg(
            joint_names_expr=["L_index_intermediate_joint", "L_middle_intermediate_joint", "L_pinky_intermediate_joint", "L_ring_intermediate_joint", "L_thumb_intermediate_joint", "L_thumb_distal_joint"],
            effort_limit=0.0,   # 禁止主动控制
            stiffness=0.0,      # 取消刚度
            damping=0.0         # 取消阻尼
        ),
    },
    )


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
    print("TI5机械臂初始关节信息：")
    print(robot.data.joint_names)
    print("==============================")
    print(robot.data.joint_pos)
    # for name, pos_tensor in zip(robot.data.joint_names, robot.data.joint_pos):
    #     # 将张量转换为标量数值
    #     pos = pos_tensor.item() if pos_tensor.numel() == 1 else pos_tensor.mean().item()
    #     print(f"关节名称: {name:<20} 初始位置: {pos:.4f}")
    
    # 保持仿真运行
    # while simulation_app.is_running():
    #     sim.step()

    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    while simulation_app.is_running():
        sim.step()

if __name__ == "__main__":
    main()
    simulation_app.close()