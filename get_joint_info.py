# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""获取 TI5 机械臂初始关节信息的演示脚本"""

"""首先启动 Isaac Sim 仿真器"""
import argparse
import torch
from isaaclab.app import AppLauncher

# 添加 AppLauncher 命令行参数
parser = argparse.ArgumentParser(description="获取 TI5 机械臂初始关节信息")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# 启动 Omniverse 应用
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils 
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab_assets.robots.ti5_inspire import TI5_INSPIRE_CFG


"""以下是主要逻辑部分"""

def design_scene():
    """构建仿真场景"""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)
    # 创建机械臂实例
    ti5_arm = Articulation(TI5_INSPIRE_CFG.replace(prim_path="/World/Robot"))
    
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
    for name, pos_tensor in zip(robot.data.joint_names, robot.data.joint_pos):
        # 将张量转换为标量数值
        pos = pos_tensor.item() if pos_tensor.numel() == 1 else pos_tensor.mean().item()
        print(f"关节名称: {name:<20} 初始位置: {pos:.4f}")
    
    # 保持仿真运行
    while simulation_app.is_running():
        sim.step()


if __name__ == "__main__":
    main()
    simulation_app.close()