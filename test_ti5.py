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

    # TI5 机械臂
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