# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates different legged robots.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/demos/quadrupeds.py

"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different legged robots.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import numpy as np
import torch

import isaacsim.core.utils.prims as prim_utils

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation

##
# Pre-defined configs
##
from isaaclab_assets.robots.robot_spot_ean import QUAD_EAN
from isaaclab_assets.robots.anymal import ANYMAL_C_CFG, ANYMAL_B_CFG
from isaaclab_assets.robots.spot import SPOT_CFG

def define_origins(num_origins: int, spacing: float) -> list[list[float]]:
    """Defines the origins of the scene."""
    # create tensor based on number of environments
    env_origins = torch.zeros(num_origins, 3)
    # create a grid of origins
    num_cols = np.floor(np.sqrt(num_origins))
    num_rows = np.ceil(num_origins / num_cols)
    xx, yy = torch.meshgrid(torch.arange(num_rows), torch.arange(num_cols), indexing="xy")
    env_origins[:, 0] = spacing * xx.flatten()[:num_origins] - spacing * (num_rows - 1) / 2
    env_origins[:, 1] = spacing * yy.flatten()[:num_origins] - spacing * (num_cols - 1) / 2
    env_origins[:, 2] = 0.0
    # return the origins
    return env_origins.tolist()


def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Create separate groups called "Origin1", "Origin2", "Origin3"
    # Each group will have a mount and a robot on top of it
    origins = define_origins(num_origins=7, spacing=1.25)

    # Origin 1 with Anymal B
    prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])
    # -- Robot
    spot1 = Articulation(SPOT_CFG.replace(prim_path="/World/Origin1/Robot"))
    spot2 = Articulation(QUAD_EAN.replace(prim_path="/World/Origin1/Robot2"))
    # return the scene information
    scene_entities = {
        "spot1": spot1,
        "spot2": spot2
    }
    return scene_entities, origins


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 1
    # Simulate physics

    for index, robot in enumerate(entities.values()):
        joint_names = robot.joint_names  # Danh sách tên các joint (degrees of freedom - DOF)
        print("All joint names:", joint_names)
        print(robot._data.default_mass)
        print(robot._data.joint_pos_limits)
        # root state
        root_state = robot.data.default_root_state.clone()
        root_state[:, :3] += origins[index]  # Giữ nguyên vị trí
        # Reset root velocity về 0 (vận tốc tuyến tính và góc)
        root_state[:, 7:] = 0.0  # Ghi đè vận tốc thành 0
        robot.write_root_pose_to_sim(root_state[:, :7])  # Ghi vị trí và hướng
        robot.write_root_velocity_to_sim(root_state[:, 7:])  # Ghi vận tốc (bây giờ là 0)

        # joint state
        joint_pos = robot.data.default_joint_pos.clone()  # Giữ nguyên vị trí khớp mặc định
        joint_vel = torch.zeros_like(robot.data.default_joint_vel)  # Reset vận tốc khớp về 0
        robot.write_joint_state_to_sim(joint_pos, joint_vel)  # Ghi trạng thái khớp

        # reset the internal state
        robot.reset()

    while simulation_app.is_running():
        sim_time = 0.0
        # apply default actions to the quadrupedal robots
        #for robot in entities.values():
            # generate random joint positions
            #joint_pos_target = torch.randn_like(robot.data.joint_pos) * 0.1
            # apply action to the robot
            #robot.set_joint_position_target(joint_pos_target)
            # write data to sim
            #robot.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        #count += 1
        # update buffers
        for robot in entities.values():
            robot.update(sim_dt)


def main():
    """Main function."""

    # Initialize the simulation context
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01, device="cpu"))
    # Set main camera
    sim.set_camera_view(eye=[2.5, 2.5, 2.5], target=[0.0, 0.0, 0.0])
    # design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
