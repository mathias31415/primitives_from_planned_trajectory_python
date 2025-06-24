#!/usr/bin/env python3

# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Mathias Fuhrer

import rclpy
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from industrial_robot_motion_interfaces.msg import MotionPrimitive
from geometry_msgs.msg import Pose

# from .modules.planned_trajectory_reader import TrajectoryProcessor
# from .modules.fk_client import FKClient
# from .modules.csv_writer import write_to_csv
# from .modules.approx_primitives_with_rdp import approx_LIN_primitives_with_rdp, approx_PTP_primitives_with_rdp
# from .modules.execute_motion_primitives import ExecuteMotionClient
# from .modules.joint_state_logger import JointStateLogger
# from .modules.marker_publisher import publish_poses_to_rviz

# To run with play button in VSCode instead of ros2 run
from modules.planned_trajectory_reader import TrajectoryProcessor
from modules.fk_client import FKClient
from modules.csv_writer import write_to_csv
from modules.approx_primitives_with_rdp import approx_LIN_primitives_with_rdp, approx_PTP_primitives_with_rdp
from modules.execute_motion_primitives import ExecuteMotionClient
from modules.joint_state_logger import JointStateLogger
from modules.marker_publisher import publish_poses_to_rviz

from compare_planned_and_executed_traj import compare_and_plot_trajectories

SAVE_DIR = 'src/primitives_from_planned_trajectory/data/saved_trajectories'


def compute_fk_for_joint_positions(fk_client, joint_names, joint_positions):
    return [fk_client.compute_fk(joint_names, jp, from_frame='base', to_link='tool0') for jp in joint_positions]


def handle_ptp_approximation(joint_positions, joint_names, fk_poses, plot_path):
    motion_sequence_msg = approx_PTP_primitives_with_rdp(
        joint_positions=joint_positions,
        joint_names=joint_names,
        epsilon=0.01,
        blend_radius=0.1,
        velocity=0.5,
        acceleration=0.5,
        plot_filepath=plot_path
    )

    reduced_poses = []
    for primitive in motion_sequence_msg.motions:
        if primitive.type != MotionPrimitive.LINEAR_JOINT:
            continue
        joint_vec = primitive.joint_positions
        match_idx = next((i for i, jp in enumerate(joint_positions) if np.allclose(jp, joint_vec, atol=1e-5)), None)
        if match_idx is not None:
            reduced_poses.append(fk_poses[match_idx])
        else:
            print(f"[WARN] No FK match for joint values: {joint_vec}")
    return motion_sequence_msg, reduced_poses


def handle_lin_approximation(fk_poses, plot_path):
    motion_sequence_msg = approx_LIN_primitives_with_rdp(
        poses_list=fk_poses,
        epsilon=0.01,
        blend_radius=0.1,
        velocity=0.5,
        acceleration=0.5,
        plot_filepath=plot_path
    )

    reduced_poses = []
    for primitive in motion_sequence_msg.motions:
        for pose_stamped in primitive.poses:
            reduced_poses.append(pose_stamped.pose)
    return motion_sequence_msg, reduced_poses


def select_approximation_method(method_key, joint_positions, joint_names, fk_poses, plot_path):
    methods = {
        '1': lambda: handle_ptp_approximation(joint_positions, joint_names, fk_poses, plot_path),
        '2': lambda: handle_lin_approximation(fk_poses, plot_path),
    }
    return methods.get(method_key, methods['1'])()  # Fallback to PTP


def main():
    rclpy.init()
    node = TrajectoryProcessor()

    while rclpy.ok() and not node.trajectory_received:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.trajectory_received:
        print('No trajectory received. Exiting.')
        return

    user_mode = input("Which motion type should be used?\n[1] PTP (default)\n[2] LIN\n(Cancel with Ctrl+C)\n").strip()

    joint_positions = [list(point.positions) for point in node.trajectory_points]
    fk_client = FKClient(node)
    fk_poses = compute_fk_for_joint_positions(fk_client, node.joint_names, joint_positions)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filepath_planned_traj = f"{SAVE_DIR}/trajectory_{timestamp}_planned.csv"
    write_to_csv(node.joint_names, node.trajectory_points, fk_poses, csv_filepath_planned_traj)

    plot_filepath_simplified_with_rdp = f"{SAVE_DIR}/trajectory_{timestamp}_simplified_with_rdp.png"

    motion_sequence_msg, reduced_poses = select_approximation_method(
        method_key=user_mode,
        joint_positions=joint_positions,
        joint_names=node.joint_names,
        fk_poses=fk_poses,
        plot_path=plot_filepath_simplified_with_rdp
    )

    if reduced_poses:
        publish_poses_to_rviz(
            node=node,
            poses=reduced_poses,
            frame_id="base",
            marker_ns="motion_primitive_goal_poses",
            axis_length=0.1,
            axis_width=0.01
        )

    while True:
        user_input = input("Do you want to continue with motion primitive execution? (y/N): ").strip().lower()

        if user_input in ('y', 'yes'):
            plt.close('all')
            print("Starting execution of motion primitives...")

            motion_node = ExecuteMotionClient()
            csv_filepath_executed_traj = f"{SAVE_DIR}/trajectory_{timestamp}_executed.csv"
            joint_state_logger = JointStateLogger(motion_node, csv_filepath_executed_traj)

            motion_node.send_motion_sequence(motion_sequence_msg)

            try:
                rclpy.spin(motion_node)
            finally:
                joint_state_logger.stop()
                
                # compare planned and executed trajectories
                joint_pos_names = [name + '_pos' for name in node.joint_names]
                compare_and_plot_trajectories(filepath_planned=csv_filepath_planned_traj, filepath_executed=csv_filepath_executed_traj, joint_pos_names=joint_pos_names, n_points=100)

            break

        elif user_input in ('', 'n', 'no'):
            print("Exiting without primitive execution.")
            plt.close('all')
            break

        else:
            print("Invalid input.")


    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()