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

# from .modules.planned_trajectory_reader import TrajectoryProcessor
# from .modules.fk_client import FKClient
# from .modules.csv_writer import write_to_csv
# from .modules.approx_primitives_with_rdp import approx_PTP_primitives_with_rdp
# from .modules.execute_motion_primitives import ExecuteMotionClient
# from .modules.joint_state_logger import JointStateLogger
# from .modules.marker_publisher import publish_poses_to_rviz

# To run with play button in VSCode instead of ros2 run
from modules.planned_trajectory_reader import TrajectoryProcessor
from modules.fk_client import FKClient
from modules.csv_writer import write_to_csv
from modules.approx_primitives_with_rdp import approx_PTP_primitives_with_rdp
from modules.execute_motion_primitives import ExecuteMotionClient
from modules.joint_state_logger import JointStateLogger
from modules.marker_publisher import publish_poses_to_rviz

from industrial_robot_motion_interfaces.msg import MotionPrimitive
from geometry_msgs.msg import Pose

SAVE_DIR = 'src/primitives_from_planned_trajectory/data/saved_trajectories'

def main():
    rclpy.init()
    node = TrajectoryProcessor()

    # Wait for planned trajectory
    while rclpy.ok() and not node.trajectory_received:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.trajectory_received:
        print('No trajectory received. Exiting.')
        return

    joint_positions = [list(point.positions) for point in node.trajectory_points]

    # Calculate FK for all joint positions
    fk_client = FKClient(node)
    fk_poses = []
    for jp in joint_positions:
        pose = fk_client.compute_fk(node.joint_names, jp, from_frame='base', to_link='tool0')
        fk_poses.append(pose)

    # Save CSV with joint trajectory and FK poses
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filepath_planned_traj = f"{SAVE_DIR}/trajectory_{timestamp}_planned.csv"
    write_to_csv(node.joint_names, node.trajectory_points, fk_poses, csv_filepath_planned_traj)

    # Approximate primitives in joint space
    plot_filepath_simplified_with_rdp = f"{SAVE_DIR}/trajectory_{timestamp}_simplified_with_rdp.png"
    motion_sequence_msg = approx_PTP_primitives_with_rdp(
        joint_positions=joint_positions,
        joint_names=node.joint_names,
        epsilon=0.01,
        blend_radius=0.1,
        velocity=0.5,
        acceleration=0.5,
        plot_filepath=plot_filepath_simplified_with_rdp
    )

    reduced_fk_poses = []

    for primitive in motion_sequence_msg.motions:
        if primitive.type != MotionPrimitive.LINEAR_JOINT:
            continue  # nur PTP relevant

        joint_vec = primitive.joint_positions
        match_idx = next(
            (i for i, jp in enumerate(joint_positions) if np.allclose(jp, joint_vec, atol=1e-5)),
            None
        )

        if match_idx is not None:
            pose = Pose()
            pose = fk_poses[match_idx]
            reduced_fk_poses.append(pose)
        else:
            print(f"[WARN] Kein FK-Match für Joint-Werte: {joint_vec}")

    # Publish FK-Poses für reduzierte PTP-Punkte nach RViz
    if reduced_fk_poses:
        publish_poses_to_rviz(
            node=node,
            poses=reduced_fk_poses,
            frame_id="base",
            marker_ns="motion_primitive_goal_poses",
            axis_length=0.1,
            axis_width=0.01
        )

    # Nutzerabfrage vor Ausführung
    user_input = input("Do you want to continue with primitive execution? (Type yes): ").strip().lower()
    if user_input == 'yes':
        plt.close('all')
        print("Continuing with primitive execution...")

        motion_node = ExecuteMotionClient()

        csv_filepath_executed_traj = f"{SAVE_DIR}/trajectory_{timestamp}_executed.csv"
        joint_state_logger = JointStateLogger(motion_node, csv_filepath_executed_traj)

        motion_node.send_motion_sequence(motion_sequence_msg)

        try:
            rclpy.spin(motion_node)
        finally:
            joint_state_logger.stop()
    else:
        print("Exiting without primitive execution.")
        plt.close('all')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
