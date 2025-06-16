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
from datetime import datetime
from modules.planned_trajectory_reader import TrajectoryProcessor
from modules.fk_client import FKClient
from modules.csv_writer import write_to_csv
from modules.approx_LIN_primitives_with_rdp import approx_LIN_primitives_with_rdp

#plt.close('all')
def main():
    rclpy.init()
    node = TrajectoryProcessor()

    # Get planned trajectory
    while rclpy.ok() and not node.trajectory_received:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.trajectory_received:
        print('No trajectory received. Exiting.')
        return

    # calculate forward kinematics for each trajectory point
    fk_client = FKClient(node)
    fk_poses = []
    for point in node.trajectory_points:
        pose = fk_client.compute_fk(node.joint_names, list(point.positions))
        fk_poses.append(pose)

    # write trajectory and fk poses to CSV
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename_planned_traj = f"trajectory_{timestamp}_planned.csv"
    write_to_csv(node.joint_names, node.trajectory_points, fk_poses, csv_filename_planned_traj)

    # calculate primitives and plot them
    motion_sequence_msg = approx_LIN_primitives_with_rdp(fk_poses, epsilon=0.01)

    # ask user if they want to continue with primitive extraction
    user_input = input("Do you want to continue with primitive extraction? (Type yes): ").strip().lower()
    if user_input == 'yes':
        # execute primitives with motion primitives forward controller
        plt.close('all')
        print("Continuing with primitive extraction...")
        csv_filename_executed_traj = f"trajectory_{timestamp}_executed.csv"
        # print motion sequence message
        print(motion_sequence_msg)
        # TODO: Implement primitive extraction logic here
    else:
        print("Exiting without primitive extraction.")
        plt.close('all')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
