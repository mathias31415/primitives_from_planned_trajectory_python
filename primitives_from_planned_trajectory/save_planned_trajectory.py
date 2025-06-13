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
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from moveit_py.robot_model import RobotModel
from moveit_py.robot_state import RobotState
import csv
from datetime import datetime
import os
import numpy as np

SAVE_DIR = 'src/primitives_from_planned_trajectory/data/saved_trajectories'

class TrajectoryToCSV(Node):
    def __init__(self):
        super().__init__('trajectory_to_csv')

        self.robot_model = RobotModel()
        self.robot_model.load_robot_description()

        # Subscribe to MoveIt planned path topic
        self.subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.callback,
            10
        )
        self.saved = False
        self.get_logger().info('Waiting for a planned trajectory from /display_planned_path... with pose')

    def callback(self, msg):
        if self.saved:
            return

        self.get_logger().info('Planned trajectory received. Processing...')

        trajectory = msg.trajectory[0].joint_trajectory
        robot_state = RobotState(self.robot_model)

        os.makedirs(SAVE_DIR, exist_ok=True)

        # Create timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(SAVE_DIR, f"trajectory_{timestamp}.csv")

        try:
            # Build header with joint names for positions, velocities, accelerations
            fieldnames = ['time_from_start']
            for joint_name in trajectory.joint_names:
                fieldnames.append(f'{joint_name}_pos')
            for joint_name in trajectory.joint_names:
                fieldnames.append(f'{joint_name}_vel')
            for joint_name in trajectory.joint_names:
                fieldnames.append(f'{joint_name}_acc')
            # Add Cartesian pose fields
            fieldnames.extend(['ee_pos_x', 'ee_pos_y', 'ee_pos_z',
                            'ee_ori_x', 'ee_ori_y', 'ee_ori_z', 'ee_ori_w'])


            # Write CSV
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

                for point in trajectory.points:
                    row = {
                        'time_from_start': point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                    }

                    # Add joint positions
                    for name, pos in zip(trajectory.joint_names, point.positions):
                        row[f'{name}_pos'] = pos

                    # Add joint velocities if available
                    if point.velocities:
                        for name, vel in zip(trajectory.joint_names, point.velocities):
                            row[f'{name}_vel'] = vel

                    # Add joint accelerations if available
                    if point.accelerations:
                        for name, acc in zip(trajectory.joint_names, point.accelerations):
                            row[f'{name}_acc'] = acc

                    # Set joint positions in RobotState
                    robot_state.set_variable_positions({
                        name: pos for name, pos in zip(trajectory.joint_names, point.positions)
                    })
                    robot_state.update()  # wichtig, damit Kinematik aktualisiert wird

                    # Forward Kinematics für Endeffektor-Link (z. B. "tool0")
                    ee_link = "tool0"
                    pose = robot_state.get_global_link_transform(ee_link)

                    # Position und Orientierung extrahieren
                    translation = pose.translation
                    rotation = pose.rotation  # Quaternion (x, y, z, w)

                    row['ee_pos_x'] = translation.x
                    row['ee_pos_y'] = translation.y
                    row['ee_pos_z'] = translation.z
                    row['ee_ori_x'] = rotation.x
                    row['ee_ori_y'] = rotation.y
                    row['ee_ori_z'] = rotation.z
                    row['ee_ori_w'] = rotation.w

                    writer.writerow(row)

            self.get_logger().info(f'Trajectory saved to: {filename}')
            self.saved = True
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error while saving trajectory: {e}')

def main():
    rclpy.init()
    node = TrajectoryToCSV()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
