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
from moveit_msgs.srv import GetPositionFK
from datetime import datetime
import os
import csv

SAVE_DIR = 'src/primitives_from_planned_trajectory/data/saved_trajectories'


class FKClient:
    def __init__(self, node: Node):
        self.node = node
        self.client = node.create_client(GetPositionFK, '/compute_fk')
        while not self.client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Waiting for /compute_fk service...')

    def compute_fk(self, joint_names, joint_positions):
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_link'
        request.fk_link_names = ['tool0']
        request.robot_state.joint_state.name = joint_names
        request.robot_state.joint_state.position = joint_positions

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)

        if future.done():
            result = future.result()
            if result and result.error_code.val == 1:
                return result.pose_stamped[0].pose
            else:
                self.node.get_logger().warn(f'FK error: code={result.error_code.val}')
        else:
            self.node.get_logger().error('FK call timed out')

        return None


class TrajectoryProcessor(Node):
    def __init__(self):
        super().__init__('trajectory_processor')

        self.joint_names = []
        self.trajectory_points = []
        self.trajectory_received = False

        self.subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.trajectory_callback,
            10
        )

        self.get_logger().info('Node ready. Waiting for trajectory...')

    def trajectory_callback(self, msg):
        if self.trajectory_received:
            return

        traj = msg.trajectory[0].joint_trajectory
        self.joint_names = traj.joint_names
        self.trajectory_points = traj.points
        self.trajectory_received = True

        self.get_logger().info(f'Trajectory received with {len(self.trajectory_points)} points.')


def write_to_csv(joint_names, points, fk_poses, filename):
    fieldnames = ['time_from_start']
    for name in joint_names:
        fieldnames.append(f'{name}_pos')
    for name in joint_names:
        fieldnames.append(f'{name}_vel')
    for name in joint_names:
        fieldnames.append(f'{name}_acc')
    fieldnames += ['fk_x', 'fk_y', 'fk_z', 'fk_qx', 'fk_qy', 'fk_qz', 'fk_qw']

    os.makedirs(SAVE_DIR, exist_ok=True)
    filepath = os.path.join(SAVE_DIR, filename)

    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for point, fk_pose in zip(points, fk_poses):
            row = {
                'time_from_start': point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            }
            for name, pos in zip(joint_names, point.positions):
                row[f'{name}_pos'] = pos
            for name, vel in zip(joint_names, point.velocities or []):
                row[f'{name}_vel'] = vel
            for name, acc in zip(joint_names, point.accelerations or []):
                row[f'{name}_acc'] = acc

            if fk_pose:
                row.update({
                    'fk_x': fk_pose.position.x,
                    'fk_y': fk_pose.position.y,
                    'fk_z': fk_pose.position.z,
                    'fk_qx': fk_pose.orientation.x,
                    'fk_qy': fk_pose.orientation.y,
                    'fk_qz': fk_pose.orientation.z,
                    'fk_qw': fk_pose.orientation.w
                })
            else:
                for f in ['fk_x', 'fk_y', 'fk_z', 'fk_qx', 'fk_qy', 'fk_qz', 'fk_qw']:
                    row[f] = float('nan')

            writer.writerow(row)

    print(f'Trajectory with FK saved to: {filepath}')


def main():
    rclpy.init()
    node = TrajectoryProcessor()

    while rclpy.ok() and not node.trajectory_received:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.trajectory_received:
        print('No trajectory received. Exiting.')
        return

    fk_client = FKClient(node)

    fk_poses = []
    for i, point in enumerate(node.trajectory_points):
        pose = fk_client.compute_fk(node.joint_names, list(point.positions))
        fk_poses.append(pose)

    print(f'Calculated FK for {len(fk_poses)} points.')

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_name = f"planned_trajectory_{timestamp}.csv"
    write_to_csv(node.joint_names, node.trajectory_points, fk_poses, csv_name)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
