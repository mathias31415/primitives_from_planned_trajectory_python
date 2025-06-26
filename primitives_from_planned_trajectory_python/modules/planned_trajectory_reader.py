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
        self.get_logger().info('Waiting for trajectory...')

    def trajectory_callback(self, msg):
        if self.trajectory_received:
            return

        traj = msg.trajectory[0].joint_trajectory
        self.joint_names = traj.joint_names
        self.trajectory_points = traj.points
        self.trajectory_received = True
        self.get_logger().info(f'Trajectory received with {len(self.trajectory_points)} points.')
