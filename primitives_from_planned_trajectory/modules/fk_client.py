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

from moveit_msgs.srv import GetPositionFK
import rclpy

class FKClient:
    def __init__(self, node):
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
