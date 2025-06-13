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

# https://moveit.picknik.ai/main/doc/api/python_api/api.html

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class FKClient(Node):
    def __init__(self):
        super().__init__('fk_client')
        self.cli = self.create_client(GetPositionFK, '/compute_fk')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_fk service...')
        self.get_logger().info('/compute_fk service available')

    def send_request(self, joint_names, joint_positions):
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_link'
        request.fk_link_names = ['tool0']

        # Fülle robot_state mit den Gelenkpositionen
        request.robot_state.joint_state.name = joint_names
        request.robot_state.joint_state.position = joint_positions

        self.future = self.cli.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    fk_client = FKClient()

    # Beispiel-Jointnamen und Positionen anpassen!
    joint_names = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]

    joint_positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]

    fk_client.send_request(joint_names, joint_positions)

    # Warte auf Antwort (Timeout 3 Sekunden)
    rclpy.spin_until_future_complete(fk_client, fk_client.future, timeout_sec=3.0)

    if fk_client.future.done():
        response = fk_client.future.result()
        if response.error_code.val == 1:  # SUCCESS
            pose = response.pose_stamped[0].pose
            fk_client.get_logger().info(f'Tool0 pose:\nPosition: x={pose.position.x:.4f}, y={pose.position.y:.4f}, z={pose.position.z:.4f}\n'
                                        f'Orientation: x={pose.orientation.x:.4f}, y={pose.orientation.y:.4f}, z={pose.orientation.z:.4f}, w={pose.orientation.w:.4f}')
        else:
            fk_client.get_logger().error(f'FK service returned error code: {response.error_code.val}')
    else:
        fk_client.get_logger().error('FK service call timed out')

    fk_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



