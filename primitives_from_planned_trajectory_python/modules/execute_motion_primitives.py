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
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from industrial_robot_motion_interfaces.msg import MotionPrimitive, MotionArgument, MotionSequence
from industrial_robot_motion_interfaces.action import ExecuteMotion
from action_msgs.srv import CancelGoal
import threading
import sys


class ExecuteMotionClient(Node):
    def __init__(self):
        super().__init__("motion_sequence_client")

        self._client = ActionClient(
            self, ExecuteMotion, "/motion_primitive_forward_controller/motion_sequence"
        )
        self._cancel_client = self.create_client(
            CancelGoal, "/motion_primitive_forward_controller/motion_sequence/_action/cancel_goal"
        )

        self._goal_id = None  # goal_id for cancel request
        self._cancel_thread = None

    def send_motion_sequence(self, motion_sequence: MotionSequence, enable_cancel: bool = True):
        """Send a custom motion sequence to the action server.

        Args:
            motion_sequence (MotionSequence): The motion sequence to execute.
            enable_cancel (bool): If True, enables ENTER key canceling.
        """
        self.get_logger().info("Waiting for action server...")
        self._client.wait_for_server()

        goal_msg = ExecuteMotion.Goal()
        goal_msg.trajectory = motion_sequence

        self.get_logger().info(
            f"Sending {len(goal_msg.trajectory.motions)} motion primitives as a sequence..."
        )
        send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        if enable_cancel:
            self._cancel_thread = threading.Thread(target=self._wait_for_keypress, daemon=True)
            self._cancel_thread.start()

    def goal_response_callback(self, future):
        """Callback called when the action server accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._goal_id = goal_handle.goal_id  # Store goal ID for cancellation

        # Wait for result asynchronously
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive feedback about the current motion primitive being executed."""
        current_index = feedback_msg.feedback.current_primitive_index
        self.get_logger().info(f"Executing primitive index: {current_index}")

    def result_callback(self, future):
        """Handle the result from the action server after goal finishes or is canceled."""
        result = future.result().result
        if result.error_code == ExecuteMotion.Result.SUCCESSFUL:
            self.get_logger().info("Motion sequence executed successfully!")
        elif result.error_code == ExecuteMotion.Result.CANCELED:
            self.get_logger().info("Motion sequence was canceled.")
        elif result.error_code == ExecuteMotion.Result.FAILED:
            self.get_logger().error("Motion sequence execution failed.")
        else:
            self.get_logger().error(
                f"Execution failed: {result.error_code} - {result.error_string}"
            )
        rclpy.shutdown()

    def _wait_for_keypress(self):
        """Wait for the user to press ENTER key to cancel the motion sequence."""
        print("Press ENTER to cancel the motion sequence...")
        while True:
            input_str = sys.stdin.readline().strip()
            if input_str == "":
                self.get_logger().info("ENTER key pressed: sending cancel request.")
                self.cancel_goal()
                break

    def cancel_goal(self):
        """Send a cancel request for the currently running goal."""
        if self._goal_id is None:
            self.get_logger().warn("No goal to cancel (goal_id not set yet).")
            return

        if not self._cancel_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Cancel service is not available.")
            return

        request = CancelGoal.Request()
        request.goal_info.goal_id = self._goal_id

        future = self._cancel_client.call_async(request)
        future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        """Handle the response from the cancel service call."""
        try:
            response = future.result()
            if response.return_code == 0:
                self.get_logger().info("Cancel request accepted.")
            elif response.return_code == 1:
                self.get_logger().warn("Cancel request rejected.")
            else:
                self.get_logger().warn(f"Cancel returned code: {response.return_code}")
        except Exception as e:
            self.get_logger().error(f"Failed to call cancel service: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ExecuteMotionClient()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
