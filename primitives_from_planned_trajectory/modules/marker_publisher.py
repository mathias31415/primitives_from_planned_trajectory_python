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
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from tf_transformations import quaternion_matrix


def publish_poses_to_rviz(
    node: Node,
    poses: list[Pose],
    frame_id: str = "world",
    marker_ns: str = "pose_axes",
    axis_length: float = 0.1,
    axis_width: float = 0.002,
):
    """
    Publishes coordinate axes (X, Y, Z) for a list of poses using LINE_LIST markers.

    Args:
        node (Node): ROS2 node instance.
        poses (List[Pose]): List of Pose messages.
        frame_id (str): The reference frame for RViz.
        marker_ns (str): Marker namespace.
        axis_length (float): Length of each axis line.
        axis_width (float): Thickness of axis lines.
    """
    marker_array = MarkerArray()
    marker_pub = node.create_publisher(MarkerArray, "/visualization_marker_array", 10)
    marker_id = 0

    for i, pose in enumerate(poses):
        # Extract position and orientation
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        orientation = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

        # Convert quaternion to rotation matrix
        rot_matrix = quaternion_matrix(orientation)[:3, :3]

        # Create a marker for this pose
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = node.get_clock().now().to_msg()
        marker.ns = marker_ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = axis_width

        # Define colored axis lines
        colors = {
            "x": (1.0, 0.0, 0.0, 1.0),  # Red
            "y": (0.0, 1.0, 0.0, 1.0),  # Green
            "z": (0.0, 0.0, 1.0, 1.0),  # Blue
        }

        for j, (axis, color) in enumerate(colors.items()):
            direction = rot_matrix[:, j]
            start = position
            end = position + axis_length * direction

            marker.points.append(Point(x=start[0], y=start[1], z=start[2]))
            marker.points.append(Point(x=end[0], y=end[1], z=end[2]))

            c = Marker().color
            c.r, c.g, c.b, c.a = color
            marker.colors.append(c)
            marker.colors.append(c)

        marker_array.markers.append(marker)
        marker_id += 1

    marker_pub.publish(marker_array)
    node.get_logger().info(f"Published {len(marker_array.markers)} coordinate frames to /visualization_marker_array.")