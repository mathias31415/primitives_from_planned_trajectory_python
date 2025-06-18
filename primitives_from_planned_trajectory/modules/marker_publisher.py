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
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose


def publish_poses_to_rviz(
    node: Node,
    poses: list[Pose],
    frame_id: str = "world",
    marker_ns: str = "poses",
    marker_type: int = Marker.SPHERE,
    scale: tuple = (0.01, 0.01, 0.01),
    color: tuple = (1.0, 0.0, 0.0, 1.0),  # RGBA: red by default
):
    """
    Publish a list of poses as visualization markers in RViz.

    Args:
        node (rclpy.node.Node): The running ROS2 node instance.
        poses (List[Pose]): List of geometry_msgs.msg.Pose.
        frame_id (str): Reference frame (e.g., "world").
        marker_ns (str): Namespace of the markers.
        marker_type (int): Marker shape type (e.g., Marker.SPHERE, Marker.ARROW).
        scale (Tuple[float, float, float]): Size of the marker.
        color (Tuple[float, float, float, float]): RGBA color.
    """

    marker_array = MarkerArray()
    marker_pub = node.create_publisher(MarkerArray, "/visualization_marker_array", 10)

    for i, pose in enumerate(poses):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = node.get_clock().now().to_msg()
        marker.ns = marker_ns
        marker.id = i
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime.sec = 0  # forever
        marker_array.markers.append(marker)

    marker_pub.publish(marker_array)
    node.get_logger().info(f"Published {len(poses)} poses to RViz.")
