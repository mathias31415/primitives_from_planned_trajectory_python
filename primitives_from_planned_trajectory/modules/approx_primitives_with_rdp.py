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

import numpy as np
from rdp import rdp, pldist_nd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped, Pose
from industrial_robot_motion_interfaces.msg import MotionPrimitive, MotionSequence, MotionArgument

def approx_LIN_primitives_with_rdp(poses_list, epsilon=0.01, blend_radius=0.0, velocity=0.01, acceleration=0.01, plot_filepath=None):
    """
    Approximates motion primitives from Cartesian poses using Ramer-Douglas-Peucker Algorithm (RDP).
    Also plots the original vs. reduced path in 3D.

    Args:
        poses_list (List[geometry_msgs/Pose]): Cartesian poses (usually FK results).
        epsilon (float): RDP simplification epsilon.

    Returns:
        MotionSequence: ROS2 message containing motion primitives.
    """

    # RDP Reduction
    points = np.array([[p.position.x, p.position.y, p.position.z] for p in poses_list])
    reduced_points = rdp(points, epsilon=epsilon)

    # MotionSequence Message
    motion_sequence = MotionSequence()
    motion_primitives = []

    for pt in reduced_points[1:]:  # Skip the first point (start position)
        primitive = MotionPrimitive()
        primitive.type = MotionPrimitive.LINEAR_CARTESIAN
        primitive.blend_radius = blend_radius
        primitive.additional_arguments = [
            MotionArgument(argument_name="velocity", argument_value=velocity),
            MotionArgument(argument_name="acceleration", argument_value=acceleration),
        ]

        pose = PoseStamped()
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.position.z = pt[2]

        # Use the original pose orientation for the reduced point
        matches = np.where((points == pt).all(axis=1))[0]
        if len(matches) == 0:
            raise ValueError(f"Reduced point {pt} not found in original points!")
        idx = matches[0]

        pose.pose.orientation.x = poses_list[idx].orientation.x
        pose.pose.orientation.y = poses_list[idx].orientation.y
        pose.pose.orientation.z = poses_list[idx].orientation.z
        pose.pose.orientation.w = poses_list[idx].orientation.w

        primitive.poses.append(pose)
        motion_primitives.append(primitive)

        print(f"Added LINEAR_CARTESIAN (LIN): [x: {pose.pose.position.x}, y: {pose.pose.position.y}, z: {pose.pose.position.z}, "
              f"qx: {pose.pose.orientation.x}, qy: {pose.pose.orientation.y}, "
              f"qz: {pose.pose.orientation.z}, qw: {pose.pose.orientation.w}, "
              f"blend_radius: {primitive.blend_radius}, velocity: {velocity}, acceleration: {acceleration}]\n")

    motion_sequence.motions = motion_primitives

    # 3D Plot for x,y,z coordinates
    fig3d = plt.figure(figsize=(10, 8))
    ax3d = fig3d.add_subplot(111, projection='3d')

    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    xr, yr, zr = reduced_points[:, 0], reduced_points[:, 1], reduced_points[:, 2]

    ax3d.plot(x, y, z, marker='o', label='Original Path', color='blue', alpha=0.5)
    ax3d.plot(xr, yr, zr, marker='o', label='RDP Path', color='orange')
    ax3d.scatter(x[0], y[0], z[0], color='red', s=50, label='Start')
    ax3d.scatter(x[-1], y[-1], z[-1], color='green', s=50, label='End')

    # show orientation arrows at reduced points
    arrow_len = 0.05
    for i, pt in enumerate(reduced_points):
        matches = np.where((points == pt).all(axis=1))[0]
        if len(matches) == 0:
            continue
        idx = matches[0]
        pose = poses_list[idx]

        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rot = R.from_quat(quat)

        x_axis = rot.apply([1, 0, 0])
        y_axis = rot.apply([0, 1, 0])
        z_axis = rot.apply([0, 0, 1])

        ax3d.quiver(pt[0], pt[1], pt[2], x_axis[0], x_axis[1], x_axis[2],
                    length=arrow_len, color='r', normalize=True, label='_nolegend_')
        ax3d.quiver(pt[0], pt[1], pt[2], y_axis[0], y_axis[1], y_axis[2],
                    length=arrow_len, color='g', normalize=True, label='_nolegend_')
        ax3d.quiver(pt[0], pt[1], pt[2], z_axis[0], z_axis[1], z_axis[2],
                    length=arrow_len, color='b', normalize=True, label='_nolegend_')

    ax3d.set_xlabel('X')
    ax3d.set_ylabel('Y')
    ax3d.set_zlabel('Z')
    ax3d.set_title('Cartesian trajectory before and after RDP simplification for LIN primitives')
    ax3d.legend()

    # Equal scaling
    max_range = max(max(x)-min(x), max(y)-min(y), max(z)-min(z)) / 2.0
    mid_x, mid_y, mid_z = (max(x)+min(x))/2.0, (max(y)+min(y))/2.0, (max(z)+min(z))/2.0
    ax3d.set_xlim(mid_x - max_range, mid_x + max_range)
    ax3d.set_ylim(mid_y - max_range, mid_y + max_range)
    ax3d.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.tight_layout()

    if plot_filepath:
        plt.savefig(plot_filepath, dpi=300)
        print(f"Plot saved to {plot_filepath}")
        
    plt.show(block=False)

    print(f"Reduced {len(points)} trajectory points to {len(reduced_points)-1} LIN primitives using RDP with epsilon={epsilon}")
    return motion_sequence


def approx_PTP_primitives_with_rdp(joint_positions, joint_names, epsilon=0.01, blend_radius=0.0, velocity=0.01, acceleration=0.01, plot_filepath=None):
    """
    Approximates PTP joint motion primitives using Ramer-Douglas-Peucker Algorithm (RDP)
    and plots each joint's trajectory before and after simplification.

    Args:
        joint_positions (List[np.ndarray]): List of joint position vectors (shape: dof).
        joint_names (List[str]): List of joint names corresponding to the joint positions.
        epsilon (float): RDP simplification epsilon.
        plot_filepath (str, optional): If provided, save the plot to this path.

    Returns:
        MotionSequence: ROS2 message containing motion primitives.
    """
    points = np.array(joint_positions)
    reduced_points = rdp(points, epsilon=epsilon, dist=pldist_nd)

    motion_sequence = MotionSequence()
    motion_primitives = []

    for pt in reduced_points[1:]:  # Skip the first point (start position)
        primitive = MotionPrimitive()
        primitive.type = MotionPrimitive.LINEAR_JOINT
        primitive.blend_radius = blend_radius
        primitive.additional_arguments = [
            MotionArgument(argument_name="velocity", argument_value=velocity),
            MotionArgument(argument_name="acceleration", argument_value=acceleration),
        ]
        primitive.joint_positions = pt.tolist()
        motion_primitives.append(primitive)

        print(f"Added LINEAR_JOINT (PTP): joints={pt.tolist()}, blend_radius={blend_radius}, "
              f"velocity={velocity}, acceleration={acceleration}")

    motion_sequence.motions = motion_primitives

    print(f"Reduced {len(points)} joint positions to {len(reduced_points)-1} PTP primitives using RDP with epsilon={epsilon}")

    # Plot Joint Axes
    num_joints = points.shape[1]
    fig, axs = plt.subplots(num_joints, 1, figsize=(10, 2.5 * num_joints), sharex=True)

    if num_joints == 1:
        axs = [axs]  # make iterable

    reduced_indices = []
    for pt in reduced_points:
        match = np.where((points == pt).all(axis=1))[0]
        reduced_indices.append(match[0] if len(match) > 0 else None)

    for i in range(num_joints):
        axs[i].plot(range(len(points)), points[:, i], marker='o', color='blue', alpha=0.5, label='Original')
        axs[i].plot(reduced_indices, reduced_points[:, i], marker='o', color='orange', label='Simplified with RDP')

        axs[i].set_ylim([-3.2, 3.2])
        axs[i].set_ylabel("Joint position in rad")
        title = joint_names[i] if i < len(joint_names) else f"Joint {i + 1}"
        axs[i].set_title(title, pad=10)
        axs[i].grid(True)

    axs[-1].set_xlabel("Trajectory point index")

    # Legend only once, below the last plot, right aligned
    handles, labels = axs[-1].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower right', bbox_to_anchor=(0.98, 0.01))

    fig.suptitle('Joint-trajectories before and after RDP simplification for PTP primitives', fontsize=14)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    if plot_filepath:
        plt.savefig(plot_filepath, dpi=300)
        print(f"PTP joint plot saved to {plot_filepath}")

    plt.show(block=False)

    return motion_sequence

