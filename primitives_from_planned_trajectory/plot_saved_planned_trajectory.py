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

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

# Default path and filename (used if no CLI argument is given)
DEFAULT_DIR = 'src/primitives_from_planned_trajectory/data/saved_trajectories'
# DEFAULT_FILENAME = 'planned_trajectory_20250616_102657_pilz_lin.csv'
# DEFAULT_FILENAME = 'planned_trajectory_20250616_102738_pilz_ptp.csv'
# DEFAULT_FILENAME = 'planned_trajectory_20250616_102206_ompl_with_obstacle.csv'
DEFAULT_FILENAME = 'trajectory_20250617_153137_planned.csv'

def load_trajectory_csv(filepath):
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        data = [row for row in reader]
    return data, reader.fieldnames

def plot_trajectory(data, fieldnames, filepath):
    time = [float(row['time_from_start']) for row in data]
    num_steps = len(time)
    filename = os.path.basename(filepath)
    basename, _ = os.path.splitext(filename)  # remove .csv

    pos_fields = [f for f in fieldnames if f.endswith('_pos')]
    vel_fields = [f for f in fieldnames if f.endswith('_vel')]
    acc_fields = [f for f in fieldnames if f.endswith('_acc')]

    cart_pos_fields = ['fk_x', 'fk_y', 'fk_z']
    cart_quat_fields = ['fk_qx', 'fk_qy', 'fk_qz', 'fk_qw']
    has_cart_ee = all(f in fieldnames for f in cart_pos_fields + cart_quat_fields)

    n_rows = 5 if has_cart_ee else 3
    fig, axes = plt.subplots(n_rows, 1, sharex=True, figsize=(12, 12 if has_cart_ee else 8))
    fig.suptitle(f'Trajectory: {filename}   |   Steps: {num_steps}', fontsize=14)

    def plot_category(ax, fields, title, ylabel):
        for field in fields:
            values = [float(row[field]) for row in data]
            ax.plot(time, values, label=field, marker='o', markersize=3)
        ax.set_ylabel(ylabel)
        ax.set_title(title, loc='left')
        ax.grid(True)
        ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), borderaxespad=0)

    plot_category(axes[0], pos_fields, 'Joint Positions over Time', 'Position in rad')
    plot_category(axes[1], vel_fields, 'Joint Velocities over Time', 'Velocity in rad/s')
    plot_category(axes[2], acc_fields, 'Joint Accelerations over Time', 'Acceleration in rad/s²')

    if has_cart_ee:
        plot_category(axes[3], cart_pos_fields, 'EE Cartesian Position over Time', 'Position in m')
        plot_category(axes[4], cart_quat_fields, 'EE Orientation (Quaternion) over Time', 'Quaternion Value')

    axes[-1].set_xlabel('Time in seconds')
    plt.tight_layout(rect=[0, 0, 0.85, 0.95])
    plt.savefig(f'{DEFAULT_DIR}/{basename}_timeplots.png')
    plt.show()

    if has_cart_ee:
        fig3d = plt.figure(figsize=(8, 6))
        ax3d = fig3d.add_subplot(111, projection='3d')
        x = [float(row['fk_x']) for row in data]
        y = [float(row['fk_y']) for row in data]
        z = [float(row['fk_z']) for row in data]
        ax3d.plot(x, y, z, marker='o', label='EE Path')

        # mark start and end points
        ax3d.scatter(x[0], y[0], z[0], color='red', s=50, label='Start')
        ax3d.scatter(x[-1], y[-1], z[-1], color='green', s=50, label='End')

        ax3d.set_title('End-Effector Position in 3D Space')
        ax3d.set_xlabel('X in m')
        ax3d.set_ylabel('Y in m')
        ax3d.set_zlabel('Z in m')
        ax3d.grid(True)
        ax3d.legend()

        # scale axis equal
        max_range = max(
            max(x) - min(x),
            max(y) - min(y),
            max(z) - min(z)
        ) / 2.0
        mid_x = (max(x) + min(x)) / 2.0
        mid_y = (max(y) + min(y)) / 2.0
        mid_z = (max(z) + min(z)) / 2.0
        ax3d.set_xlim(mid_x - max_range, mid_x + max_range)
        ax3d.set_ylim(mid_y - max_range, mid_y + max_range)
        ax3d.set_zlim(mid_z - max_range, mid_z + max_range)

        plt.tight_layout()
        plt.savefig(f'{DEFAULT_DIR}/{basename}_3dplot.png')
        plt.show()


def main():
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        filepath = filename if os.path.isabs(filename) else os.path.join(DEFAULT_DIR, filename)
    else:
        filepath = os.path.join(DEFAULT_DIR, DEFAULT_FILENAME)

    if not os.path.isfile(filepath):
        print(f"[ERROR] File not found: {filepath}")
        return

    print(f"[INFO] Loading trajectory from: {filepath}")
    data, fieldnames = load_trajectory_csv(filepath)
    plot_trajectory(data, fieldnames, filepath)

if __name__ == '__main__':
    main()