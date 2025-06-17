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
DEFAULT_FILENAME = 'trajectory_20250617_153137_executed.csv'

def load_trajectory_csv(filepath):
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        data = [row for row in reader]
    return data, reader.fieldnames

def plot_trajectory(data, fieldnames, filepath):
    time = [float(row['timestamp']) for row in data]
    num_steps = len(time)
    filename = os.path.basename(filepath)
    basename, _ = os.path.splitext(filename)  # remove .csv

    pos_fields = [f for f in fieldnames if f.endswith('_pos')]
    vel_fields = [f for f in fieldnames if f.endswith('_vel')]
    eff_fields = [f for f in fieldnames if f.endswith('_eff')]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
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
    plot_category(axes[2], eff_fields, 'Joint Efforts over Time', 'Effort in Nm')

    axes[-1].set_xlabel('Time in seconds')
    plt.tight_layout(rect=[0, 0, 0.85, 0.95])
    plt.savefig(f'{DEFAULT_DIR}/{basename}_timeplots.png')
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