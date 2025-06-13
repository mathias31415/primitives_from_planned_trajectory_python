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
import os
import sys

# Default path and filename (used if no CLI argument is given)
DEFAULT_DIR = 'src/primitives_from_planned_trajectory/data/saved_trajectories'
DEFAULT_FILENAME = 'trajectory_20250612_160037.csv'

def load_trajectory_csv(filepath):
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        data = [row for row in reader]
    return data, reader.fieldnames

import os  # falls noch nicht importiert

def plot_trajectory(data, fieldnames, filepath):
    time = [float(row['time_from_start']) for row in data]
    num_steps = len(time)
    filename = os.path.basename(filepath)

    pos_fields = [f for f in fieldnames if f.endswith('_pos')]
    vel_fields = [f for f in fieldnames if f.endswith('_vel')]
    acc_fields = [f for f in fieldnames if f.endswith('_acc')]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    
    # 🟢 Titel mit Dateiname und Zeitpunkten
    fig.suptitle(f'Trajectory: {filename}   |   Steps: {num_steps}', fontsize=14)

    def plot_category(ax, fields, title, ylabel):
        for field in fields:
            values = [float(row[field]) for row in data]
            ax.plot(time, values, label=field, marker='o', markersize=3)
        ax.set_ylabel(ylabel)
        ax.set_title(title, loc='left')
        ax.grid(True)
        ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), borderaxespad=0)

    plot_category(axes[0], pos_fields, 'Positions over Time', 'Position [rad]')
    plot_category(axes[1], vel_fields, 'Velocities over Time', 'Velocity [rad/s]')
    plot_category(axes[2], acc_fields, 'Accelerations over Time', 'Acceleration [rad/s²]')

    axes[2].set_xlabel('Time [s]')
    plt.tight_layout(rect=[0, 0, 0.85, 0.95])
    plt.show()


def main():
    # Use filename from CLI argument if provided
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

