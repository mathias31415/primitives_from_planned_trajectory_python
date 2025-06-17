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

import os
import csv

def write_to_csv(joint_names, points, fk_poses, filename, directory=None):
    fieldnames = ['time_from_start']
    for name in joint_names:
        fieldnames.append(f'{name}_pos')
    for name in joint_names:
        fieldnames.append(f'{name}_vel')
    for name in joint_names:
        fieldnames.append(f'{name}_acc')
    fieldnames += ['fk_x', 'fk_y', 'fk_z', 'fk_qx', 'fk_qy', 'fk_qz', 'fk_qw']

    os.makedirs(directory, exist_ok=True)
    filepath = os.path.join(directory, filename)

    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for point, fk_pose in zip(points, fk_poses):
            row = {
                'time_from_start': point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            }
            for name, pos in zip(joint_names, point.positions):
                row[f'{name}_pos'] = pos
            for name, vel in zip(joint_names, point.velocities or []):
                row[f'{name}_vel'] = vel
            for name, acc in zip(joint_names, point.accelerations or []):
                row[f'{name}_acc'] = acc

            if fk_pose:
                row.update({
                    'fk_x': fk_pose.position.x,
                    'fk_y': fk_pose.position.y,
                    'fk_z': fk_pose.position.z,
                    'fk_qx': fk_pose.orientation.x,
                    'fk_qy': fk_pose.orientation.y,
                    'fk_qz': fk_pose.orientation.z,
                    'fk_qw': fk_pose.orientation.w
                })
            else:
                for f in ['fk_x', 'fk_y', 'fk_z', 'fk_qx', 'fk_qy', 'fk_qz', 'fk_qw']:
                    row[f] = float('nan')

            writer.writerow(row)

    print(f'Trajectory with FK saved to: {filepath}')
