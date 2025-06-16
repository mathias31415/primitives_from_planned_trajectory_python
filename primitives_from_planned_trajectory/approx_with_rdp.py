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
import pandas as pd
import numpy as np
from rdp import rdp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

FILE_DIR = 'src/primitives_from_planned_trajectory/data/saved_trajectories'
# FILE_NAME = 'planned_trajectory_20250616_102657_pilz_lin.csv'
FILE_NAME = 'planned_trajectory_20250616_102206_ompl_with_obstacle.csv'
RDP_EPSILON = 0.01  # Toleranzwert für RDP

file_path = os.path.join(FILE_DIR, FILE_NAME)
df = pd.read_csv(file_path)

points = df[['fk_x', 'fk_y', 'fk_z']].values

reduced_points = rdp(points, epsilon=RDP_EPSILON)

fig3d = plt.figure(figsize=(8, 6))
ax3d = fig3d.add_subplot(111, projection='3d')

x = points[:, 0]
y = points[:, 1]
z = points[:, 2]

ax3d.plot(x, y, z, marker='o', label='Original Path', color='blue', alpha=0.5)
ax3d.plot(reduced_points[:, 0], reduced_points[:, 1], reduced_points[:, 2],
          marker='o', label='RDP Path', color='orange')

ax3d.scatter(x[0], y[0], z[0], color='red', s=50, label='Start')
ax3d.scatter(x[-1], y[-1], z[-1], color='green', s=50, label='End')

ax3d.set_xlabel('X')
ax3d.set_ylabel('Y')
ax3d.set_zlabel('Z')
ax3d.set_title('3D-Trajektorie mit RDP-Reduktion')
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

plt.show()
