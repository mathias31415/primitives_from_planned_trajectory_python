
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

# joint_state_recorder.py

import csv
import os
from threading import Event
from sensor_msgs.msg import JointState


class JointStateLogger:
    def __init__(self, node, filepath, topic_name='/joint_states'):
        self.node = node
        self.filepath = filepath
        self.topic_name = topic_name
        self.joint_names = []
        self._stop_event = Event()
        self._header_written = False

        self._file = open(self.filepath, mode='w', newline='')
        self._writer = csv.writer(self._file)

        self.sub = self.node.create_subscription(
            JointState,
            self.topic_name,
            self._callback,
            10
        )

        self.node.get_logger().info(f"Started recording joint states to {self.filepath}")

    def _callback(self, msg):
        if not self._header_written:
            self.joint_names = msg.name
            header = ['timestamp']
            header += [f"{n}_pos" for n in msg.name]
            header += [f"{n}_vel" for n in msg.name]
            header += [f"{n}_eff" for n in msg.name]
            self._writer.writerow(header)
            self._header_written = True

        now = self.node.get_clock().now().to_msg()
        timestamp = now.sec + now.nanosec * 1e-9
        row = [timestamp]
        row += list(msg.position)
        row += list(msg.velocity)
        row += list(msg.effort)
        self._writer.writerow(row)

    def stop(self):
        self._stop_event.set()
        self.sub.destroy()
        self._file.close()
        self.node.get_logger().info(f"Stopped recording joint states. File saved to {self.filepath}")
