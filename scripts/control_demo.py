#!/usr/bin/env python3

# Copyright 2024 Alex Arbogast
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

import numpy as np

import rclpy
import threading


from std_msgs.msg import ColorRGBA

from taskspace_control_examples import ControlDemo, PathVisualization
from taskspace_control_examples.trajectory import circular_traj


NODE_NAME = "za6_control_demo"


class Za6ControlDemo(ControlDemo):
    def __init__(self, node_name: str, setpoint_hz=250):
        super().__init__(node_name, setpoint_hz)

        self.path_viz = PathVisualization(
            self, 0.007, ColorRGBA(r=0.96, g=0.38, b=0.21, a=1.0)
        )

        self.static_orient = np.array([0.0, 0.70710678, 0.0, 0.70710678])

    def run(self):
        self.test_line()
        # self.test_circle()

    def test_line(self):
        tf = 3
        p_start = np.array([0.5, 0.3, 0.25])
        p_end = np.array([0.5, -0.3, 0.25])
        self.path_viz.visualize_path([p_start, p_end], "base_link")

        self.movel(p_start, self.static_orient, 3)
        self.execute_linear_path(
            p_start, p_end, self.static_orient, self.static_orient, tf
        )
        self.path_viz.reset()

    def test_circle(self):
        tf = 5
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = circular_traj(1 / 7, tf)

        offset = np.array([0.6, 0.0, 0.2])
        ft, f_dott = f(tt) + offset, f_dot(tt)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, tf, 500)], "base_link"
        )

        self.movel(ft[0], self.static_orient, 2)
        self.execute_path(ft, f_dott, self.static_orient)
        self.path_viz.reset()


def main(args=None):
    rclpy.init(args=args)
    node = Za6ControlDemo(NODE_NAME)

    try:
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        node.run()
    except Exception as e:
        node.get_logger().error(f"Exception in demo: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
