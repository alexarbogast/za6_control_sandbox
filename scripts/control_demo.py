import numpy as np
import rospy

from geometry_msgs.msg import Quaternion

from taskspace_control_examples import ControlDemo
from taskspace_control_examples.trajectory import *


class BaseframeControlDemo(ControlDemo):
    def __init__(self, setpoint_hz=1000):
        super(BaseframeControlDemo, self).__init__(setpoint_hz)
        self.static_orient = Quaternion(0, 0, 0, 1)
        self.home = [0.238, 0.496, 0.568, 1.498, -1.531, -0.505]
        self.arm_id = rospy.get_param("~arm_id")

    def run(self):
        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 2.0)

        self.start_taskspace_control()

        self.test_cube()
        self.base_frame_circle()
        self.base_frame_hypotrochoid()

        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 3.0)

    def test_cube(self):
        center = np.array([0.5, 0.0, 0.5])
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [0.0, 0.5, -0.15],
                [-1.0, 0.5, -0.15],
                [-1.0, 0.5, 0.50],
                [0.0, 0.5, 0.50],
                [0.0, -0.5, 0.50],
                [-1.0, -0.5, 0.50],
                [-1.0, -0.5, -0.15],
                [0.0, -0.5, -0.15],
                [0.0, 0.5, -0.15],
            ]
        )
        points += center
        self.path_viz.visualize_path(points[1:], f"{self.arm_id}_base_link")

        for i in range(len(points) - 1):
            current = points[i]
            next = points[i + 1]
            self.execute_linear_path(current, next, 2.0)

        self.path_viz.reset()

    def base_frame_circle(self):
        tf = 5
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = circular_traj(1 / 5, tf)

        offset = np.array([0.5, 0.0, 0.2])
        ft, f_dott = f(tt) + offset, f_dot(tt)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, tf, 500)],
            f"{self.arm_id}_base_link",
        )

        self.movel(ft[0], 2)
        self.execute_path(ft, f_dott)
        self.path_viz.reset()

    def base_frame_hypotrochoid(self):
        scale = 1 / 28
        tf = 10
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = hypotrochoid_traj(3, 5, 4.5, tf, scaling=Order.THIRD)

        offset = np.array([0.5, 0.0, 0.2])
        ft, f_dott = scale * f(tt) + offset, scale * f_dot(tt)

        self.path_viz.visualize_path(
            [scale * f(t) + offset for t in np.linspace(0, tf, 500)],
            f"{self.arm_id}_base_link",
        )

        self.movel(ft[0], 1)
        self.execute_path(ft, f_dott)
        self.path_viz.reset()


if __name__ == "__main__":
    rospy.init_node("base_frame_control_client")

    try:
        demo = BaseframeControlDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass
