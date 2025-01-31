import numpy as np
import quaternion
import rospy

from taskspace_control_examples import ControlDemo
from taskspace_control_examples.trajectory import *

robot_params = {
    "pose_controller": {
        "home": [0.484, 0.425, 0.661, 1.959, -1.77, -0.446],
    },
    "as_nullspace_controller": {
        "home": [0.484, 0.425, 0.661, 1.959, -1.77, -0.446],
    },
}


class BaseframeControlDemo(ControlDemo):
    def __init__(self, setpoint_hz=1000):
        super(BaseframeControlDemo, self).__init__(setpoint_hz)
        controller_type = rospy.get_param("~controller", "pose_controller")
        self.home = robot_params[controller_type]["home"]
        self.static_orient = np.quaternion(1.0, 0.0, 0.0, 0.0)
        self.arm_id = rospy.get_param("~arm_id")

    def run(self):
        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 2.0)

        self.start_taskspace_control()

        self.test_line()
        self.test_cube()
        self.base_frame_circle()
        self.base_frame_hypotrochoid()

        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 3.0)

    def test_line(self):
        tf = 3
        p_start = np.array([0.5, 0.3, 0.1])
        p_end = np.array([0.5, -0.3, 0.1])
        self.path_viz.visualize_path([p_start, p_end], f"{self.arm_id}_base_link")

        self.movel(p_start, self.static_orient, 3)
        self.execute_linear_path(
            p_start, p_end, self.static_orient, self.static_orient, tf
        )
        self.path_viz.reset()

    def test_cube(self):
        center = np.array([0.4, 0.0, 0.35])
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [-0.05, 0.5, -0.25],
                [-1.0, 0.5, -0.25],
                [-1.0, 0.5, 0.45],
                [-0.05, 0.5, 0.45],
                [-0.05, -0.5, 0.45],
                [-1.0, -0.5, 0.45],
                [-1.0, -0.5, -0.25],
                [-0.05, -0.5, -0.25],
                [-0.05, 0.5, -0.25],
            ]
        )
        points += center
        self.path_viz.visualize_path(points[1:], f"{self.arm_id}_base_link")

        self.movel(center, self.static_orient, 2)
        for i in range(len(points) - 1):
            current = points[i]
            next = points[i + 1]
            self.execute_linear_path(
                current, next, self.static_orient, self.static_orient, 2.0
            )

        self.path_viz.reset()

    def base_frame_circle(self):
        tf = 5
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = circular_traj(1 / 7, tf)

        offset = np.array([0.5, 0.0, 0.2])
        ft, f_dott = f(tt) + offset, f_dot(tt)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, tf, 500)],
            f"{self.arm_id}_base_link",
        )

        self.movel(ft[0], self.static_orient, 2)
        self.execute_path(ft, f_dott, self.static_orient)
        self.path_viz.reset()

    def base_frame_hypotrochoid(self):
        scale = 1 / 30
        tf = 10
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = hypotrochoid_traj(3, 5, 4.5, tf, scaling=Order.THIRD)

        offset = np.array([0.45, 0.0, 0.2])
        ft, f_dott = scale * f(tt) + offset, scale * f_dot(tt)

        self.path_viz.visualize_path(
            [scale * f(t) + offset for t in np.linspace(0, tf, 500)],
            f"{self.arm_id}_base_link",
        )

        self.movel(ft[0], self.static_orient, 1)
        self.execute_path(ft, f_dott, self.static_orient)
        self.path_viz.reset()


if __name__ == "__main__":
    rospy.init_node("base_frame_control_client")

    try:
        demo = BaseframeControlDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass
