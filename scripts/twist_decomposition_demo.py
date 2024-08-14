import numpy as np
import rospy

from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from coordinated_control_msgs.msg import AxiallySymmetricSetpoint

from coordinated_motion_examples import (
    ControllerClient,
    JointControllerClient,
    ControllerManagerClient,
    PathVisualization,
)
from coordinated_motion_examples.path import linear_path, hypotrochoid

NS = "rob1"
LINEAR_VELOCITY = 0.300


class NullspaceControlDemo:
    def __init__(self):
        self.controller_client = ControllerClient("twist_decomposition_controller")
        self.joint_controller_client = JointControllerClient(
            "position_trajectory_controller"
        )

        self.controller_manager_client = ControllerManagerClient()
        self.path_viz = PathVisualization(0.007, ColorRGBA(0.96, 0.38, 0.21, 1.0), NS)

        # home flange x = 0.5, y = 0, z = 0.5
        self.home = [0.238, 0.496, 0.568, 1.498, -1.531, -0.505]

    def run(self):
        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 3.0)

        self.start_taskspace_control()
        self.test_cube()
        self.base_frame_hypotrochoid()

        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 3.0)

    def test_cube(self):
        rate = rospy.Rate(1000)
        setpoint = AxiallySymmetricSetpoint()
        setpoint.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        center = np.array([0.5, 0.0, 0.5])
        points = [
            center + np.array([0.0, 0.5, -0.15]),
            center + np.array([-1.0, 0.5, -0.15]),
            center + np.array([-1.0, 0.5, 0.50]),
            center + np.array([0.0, 0.5, 0.50]),
            center + np.array([0.0, -0.5, 0.50]),
            center + np.array([-1.0, -0.5, 0.50]),
            center + np.array([-1.0, -0.5, -0.15]),
            center + np.array([0.0, -0.5, -0.15]),
            center + np.array([0.0, 0.5, -0.15]),
        ]

        self.path_viz.visualize_path(points, f"{NS}_base_link")

        points.insert(0, center)
        for i in range(len(points) - 1):
            current = points[i]
            next = points[i + 1]

            g, g_dot, dur = linear_path(current, next, LINEAR_VELOCITY)

            for t in np.linspace(0, 1, int(dur * 1000)):
                gt = g(t)
                g_dott = g_dot(t)
                setpoint.pose.position = Point(gt[0], gt[1], gt[2])
                setpoint.velocity = Vector3(g_dott[0], g_dott[1], g_dott[2])
                self.controller_client.publish_setpoint(setpoint)
                rate.sleep()

        self.path_viz.reset()

    def base_frame_hypotrochoid(self):
        scaling = 1 / 27
        offset = np.array([0.5, 0.0, 0.2])
        tt = np.linspace(0, 6 * np.pi, 10000)
        f, f_dot = hypotrochoid(scaling)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, 6 * np.pi, 500)],
            f"{NS}_base_link",
        )

        rate = rospy.Rate(1000)
        setpoint = AxiallySymmetricSetpoint()
        setpoint.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        # travel to start
        pose = self.controller_client.get_pose()
        if pose is None:
            return
        current_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        init_path_p = f(tt[0]) + offset

        g, g_dot, dur = linear_path(current_position, init_path_p, LINEAR_VELOCITY)
        for t in np.linspace(0, 1, int(dur * 1000)):
            gt = g(t)
            g_dott = g_dot(t)
            setpoint.pose.position = Point(gt[0], gt[1], gt[2])
            setpoint.velocity = Vector3(g_dott[0], g_dott[1], g_dott[2])
            self.controller_client.publish_setpoint(setpoint)
            rate.sleep()

        # follow path
        for t in tt:
            ft = f(t) + offset
            f_dott = f_dot(t)

            setpoint.pose.position = Point(ft[0], ft[1], ft[2])
            setpoint.velocity = Vector3(f_dott[0], f_dott[1], f_dott[2])
            self.controller_client.publish_setpoint(setpoint)
            rate.sleep()

        self.path_viz.reset()

    def start_joint_control(self):
        self.controller_manager_client.switch_controller(
            [self.joint_controller_client.name], [self.controller_client.name]
        )

    def start_taskspace_control(self):
        self.controller_manager_client.switch_controller(
            [self.controller_client.name], [self.joint_controller_client.name]
        )


if __name__ == "__main__":
    rospy.init_node("nullspace_control_client")

    try:
        demo = NullspaceControlDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass

