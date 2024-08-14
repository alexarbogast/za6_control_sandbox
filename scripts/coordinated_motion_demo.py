import numpy as np
import rospy
import actionlib

from geometry_msgs.msg import Vector3
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

from coordinated_control_msgs.msg import RobotSetpoint

NS = "rob1"
LINEAR_VELOCITY = 0.100


def linear_path(start, end, vel):
    diff = end - start
    path_len = np.linalg.norm(diff)
    u = diff / path_len
    dur = path_len / vel

    f = lambda t: start + t * diff
    f_dot = lambda t: vel * u
    return f, f_dot, dur


class ControllerManagerClient(object):
    def __init__(self, ns):
        self.ns = ns
        rospy.wait_for_service(f"/{ns}/controller_manager/switch_controller")
        self.switch_controller_client = rospy.ServiceProxy(
            f"/{ns}/controller_manager/switch_controller", SwitchController
        )

    def switch_controller(self, start_controllers, stop_controllers):
        try:
            req = SwitchControllerRequest()
            req.start_controllers = start_controllers
            req.stop_controllers = stop_controllers
            req.strictness = 1
            self.switch_controller_client.call(req)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")


class CoordinatedMotionDemo(object):
    def __init__(self):
        self.joint_controller_name = "position_trajectory_controller"
        self.base_frame_controller_name = "task_space_controller"

        self.home = [0.234, 0.477, 0.605, 1.387, -1.474, -0.480]
        self.joint_names = [
            f"{NS}_joint_1",
            f"{NS}_joint_2",
            f"{NS}_joint_3",
            f"{NS}_joint_4",
            f"{NS}_joint_5",
            f"{NS}_joint_6",
        ]

        self.task_space_setpoint_pub = rospy.Publisher(
            f"/{NS}/{self.base_frame_controller_name}/setpoint",
            RobotSetpoint,
            latch=True,
            queue_size=1,
        )

        print("waiting for follow_joint_trajectory")
        self._joint_traj_client = actionlib.SimpleActionClient(
            f"{NS}/{self.joint_controller_name}/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._joint_traj_client.wait_for_server()
        self._controller_client = ControllerManagerClient(NS)

    def run(self):
        self.start_joint_control()
        self.move_joint(self.home, 3.0)

        self.start_base_frame_control()
        self.test_cube()

        self.start_joint_control()
        self.move_joint(np.zeros(6), 3.0)

    def test_cube(self):
        rate = rospy.Rate(1000)
        setpoint = RobotSetpoint()
        setpoint.pose.aiming = Vector3(0.0, 0.0, 1.0)

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

        points.insert(0, center)
        for i in range(len(points) - 1):
            current = points[i]
            next = points[i + 1]

            g, g_dot, dur = linear_path(current, next, LINEAR_VELOCITY)

            for t in np.linspace(0, 1, int(dur * 1000)):
                gt = g(t)
                g_dott = g_dot(t)
                setpoint.pose.position = Vector3(gt[0], gt[1], gt[2])
                setpoint.velocity = Vector3(g_dott[0], g_dott[1], g_dott[2])
                self.task_space_setpoint_pub.publish(setpoint)
                rate.sleep()

    def move_joint(self, joint_goal, duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_goal
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]

        self._joint_traj_client.send_goal(goal)
        self._joint_traj_client.wait_for_result()

    def start_joint_control(self):
        self._controller_client.switch_controller(
            [self.joint_controller_name], [self.base_frame_controller_name]
        )

    def start_base_frame_control(self):
        self._controller_client.switch_controller(
            [self.base_frame_controller_name],
            [self.joint_controller_name],
        )


if __name__ == "__main__":
    rospy.init_node("coordinated_motion_demo")

    try:
        demo = CoordinatedMotionDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass
