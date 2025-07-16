from typing import Optional
import argparse


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with
import synchros2.scope as ros_scope
import synchros2.process as ros_process
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME


class ArmNavigatorNode(Node):
    robot_command_client: Optional[ActionClientWrapper] = None
    tf_listener: Optional[TFListenerWrapper] = None
    logger = None
    odom_frame_name = None
    grav_aligned_body_frame_name = None
    robot_name = None

    def init_spot_arm(self):
        global robot_command_client, tf_listener, logger, odom_frame_name, grav_aligned_body_frame_name
        robot_name = ""
        node = ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use synchros2.process.main?)")
        self.logger = node.get_logger()

        self.robot_command_client = ActionClientWrapper(RobotCommand, namespace_with(robot_name, "robot_command"), node)
        self.tf_listener = TFListenerWrapper(node)
        self.odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
        self.grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
        self.robot_command_client = ActionClientWrapper(RobotCommand, namespace_with(robot_name, "robot_command"), node)
        self.logger.info("{} initialized on frames: {}, {}".format(robot_name, self.odom_frame_name,
                                                                   self.grav_aligned_body_frame_name))

    def __init__(self):
        super().__init__('arm_navigator_node')
        self.init_spot_arm()
        self.subscription = self.create_subscription(
            PoseStamped,
            'arm_goal',
            self.goal_callback,
            10
        )

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info('Received goal pose, passing to move_to_goal()')
        self.move_to_goal(msg)

    def move_to_goal(self, goal_pose: PoseStamped):
        body_frame_pose = self.hand_pose_in_flat_body_frame(goal_pose.pose)
        odom = self.odom_in_flat_body_frame(body_frame_pose)
        action_goal = self.create_arm_command_as_message(odom, 2)

        self.logger().info(
            f"Sending arm command to x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}, z={goal_pose.pose.position.z:.2f}")
        self.robot_command_client.send_goal_and_wait("move_to_goal", action_goal)

    def hand_pose_in_flat_body_frame(self, pose: PoseStamped.pose):
        x_pos = pose.position.x
        y_pos = pose.position.y
        z_pos = pose.position.z
        hand_ewrt_flat_body = geometry_pb2.Vec3(x=x_pos, y=y_pos, z=z_pos)

        x_orient = pose.orientation.x
        y_orient = pose.orientation.y
        z_orient = pose.orientation.z
        w_orient = pose.orientation.w

        flat_body_q_hand = geometry_pb2.Quaternion(w=w_orient, x=x_orient, y=y_orient, z=z_orient)
        flat_body_t_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body, rotation=flat_body_q_hand)
        return flat_body_t_hand

    def odom_in_flat_body_frame(self, body_frame_pose: geometry_pb2.SE3Pose):
        odom_T_flat_body = self.tf_listener.lookup_a_tform_b(self.odom_frame_name, self.grav_aligned_body_frame_name)
        odom_T_flat_body_se3 = math_helpers.SE3Pose(
            odom_T_flat_body.transform.translation.x,
            odom_T_flat_body.transform.translation.y,
            odom_T_flat_body.transform.translation.z,
            math_helpers.Quat(
                odom_T_flat_body.transform.rotation.w,
                odom_T_flat_body.transform.rotation.x,
                odom_T_flat_body.transform.rotation.y,
                odom_T_flat_body.transform.rotation.z,
            ),
        )

        return odom_T_flat_body_se3 * math_helpers.SE3Pose.from_obj(body_frame_pose)

    def create_arm_command_as_message(self, odom, seconds=2):
        arm_command = RobotCommandBuilder.arm_pose_command(
            odom.x,
            odom.y,
            odom.z,
            odom.rot.w,
            odom.rot.x,
            odom.rot.y,
            odom.rot.z,
            ODOM_FRAME_NAME,
            seconds,
        )
        action_goal = RobotCommand.Goal()
        convert(arm_command, action_goal.command)
        return action_goal

@ros_process.main()
def main() -> None:
    node = ArmNavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()