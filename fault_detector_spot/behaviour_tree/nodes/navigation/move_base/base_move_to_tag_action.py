from bosdyn.api.geometry_pb2 import SE2VelocityLimit
from bosdyn.client import math_helpers
from bosdyn.client.robot_command import RobotCommandBuilder

import tf_transformations as tf
from bosdyn_msgs.conversions import convert
from fault_detector_spot.behaviour_tree.commands.base_tag_command import BaseTagCommand
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import ActionClientBehaviour
from geometry_msgs.msg import PoseStamped
from py_trees.common import Access
from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import namespace_with


class BaseMoveToTagAction(ActionClientBehaviour):
    """
    Moves the robot base to the tag with the given offset, using low speed limits.
    """

    def __init__(self, name="BaseMoveToTagAction", robot_name="", duration=3.0):
        super().__init__(name)
        self.robot_name = robot_name
        self.duration = duration
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="last_command", access=Access.READ)

    def _init_client(self):
        action_ns = namespace_with(self.robot_name, "robot_command")
        self._client = ActionClientWrapper(RobotCommand, action_ns, self.node)
        self.initialized = True
        return True

    def _build_goal(self) -> RobotCommand.Goal:
        if not isinstance(self.blackboard.last_command, BaseTagCommand):
            raise RuntimeError("Expected BaseTagCommand on blackboard.last_command")

        target: PoseStamped = self.blackboard.last_command.goal_pose

        # Build low-speed mobility params
        low_speed_params = RobotCommandBuilder.mobility_params()
        low_speed_params.vel_limit.CopyFrom(
            SE2VelocityLimit(
                max_vel=math_helpers.SE2Velocity(0.15, 0.15, 0.2).to_proto(),  # linear x, y and angular z
                min_vel=math_helpers.SE2Velocity(-0.15, -0.15, -0.2).to_proto(),
            )
        )

        cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=target.pose.position.x,
            goal_y=target.pose.position.y,
            goal_heading=tf.euler_from_quaternion([
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            ])[2],
            frame_name=target.header.frame_id,
            params=low_speed_params,
        )

        goal = RobotCommand.Goal()
        convert(cmd, goal.command)
        return goal