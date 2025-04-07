import argparse
from typing import Optional

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with

from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander
from geometry_msgs.msg import PoseStamped


def tag_follower(robot_name: Optional[str] = None) -> bool:
    node = ros_scope.node()
    if node is None:
        raise ValueError("no ROS 2 node available (did you use synchros2.process.main?)")
    logger = node.get_logger()

    odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
    grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
    tf_listener = TFListenerWrapper(node)
    tf_listener.wait_for_a_tform_b(odom_frame_name, grav_aligned_body_frame_name)

    robot = SimpleSpotCommander(robot_name, node)
    robot_command_client = ActionClientWrapper(RobotCommand, namespace_with(robot_name, "robot_command"), node)

    # Claim and power on robot
    logger.info("Claiming robot")
    result = robot.command("claim")
    if not result.success:
        node.get_logger().error("Unable to claim robot message was " + result.message)
        return False
    logger.info("Claimed robot")

    logger.info("Powering robot on")
    result = robot.command("power_on")
    if not result.success:
        logger.error("Unable to power on robot message was " + result.message)
        return False
    logger.info("Robot powered on")

    # Move arm to a tag position
    logger.info("Starting tag detection and movement...")
    result = detect_tag_and_move_arm(tf_listener, odom_frame_name, grav_aligned_body_frame_name, robot_command_client, node)
    
    if not result:
        logger.error("Failed to detect tag and move arm")
        return False

    return True


def detect_tag_and_move_arm(tf_listener, odom_frame_name, grav_aligned_body_frame_name, robot_command_client, node):
    # Placeholder for the AprilTag detection logic
    # In practice, you'll need to subscribe to a camera topic and detect AprilTags (e.g., via apriltag_ros)
    
    tag_pose_camera = PoseStamped()  # This should come from an AprilTag detection
    tag_pose_camera.pose.position.x = 1.0  # Example x position (from your tag detection)
    tag_pose_camera.pose.position.y = 0.0  # Example y position
    tag_pose_camera.pose.position.z = 0.5  # Example z position

    # Transform the detected pose from the camera frame to the base frame
    try:
        transform = tf_listener.lookup_a_tform_b(odom_frame_name, grav_aligned_body_frame_name)
        tag_pose_base = tf_listener.transform_pose(tag_pose_camera, transform)

        node.get_logger().info(f"Tag pose in base frame: {tag_pose_base.pose.position}")

        # Apply offset (e.g., move the arm 15 cm in front of the tag)
        tag_pose_base.pose.position.z -= 0.15  # Move 15 cm back from tag position

        # Send the arm movement command
        move_result = move_arm(tag_pose_base.pose, robot_command_client, node)

        return move_result

    except Exception as e:
        node.get_logger().error(f"Error in tag detection and transformation: {e}")
        return False


def move_arm(target_pose, robot_command_client, node):
    # Make the arm pose RobotCommand
    hand_ewrt_flat_body = geometry_pb2.Vec3(x=target_pose.position.x, y=target_pose.position.y, z=target_pose.position.z)
    flat_body_Q_hand = geometry_pb2.Quaternion(w=1, x=0, y=0, z=0)  # Adjust quaternion as needed

    flat_body_T_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body, rotation=flat_body_Q_hand)

    # Look up the transform from the odom frame to the body frame
    odom_T_flat_body = tf_listener.lookup_a_tform_b(odom_frame_name, grav_aligned_body_frame_name)
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

    odom_T_hand = odom_T_flat_body_se3 * math_helpers.SE3Pose.from_obj(flat_body_T_hand)

    # Create arm command
    arm_command = RobotCommandBuilder.arm_pose_command(
        odom_T_hand.x,
        odom_T_hand.y,
        odom_T_hand.z,
        odom_T_hand.rot.w,
        odom_T_hand.rot.x,
        odom_T_hand.rot.y,
        odom_T_hand.rot.z,
        ODOM_FRAME_NAME,
        2.0,  # Duration in seconds
    )

    # Build and send the arm movement command
    command = RobotCommandBuilder.build_synchro_command(arm_command)
    action_goal = RobotCommand.Goal()
    convert(command, action_goal.command)

    # Send the request and wait for completion
    node.get_logger().info("Moving arm to target position...")
    robot_command_client.send_goal_and_wait("arm_move_target", action_goal)

    return True


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    hello_arm(args.robot)


if __name__ == "__main__":
    main()
