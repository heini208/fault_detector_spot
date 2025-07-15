import argparse
import sys
import termios
import tty

from typing import Optional

from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with
import synchros2.scope as ros_scope
import synchros2.process as ros_process

from spot_msgs.action import RobotCommand 
from spot_examples.simple_spot_commander import SimpleSpotCommander

robot_command_client: Optional[ActionClientWrapper] = None
tf_listener: Optional[TFListenerWrapper] = None
logger = None
odom_frame_name = None
grav_aligned_body_frame_name = None

def move_arm_init():
    global robot_command_client, tf_listener, logger, odom_frame_name, grav_aligned_body_frame_name

    # Move the arm to a spot in front of the robot, and open the gripper.

    # Make the arm pose RobotCommand
    # Build a position to move the arm to (in meters, relative to and expressed in the gravity aligned body frame).
    x = 0.75
    y = 0
    z = 0.25
    hand_ewrt_flat_body = geometry_pb2.Vec3(x=x, y=y, z=z)

    # Rotation as a quaternion
    qw = 1
    qx = 0
    qy = 0
    qz = 0
    flat_body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

    flat_body_T_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body, rotation=flat_body_Q_hand)

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

    # duration in seconds
    seconds = 2

    arm_command = RobotCommandBuilder.arm_pose_command(
        odom_T_hand.x,
        odom_T_hand.y,
        odom_T_hand.z,
        odom_T_hand.rot.w,
        odom_T_hand.rot.x,
        odom_T_hand.rot.y,
        odom_T_hand.rot.z,
        ODOM_FRAME_NAME,
        seconds,
    )

    # Make the open gripper RobotCommand
    gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

    # Combine the arm and gripper commands into one RobotCommand
    command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)

    # Convert to a ROS message
    action_goal = RobotCommand.Goal()
    convert(command, action_goal.command)
    # Send the request and wait until the arm arrives at the goal
    logger.info("Moving arm to position 1.")
    robot_command_client.send_goal_and_wait("arm_move_one", action_goal)

def get_key() -> str:
    """Read one character from stdin."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def keyboard_control(robot_name: Optional[str] = None) -> None:
    global robot_command_client, tf_listener, logger, odom_frame_name, grav_aligned_body_frame_name

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

    # Claim robot
    logger.info("Claiming robot")
    result = robot.command("claim")
    if not result.success:
        node.get_logger().error("Unable to claim robot message was " + result.message)
        return False
    logger.info("Claimed robot")

    # Stand the robot up.
    logger.info("Powering robot on")
    result = robot.command("power_on")
    if not result.success:
        logger.error("Unable to power on robot message was " + result.message)
        return False
    logger.info("Standing robot up")
    result = robot.command("stand")
    if not result.success:
        logger.error("Robot did not stand message was " + result.message)
        return False
    logger.info("Successfully stood up.")


    #move_arm_init()

    x, y, z = 0.75, 0.0, 0.25
    step = 0.05

    logger.info("Ready for keyboard input. Use WASD for X/Y, F/G for Z. Ctrl+C to quit.")

    try:
        while True:
            key = get_key().lower()
            if key == "a":
                y += step
            elif key == "d":
                y -= step
            elif key == "g":
                x -= step
            elif key == "f":
                x += step
            elif key == "w":
                z += step
            elif key == "s":
                z -= step
            elif key == "\x1b" or "q":
                logger.info("Escape key pressed, shutting down.")
                break
            else:
                logger.info(f"Unknown key: {key}")
                continue

            flat_body_pos = geometry_pb2.Vec3(x=x, y=y, z=z)
            flat_body_rot = geometry_pb2.Quaternion(w=1, x=0, y=0, z=0)
            flat_body_T_hand = geometry_pb2.SE3Pose(position=flat_body_pos, rotation=flat_body_rot)

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

            arm_command = RobotCommandBuilder.arm_pose_command(
                odom_T_hand.x,
                odom_T_hand.y,
                odom_T_hand.z,
                odom_T_hand.rot.w,
                odom_T_hand.rot.x,
                odom_T_hand.rot.y,
                odom_T_hand.rot.z,
                ODOM_FRAME_NAME,
                1.0,
            )

            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
            command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)

            action_goal = RobotCommand.Goal()
            convert(command, action_goal.command)

            logger.info(f"Sending arm command to x={x:.2f}, y={y:.2f}, z={z:.2f}")
            robot_command_client.send_goal_and_wait("keyboard_arm", action_goal)

    except KeyboardInterrupt:
        logger.info("Shutting down keyboard control.")


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    keyboard_control(args.robot)


if __name__ == "__main__":
    main()
