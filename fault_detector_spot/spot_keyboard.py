import curses
import os
import threading
import argparse
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from typing import Optional
import math

from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
import synchros2.process as ros_process
import synchros2.scope as ros_scope
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with
from spot_msgs.action import RobotCommand
from spot_examples.simple_spot_commander import SimpleSpotCommander

class KeyboardTeleopNode(Node):

    def __init__(self):
        super().__init__("keyboard_teleop_node")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.robot_name = None
        self.robot_command_client: Optional[ActionClientWrapper] = None
        self.tf_listener: Optional[TFListenerWrapper] = None
        self.odom_frame_name = None
        self.grav_aligned_body_frame_name = None

        self.x, self.y, self.z = 0.75, 0.0, 0.25

        self.yaw_angle = 0.0
        self.yaw_step = math.radians(10)
        self.step = 0.05

        self.VELOCITY_BASE_SPEED = 0.3
        self.VELOCITY_BASE_ANGULAR = 0.5
        self.VELOCITY_CMD_DURATION = 1.0
        self.COMMAND_INPUT_RATE = 0.1
        self.init_robot()

    def init_robot(self):
        global robot_command_client, tf_listener, logger, odom_frame_name, grav_aligned_body_frame_name

        node = ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use synchros2.process.main?)")
        self.logger = node.get_logger()

        self.robot_name = None
        self.odom_frame_name = namespace_with(self.robot_name, ODOM_FRAME_NAME)
        self.grav_aligned_body_frame_name = namespace_with(self.robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
        self.tf_listener = TFListenerWrapper(node)
        self.tf_listener.wait_for_a_tform_b(self.odom_frame_name, self.grav_aligned_body_frame_name)

        self.robot = SimpleSpotCommander(self.robot_name, node)
        self.robot_command_client = ActionClientWrapper(RobotCommand, namespace_with(self.robot_name, "robot_command"), node)

        for cmd in ["claim", "power_on", "stand"]:
            result = self.robot.command(cmd)
            if not result.success:
                self.get_logger().error(f"Unable to {cmd} robot: {result.message}")
                return

        self.get_logger().info("Robot ready for keyboard input")

    def send_arm_command(self, x, y, z):
        flat_body_pos = geometry_pb2.Vec3(x=x, y=y, z=z)
        cr = math.cos(self.yaw_angle * 0.5)
        sr = math.sin(self.yaw_angle * 0.5)
        flat_body_rot = geometry_pb2.Quaternion(w=cr, x=sr, y=0, z=0)

        flat_body_T_hand = geometry_pb2.SE3Pose(position=flat_body_pos, rotation=flat_body_rot)

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
        odom_T_hand = odom_T_flat_body_se3 * math_helpers.SE3Pose.from_obj(flat_body_T_hand)

        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x, odom_T_hand.y, odom_T_hand.z,
            odom_T_hand.rot.w, odom_T_hand.rot.x, odom_T_hand.rot.y, odom_T_hand.rot.z,
            ODOM_FRAME_NAME, 1.0,
        )
        command = RobotCommandBuilder.build_synchro_command(arm_command)

        action_goal = RobotCommand.Goal()
        convert(command, action_goal.command)

        self.get_logger().info(f"Sending arm command to x={x:.2f}, y={y:.2f}, z={z:.2f}")
        self.robot_command_client.send_goal_and_wait("keyboard_arm", action_goal)

    def _velocity_cmd_helper(self, v_x=0.0, v_y=0.0, v_rot=0.0):
        twist = Twist()
        twist.linear.x = v_x
        twist.linear.y = v_y
        twist.angular.z = v_rot
        start_time = time.time()
        while time.time() - start_time < self.VELOCITY_CMD_DURATION:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.01)
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.3)
        self.send_arm_command(self.x, self.y, self.z)

    def start_scan(self):
        global y , x, z
        self.y = (0.15)
        self.send_arm_command(self.x, self.y, self.z)
        for i in range(6):
            self.y -= 0.05
            self.send_arm_command(self.x, self.y, self.z)
        self.y = 0
        self.send_arm_command(self.x, self.y, self.z)

    def start_full_scan(self):
        global y, x, z
        self.z = 0.07
        self.send_arm_command(self.x, self.y, self.z)
        self.y += (0.15)
        self.send_arm_command(self.x, self.y, self.z)
        for i in range(6):
            self.y -= 0.05
            self.send_arm_command(self.x, self.y, self.z)
        self.y = 0
        self.send_arm_command(self.x, self.y, self.z)

def curses_main(stdscr, node):
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0, "Control the Spot robot. Arrow keys move. WASD control arm. G/F/W/S adjust X/Z. ESC to quit.")

    while rclpy.ok():
        keys = []
        while True:
            key = stdscr.getch()
            if key == -1:
                break
            keys.append(key)

        if not keys:
            time.sleep(0.01)
            continue

        key = keys[-1]  # Only process the last key pressed

        if key == ord('8'):
            #forward
            node._velocity_cmd_helper(v_x=node.VELOCITY_BASE_SPEED)
        elif key == ord('5'):
            #backwards
            node._velocity_cmd_helper(v_x=-node.VELOCITY_BASE_SPEED)
        elif key == ord('4'):
            #left strafe
            node._velocity_cmd_helper(v_y=node.VELOCITY_BASE_SPEED)
        elif key == ord('6'):
            #right strafe
            node._velocity_cmd_helper(v_y=-node.VELOCITY_BASE_SPEED)
        elif key == ord('x'):
            node.start_scan()
        elif key == ord('v'):
            node.start_full_scan()
        elif key == ord('7'):
            #turn left
            node._velocity_cmd_helper(v_rot=node.VELOCITY_BASE_ANGULAR)
        elif key == ord('9'):
            #turn right
            node._velocity_cmd_helper(v_rot=-node.VELOCITY_BASE_ANGULAR)
        elif key == ord('a'):
            node.y += node.step
            node.send_arm_command(node.x, node.y, node.z)
        elif key == ord('d'):
            node.y -= node.step
            node.send_arm_command(node.x, node.y, node.z)
        elif key == ord('g'):
            node.x -= node.step
            node.send_arm_command(node.x, node.y, node.z)
        elif key == ord('f'):
            node.x += node.step
            node.send_arm_command(node.x, node.y, node.z)
        elif key == ord('w'):
            node.z += node.step
            node.send_arm_command(node.x, node.y, node.z)
        elif key == ord('s'):
            node.z -= node.step
            node.send_arm_command(node.x, node.y, node.z)
        elif key == ord('q'):
            node.yaw_angle += node.yaw_step
            node.send_arm_command(node.x, node.y, node.z)
        elif key == ord('e'):
            node.yaw_angle -= node.yaw_step
            node.send_arm_command(node.x, node.y, node.z)
        elif key == ord('+'):
            node.step += 0.01
            node.get_logger().info(f"speed now: {node.step}")
        elif key == ord('-'):
            node.step -= 0.01
            node.get_logger().info(f"speed now: {node.step}")
        elif key == 27:  # ESC
            break
        else:
            node.get_logger().info(f"Unknown key: {key}")

        time.sleep(0.05)
def cli():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser

@ros_process.main(cli())
def main(args):
    wasd_interface = KeyboardTeleopNode()
    try:
        # try:
        # Prevent curses from introducing a 1-second delay for ESC key
        os.environ.setdefault("ESCDELAY", "0")
        # Run wasd interface in curses mode, then restore terminal config.
        curses.wrapper(curses_main, wasd_interface)
    except Exception as e:
        wasd_interface.logger.error("WASD has thrown an error:" + str(e))

    return True

if __name__ == "__main__":
    main()