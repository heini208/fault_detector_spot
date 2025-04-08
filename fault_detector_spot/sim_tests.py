import rclpy
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import curses  # For reading WASD input
from tf2_ros import Buffer, TransformListener, ConnectivityException, LookupException, ExtrapolationException
import geometry_msgs.msg  # For tf2 transformations

def move_in_robot_frame(arm_point, direction):
    # direction is a tuple (dx, dy, dz) representing the relative movement in x, y, and z
    arm_point.positions[0] += direction[0]  # Adjust based on x-axis (forward/backward)
    arm_point.positions[1] += direction[1]  # Adjust based on y-axis (left/right)
    arm_point.positions[2] += direction[2]  # Adjust based on z-axis (up/down)

    return arm_point

def main():
    rclpy.init()
    node = rclpy.create_node("retract_manipulator_node")

    # Initialize action clients for arm and gripper
    arm_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/spotarm_joint_trajectory_controller/follow_joint_trajectory",
    )

    gripper_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/tiago_gripper_joint_trajectory_controller/follow_joint_trajectory",
    )

    arm_client.wait_for_server()
    gripper_client.wait_for_server()

    # Create a goal message for the arm
    arm_goal_msg = FollowJointTrajectory.Goal()
    arm_goal_msg.trajectory.joint_names = [
        "spotarm_1_joint", "spotarm_2_joint", "spotarm_3_joint", "spotarm_4_joint",
        "Slider11", "spotarm_5_joint", "spotarm_6_joint",
    ]

    arm_point = JointTrajectoryPoint()
    arm_point.positions = [
        0.0,  # spotarm_1_joint
        math.radians(0),  # spotarm_2_joint
        math.radians(0),  # spotarm_3_joint
        0.0,  # spotarm_4_joint
        0.0,  # Slider11 (move left/right)
        math.radians(11),  # spotarm_5_joint
        0.0,  # spotarm_6_joint
    ]
    arm_goal_msg.trajectory.points.append(arm_point)

    # Create a goal message for the gripper (add this part)
    gripper_goal_msg = FollowJointTrajectory.Goal()
    gripper_goal_msg.trajectory.joint_names = [
        "gripper_left_finger_joint",
        "gripper_right_finger_joint",
    ]
    gripper_point = JointTrajectoryPoint()
    gripper_point.positions = [0.045, 0.045]  # Open gripper
    gripper_goal_msg.trajectory.points.append(gripper_point)

    # Send action goal for the arm and gripper
    arm_future = arm_client.send_goal_async(arm_goal_msg)
    gripper_future = gripper_client.send_goal_async(gripper_goal_msg)

    # Wait for initial goal completion
    rclpy.spin_until_future_complete(node, arm_future)
    rclpy.spin_until_future_complete(node, gripper_future)

    # Wait for simulation clock to initialize
    clock_msg_count = 0
    def increment_count(_):
        nonlocal clock_msg_count
        clock_msg_count += 1

    node.create_subscription(Clock, "/clock", increment_count, 1)
    while clock_msg_count < 40:
        rclpy.spin_once(node)

    # Set up tf2 listener to get robot's current pose
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    # Use curses to capture user input (WASD keys)
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(True)

    try:
        while True:
            key = stdscr.getch()  # Get the key press

            # Try to get the transform (current pose of robot's end effector)
            try:
                # Use the correct base frame here, e.g., "spot_base" or another frame in your robot's TF tree
                transform = tf_buffer.lookup_transform('spot_base', 'spotarm_link', rclpy.time.Time())  # Replace 'world' with 'spot_base'
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                node.get_logger().warn(f"Could not get transform: {e}")
                continue

            # Check for WASD key presses and modify arm goal accordingly
            if key == ord('w'):  # Move up (in the local frame)
                direction = (0, 0, 0.05)  # Move 5 cm upwards in local Z
            elif key == ord('s'):  # Move down
                direction = (0, 0, -0.05)  # Move 5 cm down in local Z
            elif key == ord('a'):  # Move left
                direction = (-0.05, 0, 0)  # Move 5 cm left in local X
            elif key == ord('d'):  # Move right
                direction = (0.05, 0, 0)  # Move 5 cm right in local X
            else:
                continue  # Skip if no valid key press

            # Move the arm relative to its current pose
            arm_point = move_in_robot_frame(arm_point, direction)

            # Update the arm goal with new position
            arm_goal_msg.trajectory.points = [arm_point]
            arm_future = arm_client.send_goal_async(arm_goal_msg)

            # Wait for the arm goal to complete
            rclpy.spin_until_future_complete(node, arm_future)
            if arm_future.result().accepted:
                node.get_logger().info(f"Arm moved to new position: {arm_point.positions}")
            else:
                node.get_logger().warn("Arm goal was not completed successfully.")

    finally:
        curses.endwin()  # Clean up curses

    # Clean up and shut down
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
