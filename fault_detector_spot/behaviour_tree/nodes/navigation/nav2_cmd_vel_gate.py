#!/usr/bin/env python3
import rclpy
from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node


class Nav2CmdVelGate(Node):
    """
    Monitors Nav2 goal state and activates/deactivates controller_server
    to ensure Nav2 only controls the base when a goal is active.
    """

    def __init__(self):
        super().__init__("nav2_cmd_vel_gate")

        self.controller_client = self.create_client(ChangeState, "/controller_server/change_state")
        self.goal_status_sub = self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.status_callback,
            10
        )

        self.active_goal = False
        self.get_logger().info("Nav2 CmdVel Gate node started")

    def status_callback(self, msg: GoalStatusArray):
        # Check if any goal is ACCEPTED or EXECUTING
        any_active = any(s.status in (1, 2) for s in msg.status_list)  # 1=ACCEPTED, 2=EXECUTING
        if any_active != self.active_goal:
            self.active_goal = any_active
            self.update_controller()

    def update_controller(self):
        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Controller service not available")
            return

        req = ChangeState.Request()
        req.transition.id = 1 if self.active_goal else 2  # 1=ACTIVATE, 2=DEACTIVATE
        future = self.controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            state_str = "activated" if self.active_goal else "deactivated"
            self.get_logger().info(f"Controller {state_str}")
        else:
            self.get_logger().warn("Failed to change controller state")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2CmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()