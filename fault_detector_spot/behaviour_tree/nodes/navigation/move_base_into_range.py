#!/usr/bin/env python3
"""
MoveBaseIntoRange behaviour — iterative send/wait/re-evaluate loop.

This class overrides update() so it will keep sending short-step RobotCommand
goals until the tag appears in reachable_tags, or until max_retries is exceeded.
"""
from __future__ import annotations

import math
from typing import Optional

from bosdyn.client.frame_helpers import BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.math_helpers import SE3Pose, SE2Pose, Quat
from bosdyn.client.robot_command import RobotCommandBuilder

from bosdyn_msgs.conversions import convert
from fault_detector_spot.behaviour_tree.nodes.utility.spot_action import SimpleSpotAction
from py_trees.common import Status, Access
from rclpy.node import Node
from spot_msgs.action import RobotCommand  # type: ignore
from synchros2.scope import node as scoped_node
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with


class MoveBaseIntoRange(SimpleSpotAction):
    """
    Behaviour that moves Spot's base so that an AprilTag becomes within arm reach.
    Iteratively sends small SE2 goals, waits for each to finish, re-checks reachability,
    and repeats until success or until max_retries is exceeded.

    Blackboard keys:
      - visible_tags (READ)
      - reachable_tags (READ)
      - last_command.tag_id  (READ)
    """

    def __init__(
            self,
            name: str = "MoveBaseIntoRange",
            robot_name: str = "",
            arm_reach: float = 0.984,
            reach_tolerance: float = 0.15,
            step_size: float = 0.25,
            max_turn: float = 0.5,
            align_threshold: float = 0.3,
            max_retries: int = 20,
    ) -> None:
        super().__init__(name, robot_name=robot_name)
        self.robot_name = robot_name

        # reach parameters
        self.arm_reach = arm_reach
        self.reach_tolerance = reach_tolerance
        self.maximum_reach = max(0.0, self.arm_reach - self.reach_tolerance)

        # motion tuning
        self.step_size = float(step_size)
        self.max_turn = float(max_turn)
        self.align_threshold = float(align_threshold)
        self.max_retries = int(max_retries)

        # namespaced frames
        self._body_frame_name = namespace_with(self.robot_name, BODY_FRAME_NAME)
        self._odom_frame_name = namespace_with(self.robot_name, ODOM_FRAME_NAME)

        # internals
        self.tf_listener: Optional[TFListenerWrapper] = None
        self.attempts = 0

        # blackboard keys
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="visible_tags", access=Access.READ)
        self.blackboard.register_key(key="reachable_tags", access=Access.READ)
        self.blackboard.register_key(key="last_command", access=Access.READ)

    def setup(self, **kwargs) -> None:
        self.node: Node = kwargs.get("node") or scoped_node()
        if not self.node:
            raise RuntimeError(f"{self.name}: no ROS node provided")

    def _init_client(self) -> bool:
        """
        Initialize action client and TF listener. Called by update() once when needed.
        """
        try:
            ok = super()._init_client()
            if not ok:
                self.feedback_message = "Action client unavailable"
                return False

            self.tf_listener = TFListenerWrapper(self.node)
            try:
                self.tf_listener.wait_for_a_tform_b(self._odom_frame_name, self._body_frame_name, timeout_sec=2.0)
            except Exception:
                self.logger.debug(f"{self.name}: odom<->body TF not yet available")

            # reset attempts on (re)init
            self.attempts = 0
            return True
        except Exception as e:
            self.logger.error(f"[{self.name}] init failed: {e}")
            return False

    # keep the old _build_goal implementation (unchanged in logic)
    def _build_goal(self):
        """
        Build an action goal for a single small step (rotation-only or small forward+heading)
        or return Status/None as described earlier.
        """
        # Validate last_command.tag_id presence
        if not self.blackboard.exists("last_command") or getattr(self.blackboard.last_command, "tag_id", None) is None:
            self.feedback_message = "No target tag_id in last_command"
            return Status.FAILURE

        tag_id = int(self.blackboard.last_command.tag_id)

        # Validate visible tags
        if not self.blackboard.exists("visible_tags") or not self.blackboard.visible_tags:
            self.feedback_message = "No tags visible"
            return Status.FAILURE

        visible = self.blackboard.visible_tags
        if tag_id not in visible:
            self.feedback_message = f"Tag {tag_id} not visible"
            return Status.FAILURE

        # Success if reachable
        if self.blackboard.exists("reachable_tags") and self.blackboard.reachable_tags:
            if tag_id in self.blackboard.reachable_tags:
                self.feedback_message = f"Tag {tag_id} already in arm reach"
                return Status.SUCCESS

        # compute body-frame pose of tag
        tag_element = visible[tag_id]
        p = tag_element.pose.pose.position
        x = float(p.x)
        y = float(p.y)
        d_xy = math.hypot(x, y)

        if d_xy <= self.maximum_reach:
            self.feedback_message = f"Tag {tag_id} within reach (d={d_xy:.3f} <= {self.maximum_reach:.3f})"
            return Status.SUCCESS

        forward_needed = d_xy - self.maximum_reach
        bearing = math.atan2(y, x)

        forward = max(0.0, min(self.step_size, forward_needed))
        turn = max(-self.max_turn, min(self.max_turn, bearing))

        # If residual move is tiny, wait for perception instead of issuing tiny goal
        if forward < 0.02:
            self.feedback_message = f"Small residual {forward:.3f}m — waiting for perception"
            return Status.RUNNING

        body_tform_goal = SE2Pose(x=forward, y=0.0, angle=turn)

        # need TF to convert to odom goal
        try:
            odom_t_robot = self.tf_listener.lookup_a_tform_b(self._odom_frame_name, self._body_frame_name,
                                                             timeout_sec=2.0)
        except Exception as e:
            self.feedback_message = f"Waiting for TF odom->body: {e}"
            self.logger.debug(f"[{self.name}] TF not available: {e}")
            return Status.RUNNING

        odom_t_robot_se3 = SE3Pose(
            odom_t_robot.transform.translation.x,
            odom_t_robot.transform.translation.y,
            odom_t_robot.transform.translation.z,
            Quat(
                odom_t_robot.transform.rotation.w,
                odom_t_robot.transform.rotation.x,
                odom_t_robot.transform.rotation.y,
                odom_t_robot.transform.rotation.z,
            ),
        )
        odom_t_robot_se2 = odom_t_robot_se3.get_closest_se2_transform()
        odom_t_goal_se2 = odom_t_robot_se2 * body_tform_goal

        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=odom_t_goal_se2.x,
            goal_y=odom_t_goal_se2.y,
            goal_heading=odom_t_goal_se2.angle,
            frame_name=self._odom_frame_name,
        )

        action_goal = RobotCommand.Goal()
        convert(proto_goal, action_goal.command)

        self.feedback_message = (
            f"Prepared step: d={d_xy:.2f}m need={forward_needed:.2f} step={forward:.2f} turn={turn:.2f}"
        )
        return action_goal

    def _clear_goal_state(self):
        """
        Clear only goal state (so we keep the client initialized). This differs
        from _reset_state in the base class which also clears 'initialized'.
        """
        self.send_goal_future = None
        self.goal_handle = None
        self.get_result_future = None

    def update(self) -> Status:
        """
        Custom lifecycle that:
          - ensures client and TF are initialized,
          - if tag already reachable -> SUCCESS,
          - otherwise send one goal, wait for acceptance/result,
          - after each result, re-evaluate reachable_tags and loop until success/failure.
        """
        # ensure action client initialized once
        if not self.initialized:
            ok = self._init_client()
            if not ok:
                return Status.FAILURE
            self.initialized = True

        # Validate last_command.tag_id presence early (avoid sending goals without a target)
        if not self.blackboard.exists("last_command") or getattr(self.blackboard.last_command, "tag_id", None) is None:
            self.feedback_message = "No target tag_id in last_command"
            return Status.FAILURE

        tag_id = int(self.blackboard.last_command.tag_id)

        # sanity & visibility checks
        if not self.blackboard.exists("visible_tags") or not self.blackboard.visible_tags:
            self.feedback_message = "No tags visible"
            return Status.FAILURE
        visible = self.blackboard.visible_tags
        if tag_id not in visible:
            self.feedback_message = f"Tag {tag_id} not visible"
            return Status.FAILURE

        # If tag already reachable -> succeed and cancel any running goal
        if self.blackboard.exists("reachable_tags") and self.blackboard.reachable_tags and (
                tag_id in self.blackboard.reachable_tags):
            # if there's a running goal, cancel it
            if self.goal_handle:
                try:
                    self._cancel_goal(self.goal_handle)
                except Exception:
                    self.logger.debug(f"[{self.name}] cancel on success failed")
            self._clear_goal_state()
            self.attempts = 0
            self.feedback_message = f"Tag {tag_id} in reach"
            return Status.SUCCESS

        # If we have not sent a goal yet, build & send one
        if self.send_goal_future is None:
            maybe_goal = self._build_goal()
            # _build_goal may return a Status value (SUCCESS/FAILURE/RUNNING)
            if isinstance(maybe_goal, Status):
                return maybe_goal
            if maybe_goal is None:
                return Status.FAILURE
            # send goal asynchronously
            try:
                self.send_goal_future = self._send_goal(maybe_goal)
                self.feedback_message = "Goal sent"
                return Status.RUNNING
            except Exception as e:
                self.feedback_message = f"Failed to send goal: {e}"
                self.logger.error(f"[{self.name}] send failed: {e}")
                self._clear_goal_state()
                return Status.FAILURE

        # We've sent a goal; wait for acceptance if needed
        if self.goal_handle is None and self.send_goal_future:
            if not self.send_goal_future.done():
                return Status.RUNNING
            # goal accepted/rejected
            try:
                self.goal_handle = self.send_goal_future.result()
            except Exception as e:
                self.feedback_message = f"Send goal future error: {e}"
                self.logger.error(f"[{self.name}] send_goal_future.result() error: {e}")
                self._clear_goal_state()
                return Status.FAILURE

            if not getattr(self.goal_handle, "accepted", False):
                self.feedback_message = "Goal rejected"
                try:
                    self._cancel_goal(self.goal_handle)
                except Exception:
                    pass
                self._clear_goal_state()
                return Status.FAILURE

            # request result when accepted
            try:
                self.get_result_future = self.goal_handle.get_result_async()
            except Exception as e:
                self.feedback_message = f"Failed to request result: {e}"
                self.logger.error(f"[{self.name}] get_result_async error: {e}")
                self._clear_goal_state()
                return Status.FAILURE
            return Status.RUNNING

        # If waiting for result
        if self.get_result_future:
            if not self.get_result_future.done():
                return Status.RUNNING
            # evaluate result
            try:
                result = self.get_result_future.result().result
            except Exception as e:
                self.feedback_message = f"Result future error: {e}"
                self.logger.error(f"[{self.name}] get_result_future.result() error: {e}")
                self._clear_goal_state()
                return Status.FAILURE

            success = getattr(result, "success", False)
            # clear the goal state but keep client initialized for the next iteration
            self._clear_goal_state()

            if success:
                # Even if the action reported success, confirm that the tag is now reachable
                # (perception might need a small moment); next tick will decide
                self.attempts = 0
                self.feedback_message = "Last step succeeded — re-evaluating reachability"
                return Status.RUNNING
            else:
                # attempt counting to avoid infinite tries
                self.attempts += 1
                if self.attempts > self.max_retries:
                    self.feedback_message = f"Aborting: exceeded max retries ({self.max_retries})"
                    self.attempts = 0
                    return Status.FAILURE
                self.feedback_message = f"Step failed (attempt {self.attempts}/{self.max_retries}) — retrying"
                return Status.RUNNING

        # default fallthrough (safety)
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        # ensure any running goal gets cancelled when this behaviour is invalidated
        if new_status == Status.INVALID and self.goal_handle:
            try:
                self._cancel_goal(self.goal_handle)
            except Exception:
                self.logger.debug(f"[{self.name}] terminate: cancel failed")
        # clear only goal state — keep client initialisation intact for next activation
        self._clear_goal_state()
        # do not call super().terminate which would reset initialized flag