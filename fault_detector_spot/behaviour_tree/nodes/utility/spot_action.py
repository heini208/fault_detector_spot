import py_trees
from py_trees.common import Status, Access
from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import namespace_with
import synchros2.scope as ros_scope
from fault_detector_spot.behaviour_tree.command_ids import CommandID

class ActionClientBehaviour(py_trees.behaviour.Behaviour):
    """
    Minimal base for any ROS action client behaviour.
    Handles emergency cancel, lazy initialization, goal send, cancel, and result polling.
    Subclasses must implement:
      - _init_client(): bool
      - _send_goal(goal)
      - _build_goal(): Goal
    The default _cancel_goal cancels via _client and resets state.
    """
    def __init__(self, name: str):
        super().__init__(name)
        self.initialized = False
        self.send_goal_future = None
        self.goal_handle = None
        self.get_result_future = None
        self._client = None  # optional, set by subclasses
        # Blackboard client for last_command
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="last_command", access=Access.READ)

    def setup(self, **kwargs):
        # Acquire ROS node
        self.node = kwargs.get("node") or ros_scope.node()
        if not self.node:
            raise RuntimeError(f"{self.__class__.__name__} requires a ROS node")

    def update(self) -> Status:
        # Execute each lifecycle phase in sequence
        for phase in (
            self._phase_initialize,
            self._phase_send_goal,
            self._phase_wait_for_acceptance,
            self._phase_wait_for_result
        ):
            result = phase()
            if result is not None:
                return result
        return Status.RUNNING

    def _phase_initialize(self) -> Status | None:
        if not self.initialized:
            ok = self._init_client()
            if not ok:
                return Status.FAILURE
            self.feedback_message = "Client initialized"
        return None

    def _phase_send_goal(self) -> Status | None:
        if self.send_goal_future is None:
            maybe_goal = self._build_goal()
            if isinstance(maybe_goal, Status) or maybe_goal is None:
                return maybe_goal or Status.FAILURE
            self.logger.info(f"[{self.name}] Sending goal")
            self.send_goal_future = self._send_goal(maybe_goal)
            self.feedback_message = "Goal sent"
            return Status.RUNNING
        return None

    def _phase_wait_for_acceptance(self) -> Status | None:
        if self.goal_handle is None and self.send_goal_future:
            if not self.send_goal_future.done():
                return Status.RUNNING
            self.goal_handle = self.send_goal_future.result()
            if not self.goal_handle.accepted:
                self.feedback_message = "Goal rejected"
                self._cancel_goal(self.goal_handle)
                self._reset_state()
                return Status.FAILURE
            self.get_result_future = self.goal_handle.get_result_async()
            self.feedback_message = "Goal accepted"
            return Status.RUNNING
        return None

    def _phase_wait_for_result(self) -> Status | None:
        if self.get_result_future:
            if not self.get_result_future.done():
                return Status.RUNNING
            result = self.get_result_future.result().result
            success = getattr(result, "success", False)
            self.feedback_message = "Succeeded" if success else f"Failed: {result}"
            status = Status.SUCCESS if success else Status.FAILURE
            self._cancel_goal(self.goal_handle)
            self._reset_state()
            return status
        return None

    def terminate(self, new_status: Status):
        if new_status == Status.INVALID and self.goal_handle:
            self._cancel_goal(self.goal_handle)
        self._reset_state()

    def _cancel_goal(self, handle):
        """
        Default cancellation: cancel via client and clear state.
        Subclasses may override if needed.
        """
        if hasattr(self, '_client') and self._client and handle:
            self._client._cancel_goal_async(handle)
        self._reset_state()

    def _reset_state(self):
        self.send_goal_future = None
        self.goal_handle = None
        self.get_result_future = None
        self.initialized = False

    def _send_goal(self, goal):
        return self._client.send_goal_async(goal)

    # abstract methods for subclasses
    def _init_client(self) -> bool:
        raise NotImplementedError

    def _build_goal(self):
        raise NotImplementedError

class SimpleSpotAction(ActionClientBehaviour):
    """
    Base for any RobotCommand action on Spot.
    Implements _init_client and _send_goal using ActionClientWrapper.
    """
    def __init__(self, name: str, robot_name: str = ""):
        super().__init__(name)
        self.robot_name = robot_name

    def _init_client(self) -> bool:
        try:
            action_ns = namespace_with(self.robot_name, "robot_command")
            self._client = ActionClientWrapper(RobotCommand, action_ns, self.node, wait_for_server=False)
            if not self._client.wait_for_server(timeout_sec=0.0):
                self.feedback_message = f"Action server '{action_ns}' unavailable"
                return False
            self.initialized = True
            return True
        except Exception as e:
            self.logger.error(f"[{self.name}] init failed: {e}")
            return False

    # subclasses implement _build_goal() only
