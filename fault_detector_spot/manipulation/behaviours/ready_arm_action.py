import math
import time

from bosdyn.client.frame_helpers import (
    GRAV_ALIGNED_BODY_FRAME_NAME,
    HAND_FRAME_NAME,
)
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_spot_api_msgs.conversions import convert
from py_trees.common import Access, Status
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with

from fault_detector_spot.application.behaviour_tree.behaviours.spot_action import (
    RobotCommandActionBehaviour,
)
from fault_detector_spot.manipulation.arm_state_source import ArmStowState


READY_LIFT_DISTANCE_PARAMETER = "arm.ready_lift_distance_m"
READY_DURATION_PARAMETER = "arm.ready_duration_sec"
READY_STATE_TIMEOUT_PARAMETER = "arm.ready_state_timeout_sec"
READY_TF_TIMEOUT_PARAMETER = "arm.ready_tf_timeout_sec"
READY_DEPLOYED_TIMEOUT_PARAMETER = "arm.ready_deployed_timeout_sec"
DEFAULT_READY_LIFT_DISTANCE_M = 0.10
DEFAULT_READY_DURATION_SEC = 2.0
DEFAULT_READY_STATE_TIMEOUT_SEC = 2.0
DEFAULT_READY_TF_TIMEOUT_SEC = 2.0
DEFAULT_READY_DEPLOYED_TIMEOUT_SEC = 2.0


class ReadyArmActionSimple(RobotCommandActionBehaviour):
    """Deploy the arm only slightly above its measured stowed hand pose."""

    def __init__(
        self,
        name="ReadyArmAction",
        robot_name="",
        robot_command_resources=None,
        monotonic_clock=time.monotonic,
    ):
        super().__init__(
            name,
            robot_name,
            robot_command_resources,
            monotonic_clock=monotonic_clock,
        )
        self.tf_listener = None
        self.arm_state_source = None
        self._state_wait_started = None
        self._tf_wait_started = None
        self._hand_transform = None
        self._movement_completed = False
        self._verification_started = None
        self.blackboard.register_key(
            "command_failure_request_id",
            access=Access.WRITE,
        )
        self.blackboard.register_key(
            "command_failure_detail",
            access=Access.WRITE,
        )

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._declare_parameter(
            READY_LIFT_DISTANCE_PARAMETER,
            DEFAULT_READY_LIFT_DISTANCE_M,
        )
        self._declare_parameter(
            READY_DURATION_PARAMETER,
            DEFAULT_READY_DURATION_SEC,
        )
        self._declare_parameter(
            READY_STATE_TIMEOUT_PARAMETER,
            DEFAULT_READY_STATE_TIMEOUT_SEC,
        )
        self._declare_parameter(
            READY_TF_TIMEOUT_PARAMETER,
            DEFAULT_READY_TF_TIMEOUT_SEC,
        )
        self._declare_parameter(
            READY_DEPLOYED_TIMEOUT_PARAMETER,
            DEFAULT_READY_DEPLOYED_TIMEOUT_SEC,
        )

    def initialise(self):
        super().initialise()
        self._clear_failure_detail()

    def update(self) -> Status:
        if self._movement_completed:
            return self._verify_deployed()

        status = super().update()
        if status is not Status.SUCCESS:
            return status

        self._movement_completed = True
        self._verification_started = self._monotonic_clock()
        return self._verify_deployed()

    def terminate(self, new_status: Status):
        super().terminate(new_status)

    def _init_client(self) -> bool:
        if self.robot_command_resources is None:
            raise RuntimeError(
                "ReadyArmAction requires shared robot command resources"
            )
        self.tf_listener = self.robot_command_resources.get_tf_listener(
            self.node
        )
        self.arm_state_source = (
            self.robot_command_resources.get_arm_state_source(self.node)
        )
        return super()._init_client()

    def _phase_send_goal(self):
        if self.send_goal_future is None:
            state = self.arm_state_source.stow_state()
            if state is None or state is ArmStowState.UNKNOWN:
                return self._wait_for_stow_state()
            self._state_wait_started = None
            if state is ArmStowState.DEPLOYED:
                self.feedback_message = "Arm is already deployed"
                return Status.SUCCESS
            if self._hand_transform is None:
                try:
                    self._hand_transform = (
                        self.tf_listener.lookup_a_tform_b(
                            GRAV_ALIGNED_BODY_FRAME_NAME,
                            HAND_FRAME_NAME,
                            timeout_sec=0.0,
                        )
                    )
                except Exception as exception:
                    return self._wait_for_hand_transform(exception)
                self._tf_wait_started = None
        return super()._phase_send_goal()

    def _wait_for_stow_state(self) -> Status:
        now = self._monotonic_clock()
        if self._state_wait_started is None:
            self._state_wait_started = now
        timeout_sec = self._positive_parameter(
            READY_STATE_TIMEOUT_PARAMETER
        )
        if now - self._state_wait_started >= timeout_sec:
            return self._fail(
                "Fresh manipulator stow state was unavailable for "
                f"{timeout_sec:.1f} s"
            )
        self.feedback_message = "Waiting for manipulator stow state"
        return Status.RUNNING

    def _wait_for_hand_transform(self, exception: Exception) -> Status:
        now = self._monotonic_clock()
        if self._tf_wait_started is None:
            self._tf_wait_started = now
        timeout_sec = self._positive_parameter(READY_TF_TIMEOUT_PARAMETER)
        if now - self._tf_wait_started >= timeout_sec:
            return self._fail(
                "Ready arm hand transform "
                f"{GRAV_ALIGNED_BODY_FRAME_NAME} -> {HAND_FRAME_NAME} "
                f"was unavailable for {timeout_sec:.1f} s: {exception}"
            )
        self.feedback_message = (
            "Waiting for ready-arm hand pose transform "
            f"{GRAV_ALIGNED_BODY_FRAME_NAME} -> {HAND_FRAME_NAME}"
        )
        return Status.RUNNING

    def _build_goal(self) -> RobotCommand.Goal:
        transform = self._hand_transform
        if transform is None:
            raise RuntimeError("Ready arm hand pose transform is unavailable")
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        lift_distance_m = self._positive_parameter(
            READY_LIFT_DISTANCE_PARAMETER
        )
        duration_sec = self._positive_parameter(READY_DURATION_PARAMETER)

        ready_cmd = RobotCommandBuilder.arm_pose_command(
            translation.x,
            translation.y,
            translation.z + lift_distance_m,
            rotation.w,
            rotation.x,
            rotation.y,
            rotation.z,
            namespace_with(
                self.robot_name,
                GRAV_ALIGNED_BODY_FRAME_NAME,
            ),
            duration_sec,
        )
        goal = RobotCommand.Goal()
        convert(ready_cmd, goal.command)
        return goal

    def _verify_deployed(self) -> Status:
        state = self.arm_state_source.stow_state()
        if state is ArmStowState.DEPLOYED:
            self.feedback_message = "Arm deployed"
            return Status.SUCCESS

        timeout_sec = self._positive_parameter(
            READY_DEPLOYED_TIMEOUT_PARAMETER
        )
        if (
            self._verification_started is not None
            and self._monotonic_clock() - self._verification_started
            >= timeout_sec
        ):
            return self._fail(
                "Ready arm movement completed, but Spot did not report "
                f"DEPLOYED within {timeout_sec:.1f} s"
            )

        self.feedback_message = "Waiting for Spot to report arm deployed"
        return Status.RUNNING

    def _reset_subclass_state(self) -> None:
        self._state_wait_started = None
        self._tf_wait_started = None
        self._hand_transform = None
        self._movement_completed = False
        self._verification_started = None

    def _clear_failure_detail(self) -> None:
        request_id = self._current_request_id()
        self.blackboard.command_failure_request_id = request_id
        self.blackboard.command_failure_detail = ""

    def _on_failure(self, detail: str) -> None:
        self.blackboard.command_failure_request_id = self._current_request_id()
        self.blackboard.command_failure_detail = detail

    def _current_request_id(self) -> str:
        try:
            command = self.blackboard.last_command
        except (AttributeError, KeyError):
            return ""
        return str(getattr(command, "request_id", "") or "")

    def _declare_parameter(self, name: str, default: float) -> None:
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default)

    def _positive_parameter(self, name: str) -> float:
        value = float(self.node.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"Parameter '{name}' must be positive and finite")
        return value
