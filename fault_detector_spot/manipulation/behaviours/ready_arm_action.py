import math

from bosdyn.client.frame_helpers import (
    GRAV_ALIGNED_BODY_FRAME_NAME,
    HAND_FRAME_NAME,
)
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from py_trees.common import Status
from spot_msgs.action import RobotCommand
from synchros2.utilities import namespace_with

from fault_detector_spot.application.behaviour_tree.behaviours.spot_action import (
    RobotCommandActionBehaviour,
)
from fault_detector_spot.manipulation.arm_state_source import ArmStowState


READY_LIFT_DISTANCE_PARAMETER = "arm.ready_lift_distance_m"
READY_DURATION_PARAMETER = "arm.ready_duration_sec"
READY_DEPLOYED_TIMEOUT_PARAMETER = "arm.ready_deployed_timeout_sec"
DEFAULT_READY_LIFT_DISTANCE_M = 0.10
DEFAULT_READY_DURATION_SEC = 2.0
DEFAULT_READY_DEPLOYED_TIMEOUT_SEC = 2.0


class ReadyArmActionSimple(RobotCommandActionBehaviour):
    """Deploy the arm only slightly above its measured stowed hand pose."""

    def __init__(
        self,
        name="ReadyArmAction",
        robot_name="",
        robot_command_resources=None,
    ):
        super().__init__(name, robot_name, robot_command_resources)
        self.tf_listener = None
        self.arm_state_source = None
        self._movement_completed = False
        self._verification_started = None

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
            READY_DEPLOYED_TIMEOUT_PARAMETER,
            DEFAULT_READY_DEPLOYED_TIMEOUT_SEC,
        )

    def initialise(self):
        self._movement_completed = False
        self._verification_started = None
        super().initialise()

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
        self._movement_completed = False
        self._verification_started = None

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
            if state is None:
                return self._fail(
                    "Ready arm requires fresh manipulator stow state"
                )
            if state is ArmStowState.UNKNOWN:
                return self._fail(
                    "Spot reports an unknown manipulator stow state"
                )
            if state is ArmStowState.DEPLOYED:
                self.feedback_message = "Arm is already deployed"
                return Status.SUCCESS
        return super()._phase_send_goal()

    def _build_goal(self) -> RobotCommand.Goal:
        transform = self.tf_listener.lookup_a_tform_b(
            GRAV_ALIGNED_BODY_FRAME_NAME,
            HAND_FRAME_NAME,
            timeout_sec=0.0,
        )
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

    def _declare_parameter(self, name: str, default: float) -> None:
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default)

    def _positive_parameter(self, name: str) -> float:
        value = float(self.node.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"Parameter '{name}' must be positive and finite")
        return value
