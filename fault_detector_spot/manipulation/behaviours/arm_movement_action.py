"""Behavior-tree adapter for centralized Cartesian arm execution."""

from py_trees.common import Access, Status

from fault_detector_spot.application.behaviour_tree.behaviours.move_command_action import (
    MoveCommandAction,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
    ArmMovementOutcome,
)
from fault_detector_spot.manipulation.commands.manipulator_move_relative_command import (
    ManipulatorMoveRelativeCommand,
)
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import (
    ManipulatorToTagCommand,
)


class ArmMovementAction(MoveCommandAction):
    """Translate BT ticks to the shared arm executor lifecycle."""

    def __init__(
        self,
        name: str = "ArmMovementAction",
        robot_name: str = "",
        tag_state_source=None,
        robot_command_resources=None,
    ):
        super().__init__(
            name,
            robot_name=robot_name,
            robot_command_resources=robot_command_resources,
        )
        self.tag_state_source = tag_state_source
        self.arm_movement_executor = None
        self._movement_started = False
        self.blackboard.register_key(
            "command_failure_request_id",
            access=Access.WRITE,
        )
        self.blackboard.register_key(
            "command_failure_detail",
            access=Access.WRITE,
        )

    def initialise(self):
        self._movement_started = False
        self._clear_failure_detail()

    def update(self) -> Status:
        try:
            self._ensure_executor()

            if not self._movement_started:
                command = self._get_last_command()
                if command is None:
                    return self._fail_arm("No command on blackboard")

                readiness = self._prepare_move_command(command)
                if readiness is Status.RUNNING:
                    return Status.RUNNING
                if readiness is Status.FAILURE:
                    return self._fail_arm(
                        self.feedback_message
                        or "Arm movement preparation failed"
                    )

                movement = self._start_command(command)
                if movement.outcome is ArmMovementOutcome.RUNNING:
                    self._movement_started = True
            else:
                movement = self.arm_movement_executor.poll()

            self.feedback_message = movement.detail
            if movement.outcome is ArmMovementOutcome.RUNNING:
                return Status.RUNNING

            self._movement_started = False
            if movement.outcome is ArmMovementOutcome.SUCCESS:
                return Status.SUCCESS
            return self._fail_arm(movement.detail)
        except Exception as exception:
            return self._fail_arm(
                f"Arm movement failed: {exception}"
            )

    def terminate(self, new_status: Status):
        if new_status is Status.INVALID and self._movement_started:
            self.arm_movement_executor.cancel()
        self._movement_started = False

    def shutdown(self):
        if (
            self._movement_started
            and self.arm_movement_executor is not None
        ):
            self.arm_movement_executor.cancel()
        self._movement_started = False

    def _ensure_executor(self) -> None:
        if self.arm_movement_executor is not None:
            if self.tf_listener is None:
                self.tf_listener = (
                    self.arm_movement_executor.tf_listener
                )
            return

        if self.robot_command_resources is None:
            raise RuntimeError(
                "ArmMovementAction requires shared robot command resources"
            )

        self.tf_listener = (
            self.robot_command_resources.get_tf_listener(self.node)
        )
        self.arm_movement_executor = (
            self.robot_command_resources.get_arm_movement_executor(
                self.node,
                tag_state_source=self.tag_state_source,
                robot_name=self.robot_name,
            )
        )

    def _start_command(self, command):
        executor = self.arm_movement_executor
        if isinstance(command, ManipulatorMoveRelativeCommand):
            return executor.relative(command)
        if isinstance(command, ManipulatorToTagCommand):
            return executor.tag_probe(command)
        raise RuntimeError(
            "Expected manipulator relative or tag movement command, got "
            f"{type(command).__name__}"
        )

    def _fail_arm(self, detail: str) -> Status:
        if (
            self._movement_started
            and self.arm_movement_executor is not None
        ):
            self.arm_movement_executor.cancel()
        self._movement_started = False

        normalized = str(detail).strip() or "Arm movement failed"
        self._on_failure(normalized)
        self.feedback_message = normalized
        return Status.FAILURE

    def _clear_failure_detail(self) -> None:
        request_id = self._current_request_id()
        self.blackboard.command_failure_request_id = request_id
        self.blackboard.command_failure_detail = ""

    def _on_failure(self, detail: str) -> None:
        self.blackboard.command_failure_request_id = (
            self._current_request_id()
        )
        self.blackboard.command_failure_detail = str(detail).strip()

    def _current_request_id(self) -> str:
        try:
            command = self.blackboard.last_command
        except (AttributeError, KeyError):
            return ""
        return str(getattr(command, "request_id", "") or "")
