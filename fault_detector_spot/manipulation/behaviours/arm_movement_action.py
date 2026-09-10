"""Behavior-tree adapter for Cartesian arm movement commands."""

from py_trees.common import Access
from spot_msgs.action import RobotCommand

from fault_detector_spot.application.behaviour_tree.behaviours.move_command_action import (
    MoveCommandAction,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)
from fault_detector_spot.manipulation.commands.manipulator_move_relative_command import (
    ManipulatorMoveRelativeCommand,
)
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import (
    ManipulatorToTagCommand,
)


class ArmMovementAction(MoveCommandAction):
    """Delegate arm target resolution to the shared arm executor."""

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
        self.blackboard.register_key(
            "command_failure_request_id",
            access=Access.WRITE,
        )
        self.blackboard.register_key(
            "command_failure_detail",
            access=Access.WRITE,
        )

    def initialise(self):
        super().initialise()
        self._clear_failure_detail()

    def _init_client(self) -> bool:
        initialized = super()._init_client()
        if not initialized:
            return False

        if self.robot_command_resources is not None:
            self.arm_movement_executor = (
                self.robot_command_resources.get_arm_movement_executor(
                    self.node,
                    tag_state_source=self.tag_state_source,
                    robot_name=self.robot_name,
                )
            )
        elif self.arm_movement_executor is None:
            self.arm_movement_executor = ArmMovementExecutor(
                self.tf_listener,
                tag_state_source=self.tag_state_source,
                robot_name=self.robot_name,
            )
        return True

    def _build_goal(self) -> RobotCommand.Goal:
        command = self.blackboard.last_command
        executor = self.arm_movement_executor
        if executor is None:
            raise RuntimeError("Arm movement executor is unavailable")

        if isinstance(command, ManipulatorMoveRelativeCommand):
            return executor.relative(command)

        if isinstance(command, ManipulatorToTagCommand):
            return executor.tag_probe(command)

        raise RuntimeError(
            "Expected manipulator relative or tag movement command, got "
            f"{type(command).__name__}"
        )

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
