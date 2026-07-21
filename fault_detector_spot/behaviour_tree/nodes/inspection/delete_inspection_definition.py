"""Delete inspection objects and routines from complex commands."""

from pathlib import Path
from typing import Any, Optional, Union

import py_trees
from rclpy.node import Node

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.commands.generic_complex_command import (
    GenericCommand,
)
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)


_DELETION_COMMANDS = {
    CommandID.DELETE_INSPECTION_OBJECT,
    CommandID.DELETE_INSPECTION_ROUTINE,
}


class DeleteInspectionDefinition(py_trees.behaviour.Behaviour):
    """Delete one explicitly selected inspection definition."""

    def __init__(
        self,
        command_id: CommandID,
        object_root: Optional[Union[str, Path]] = None,
        name: str = "DeleteInspectionDefinition",
    ):
        """Select the deletion command handled by this tree leaf."""
        super().__init__(name)
        if command_id not in _DELETION_COMMANDS:
            raise ValueError(
                f"Unsupported inspection deletion command: {command_id}"
            )
        self.command_id = command_id
        self.object_root = object_root
        self.node: Optional[Node] = None
        self.object_repository: Optional[ObjectRepository] = None
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs: Any) -> None:
        """Create object storage access and attach the command input."""
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError(f"{self.name}: no ROS node provided")
        self.blackboard.register_key(
            "last_command",
            access=py_trees.common.Access.READ,
        )
        self.object_repository = ObjectRepository(self.object_root)

    def update(self) -> py_trees.common.Status:
        """Execute one deletion command."""
        try:
            command = self._command()
            inspection = command.inspection
            object_id = inspection.object.object_id
            if not object_id:
                raise ValueError("Command object_id must not be empty")

            if self.command_id == CommandID.DELETE_INSPECTION_OBJECT:
                if not self.object_repository.delete_object(object_id):
                    raise FileNotFoundError(
                        f"Inspection object does not exist: {object_id}"
                    )
                message = f"Deleted inspection object {object_id}"
            else:
                routine_id = inspection.routine.routine_id
                if not routine_id:
                    raise ValueError(
                        "Command routine_id must not be empty"
                    )
                self.object_repository.delete_routine(
                    object_id,
                    routine_id,
                )
                message = (
                    "Deleted inspection routine "
                    f"{object_id}/{routine_id}"
                )
        except Exception as exception:
            self.feedback_message = (
                f"Inspection definition deletion failed: {exception}"
            )
            if self.node is not None:
                self.node.get_logger().error(self.feedback_message)
            return py_trees.common.Status.FAILURE

        self.feedback_message = message
        self.node.get_logger().info(message)
        return py_trees.common.Status.SUCCESS

    def _command(self) -> GenericCommand:
        if (
            not self.blackboard.exists("last_command")
            or self.blackboard.last_command is None
        ):
            raise RuntimeError("No command is available")

        command = self.blackboard.last_command
        if not isinstance(command, GenericCommand):
            raise TypeError(
                "Inspection definition deletion requires a GenericCommand"
            )
        if command.command_id != self.command_id:
            raise ValueError(
                "Command does not match the inspection deletion behavior"
            )
        if command.inspection is None:
            raise ValueError("Command inspection payload must be set")
        return command
