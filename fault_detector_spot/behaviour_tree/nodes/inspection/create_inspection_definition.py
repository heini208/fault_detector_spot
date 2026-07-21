"""Create inspection objects and routines from complex commands."""

from pathlib import Path
from typing import Any, Optional, Union

import py_trees
from rclpy.node import Node

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.commands import (
    generic_complex_command,
)
from fault_detector_spot.inspection.models import (
    InspectionObject,
    InspectionRoutine,
    ReferenceTag,
)
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)


_CREATION_COMMANDS = {
    CommandID.CREATE_INSPECTION_OBJECT,
    CommandID.CREATE_INSPECTION_ROUTINE,
}
GenericCommand = generic_complex_command.GenericCommand


class CreateInspectionDefinition(py_trees.behaviour.Behaviour):
    """Create one explicitly selected inspection definition."""

    def __init__(
        self,
        command_id: CommandID,
        object_root: Optional[Union[str, Path]] = None,
        name: str = "CreateInspectionDefinition",
    ):
        """Select the creation command handled by this tree leaf."""
        super().__init__(name)
        if command_id not in _CREATION_COMMANDS:
            raise ValueError(
                f"Unsupported inspection creation command: {command_id}"
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
        """Execute one non-overwriting definition creation."""
        try:
            command = self._command()
            if self.command_id == CommandID.CREATE_INSPECTION_OBJECT:
                message = self._create_object(command)
            else:
                message = self._create_routine(command)
        except Exception as exception:
            self.feedback_message = (
                f"Inspection definition creation failed: {exception}"
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
                "Inspection definition creation requires a GenericCommand"
            )
        if command.command_id != self.command_id:
            raise ValueError(
                "Command does not match the inspection creation behavior"
            )
        return command

    def _create_object(self, command: GenericCommand) -> str:
        if command.reference_tag_id is None:
            raise ValueError("Command reference_tag_id must be set")
        definition = InspectionObject(
            object_id=command.object_id or "",
            display_name=command.display_name or "",
            reference_tag=ReferenceTag(
                tag_id=command.reference_tag_id,
                tag_family=command.reference_tag_family or "",
            ),
        )
        self.object_repository.create(definition)
        return f"Created inspection object {definition.object_id}"

    def _create_routine(self, command: GenericCommand) -> str:
        routine = InspectionRoutine(
            routine_id=command.routine_id or "",
            display_name=command.display_name or "",
            sensor_id=command.sensor_id or "",
            probe_frame=command.probe_frame or "",
        )
        self.object_repository.add_routine(
            command.object_id or "",
            routine,
        )
        return (
            "Created inspection routine "
            f"{command.object_id}/{routine.routine_id}"
        )
