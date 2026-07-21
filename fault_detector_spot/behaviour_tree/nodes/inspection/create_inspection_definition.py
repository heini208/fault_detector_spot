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
        inspection = self._inspection(command)
        object_definition = inspection.object
        if object_definition.reference_tag_id < 0:
            raise ValueError("Command reference_tag_id must be set")
        definition = InspectionObject(
            object_id=object_definition.object_id,
            display_name=object_definition.display_name,
            reference_tag=ReferenceTag(
                tag_id=object_definition.reference_tag_id,
                tag_family=object_definition.reference_tag_family,
            ),
        )
        self.object_repository.create(definition)
        return f"Created inspection object {definition.object_id}"

    def _create_routine(self, command: GenericCommand) -> str:
        inspection = self._inspection(command)
        object_definition = inspection.object
        routine_definition = inspection.routine
        routine = InspectionRoutine(
            routine_id=routine_definition.routine_id,
            display_name=routine_definition.display_name,
            sensor_id=routine_definition.sensor_id,
            probe_frame=routine_definition.probe_frame,
        )
        self.object_repository.add_routine(
            object_definition.object_id,
            routine,
        )
        return (
            "Created inspection routine "
            f"{object_definition.object_id}/{routine.routine_id}"
        )

    @staticmethod
    def _inspection(command: GenericCommand):
        if command.inspection is None:
            raise ValueError("Command inspection payload must be set")
        return command.inspection
