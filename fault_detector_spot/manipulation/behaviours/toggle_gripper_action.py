"""Behavior-tree adapter for the shared gripper execution lifecycle."""

from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import (
    ArmMovementBehaviour,
)


class ToggleGripperAction(ArmMovementBehaviour):
    def __init__(
        self,
        name="ToggleGripperAction",
        robot_name="",
        robot_command_resources=None,
    ):
        super().__init__(
            name,
            robot_name=robot_name,
            robot_command_resources=robot_command_resources,
        )

    def _start_operation(self):
        return self.executor.toggle_gripper()
