from fault_detector_spot.application.behaviour_tree.behaviours.move_command_action import (
    MoveCommandAction,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import (
    ManipulatorToTagCommand,
)
from spot_msgs.action import RobotCommand


class ManipulatorMoveArmAction(MoveCommandAction):
    """Execute the Cartesian hand target produced by a tag movement command."""

    def __init__(
        self,
        name="ManipulatorMoveArmAction",
        robot_name="",
        duration=3.0,
        robot_command_resources=None,
    ):
        super().__init__(name, robot_name, robot_command_resources)
        self.duration = duration
        self.arm_movement_executor = None

    def _init_client(self) -> bool:
        initialized = super()._init_client()
        if not initialized:
            return False
        if self.robot_command_resources is not None:
            self.arm_movement_executor = (
                self.robot_command_resources.get_arm_movement_executor(
                    self.node,
                    self.robot_name,
                )
            )
        elif self.arm_movement_executor is None:
            self.arm_movement_executor = ArmMovementExecutor(
                self.tf_listener,
                robot_name=self.robot_name,
            )
        return True

    def _build_goal(self) -> RobotCommand.Goal:
        command = self.blackboard.last_command
        if not isinstance(command, ManipulatorToTagCommand):
            raise RuntimeError(
                "Expected ManipulatorToTagCommand, got "
                f"{type(command).__name__}"
            )
        if self.arm_movement_executor is None:
            raise RuntimeError("Arm movement executor is unavailable")

        target = command.compute_goal_pose(self.tf_listener)
        return self.arm_movement_executor.pose(
            target,
            self.duration,
        )
