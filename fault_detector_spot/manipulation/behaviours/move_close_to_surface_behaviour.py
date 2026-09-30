"""Behavior-tree adapter for close-surface execution."""

import time

from py_trees.common import Status

from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import (
    ArmMovementBehaviour,
)
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)
from fault_detector_spot.manipulation.move_close_to_surface_execution import (
    CONTACT_MODE_PLANNING_DISTANCE_M,
    MAX_CARTESIAN_CONTACT_MOVES,
    MAX_CARTESIAN_STANDOFF_MOVES,
    MoveCloseToSurfaceConfig,
    MoveCloseToSurfaceExecution,
    MoveCloseToSurfaceOutcome,
)


class MoveCloseToSurfaceBehaviour(ArmMovementBehaviour):
    """Adapt one close-surface execution to the behavior tree."""

    def __init__(
        self,
        name: str = "MoveCloseToSurfaceBehaviour",
        robot_name=None,
        robot_command_resources=None,
        surface_source=None,
        config=None,
        monotonic_clock=time.monotonic,
    ):
        super().__init__(
            name,
            robot_name=str(robot_name or ""),
            robot_command_resources=robot_command_resources,
        )
        self._configured_robot_name = robot_name
        self.surface_source = surface_source
        self.config = config
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")
        self._clock = monotonic_clock
        self._execution = None
        self._command = None
        self._initialise_error = ""
        if self.config is not None:
            self.config.validate()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        if self._configured_robot_name is None:
            if not self.node.has_parameter("close_surface.robot_name"):
                self.node.declare_parameter("close_surface.robot_name", "")
            self.robot_name = str(
                self.node.get_parameter("close_surface.robot_name").value
            ).strip()
        self._ensure_executor()
        if self.surface_source is None:
            self.surface_source = (
                self.robot_command_resources.get_probe_surface_source(
                    self.node
                )
            )
        if self.config is None:
            self.config = MoveCloseToSurfaceConfig.from_node(self.node)
        self._ensure_execution()

    def initialise(self):
        super().initialise()
        self._command = None
        self._initialise_error = ""
        try:
            command = self._last_command()
            if not isinstance(command, MoveCloseToSurfaceCommand):
                raise RuntimeError(
                    "Expected MoveCloseToSurfaceCommand, got "
                    f"{type(command).__name__}"
                )
            self._command = command
            self.feedback_message = "Resolving active probe attachment"
        except Exception as exception:
            self._initialise_error = str(exception)

    def update(self) -> Status:
        try:
            if self._initialise_error:
                return self._fail_workflow(self._initialise_error)
            self._ensure_executor()
            if self.surface_source is None:
                return self._fail_workflow(
                    "Close-surface surface source is not configured"
                )
            if self.config is None:
                return self._fail_workflow(
                    "Close-surface configuration is not available"
                )
            self._ensure_execution()

            outcome = (
                self._execution.poll()
                if self._started
                else self._execution.start(self._command)
            )
            self.feedback_message = self._execution.feedback_message
            if outcome is MoveCloseToSurfaceOutcome.RUNNING:
                self._started = True
                return Status.RUNNING

            self._started = False
            if outcome is MoveCloseToSurfaceOutcome.SUCCESS:
                return Status.SUCCESS
            return self._fail_workflow(self.feedback_message)
        except Exception as exception:
            return self._fail_workflow(str(exception))

    def terminate(self, new_status: Status):
        if (
            new_status is Status.INVALID
            and self._execution is not None
            and self._execution.active
        ):
            self._execution.cancel()
        self._started = False

    def shutdown(self):
        if self._execution is not None and self._execution.active:
            self._execution.cancel()
        self._started = False
        self._initialise_error = ""

    def _ensure_execution(self) -> None:
        if self._execution is not None:
            return
        self._execution = MoveCloseToSurfaceExecution(
            self.executor,
            self.surface_source,
            self.config,
            monotonic_clock=self._clock,
            logger=self.node.get_logger() if self.node is not None else None,
        )

    def _fail_workflow(self, detail: str) -> Status:
        if self._execution is not None and self._execution.active:
            self._execution.cancel()
        self._started = False
        return super()._fail(detail)


__all__ = [
    "CONTACT_MODE_PLANNING_DISTANCE_M",
    "MAX_CARTESIAN_CONTACT_MOVES",
    "MAX_CARTESIAN_STANDOFF_MOVES",
    "MoveCloseToSurfaceBehaviour",
    "MoveCloseToSurfaceConfig",
]
