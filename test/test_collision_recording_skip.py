"""Recovered contact feedback reaches skip handling through the status pipeline."""

from dataclasses import replace

import pytest
from builtin_interfaces.msg import Time
from py_trees.common import Status

from fault_detector_spot.application.behaviour_tree.behaviours.movement_behaviour import MovementBehaviour
from fault_detector_spot.application.behaviour_tree.commands.execution_command import ExecutionCommand
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
from fault_detector_spot.application.controllers.command_controller import CommandController
from fault_detector_spot.application.recording.recorded_step import RecordedStep
from fault_detector_spot.application.ros.command_request_adapter import command_request_from_message
from fault_detector_spot.application.ros.command_transport import RosCommandTransport
from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome, ArmMovementUpdate
from fault_detector_spot.manipulation.arm_state_source import HandForceSample
from test_buffer_and_status_publisher import behavior
from test_command_ros_transport import FakeNode
from test_guarded_arm_movement_executor import (
    ManualClock, FakeArmStateSource, GoalDriver, execution, plan, pose, confirm_physical_stop,
)
from test_semantic_record_manager import RecordingStorageHarness


@pytest.mark.parametrize("command_id", [CommandID.MOVE_ARM_RELATIVE, CommandID.MOVE_ARM_TO_TAG])
@pytest.mark.parametrize("retries", [0, 2])
@pytest.mark.parametrize("recovered", [True, False])
def test_collision_feedback_retries_then_skips_only_after_recovery(tmp_path, command_id, retries, recovered):
    manager = RecordingStorageHarness(tmp_path)
    manager.repository.save("collision", [
        RecordedStep(SemanticCommand(command_id), retries, 1),
        RecordedStep(SemanticCommand(CommandID.WAIT_TIME, wait_time=1.0), 0, 1),
        RecordedStep(SemanticCommand(CommandID.WAIT_TIME, wait_time=2.0), 0, 2),
    ])
    manager.control("play", "collision")
    node = FakeNode()
    transport = RosCommandTransport(node, CommandController())

    class ContactMovement(MovementBehaviour):
        RETRY_POSE_COMMAND_ID = CommandID.MOVE_ARM_TO_POSE
        RUNNING_OUTCOME = ArmMovementOutcome.RUNNING
        SUCCESS_OUTCOME = ArmMovementOutcome.SUCCESS

        def _ensure_executor(self):
            pass

        def _start_operation(self):
            return self.result

    for attempt in range(retries + 1 if recovered else 1):
        clock, state, driver = ManualClock(), FakeArmStateSource(), GoalDriver()
        guard = execution(state, driver, clock, pose(0.0))
        guard.start(plan)
        clock.now = 0.1
        guard.observe_force_sample(HandForceSample(0.1, -20.0, 2.0, 3.0))
        driver.stop_updates.append(ArmMovementUpdate(
            ArmMovementOutcome.SUCCESS if recovered else ArmMovementOutcome.EXECUTION_ERROR,
            "Stop response",
        ))
        result = guard.poll()
        if recovered:
            result = confirm_physical_stop(guard, state, clock, 0.2)
        # Relative execution supplies its frozen target independently of stop permission.
        if command_id is CommandID.MOVE_ARM_RELATIVE:
            target = pose(0.1)
            target.header.frame_id = "odom"
            result = replace(result, retry_target=target)

        submission = manager.command_submission_pub.messages[-1]
        request = command_request_from_message(submission)
        assert transport.submit_message(submission)
        publisher = behavior(Status.FAILURE, request.command.command_id.value)
        command = ExecutionCommand(request.command.command_id, Time(), request.request_id)
        command.semantic_command = request.command
        publisher.blackboard.last_command = command
        movement = ContactMovement("contact")
        movement.blackboard = publisher.blackboard
        movement.result = result
        movement.initialise()
        assert movement.update() is Status.FAILURE
        assert transport.handle_command_status(publisher.get_structured_status_message())
        status = node.publishers["fault_detector/_internal/commands/status"].messages[-1]
        assert status.continuation_safe is recovered
        assert manager.handle_playback_status(status)
        if recovered:
            assert manager._playback_step.skip_block == (1 if attempt < retries else 2)

    if recovered:
        assert manager._playback_had_skips
        assert manager._playback_step.command.wait_time == 2.0
        assert len(manager.command_submission_pub.messages) == retries + 2
    else:
        assert not manager._playback_request_id
        assert len(manager.command_submission_pub.messages) == 1
