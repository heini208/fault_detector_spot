"""Offline contracts for the combined saved-point measurement command."""
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock
from threading import RLock

import pytest
from py_trees.common import Status
from fault_detector_msgs.msg import OperationalIntent, TagElement
from fault_detector_msgs.srv import ProbePointExecutionStep
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import SemanticCommand, InspectionSelection
from fault_detector_spot.application.ros.operational_intent_adapter import operational_intent_to_command
from fault_detector_spot.application.ros.semantic_command_adapter import semantic_command_from_message, semantic_command_to_message
from fault_detector_spot.application.recording.semantic_command_codec import serialize_recorded_command, deserialize_recorded_command
from fault_detector_spot.inspection.execution.saved_probe_motion import probe_point_plan
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeSetupMotionCommandFactory
from fault_detector_spot.inspection.behaviours.execute_probe_point import ExecuteProbePoint
from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome as Outcome, ArmMovementUpdate
from fault_detector_spot.application.api.probe_point_execution_api import ProbePointExecutionApi
from fault_detector_spot.application.coordinators.sensor_acquisition_coordinator import SensorAcquisitionStatus as Acquisition
from test_probe_execution_target import inspection_object, sensor


def command(**kwargs):
    return SemanticCommand(CommandID.EXECUTE_PROBE_POINT, wait_time=2.5,
                           inspection=InspectionSelection("motor_a", "scan", "point_1"), **kwargs)


def test_intent_wire_and_saved_recording_preserve_duration_retries_and_point():
    intent = OperationalIntent()
    intent.intent = intent.INTENT_EXECUTE_PROBE_POINT
    intent.duration_sec, intent.retries = 2.5, 2
    intent.object_id, intent.routine_id, intent.probe_point_id = "motor_a", "scan", "point_1"
    semantic = operational_intent_to_command(intent)
    assert semantic.inspection == command().inspection
    assert semantic.retries == 2
    assert semantic.wait_time == 2.5
    assert semantic_command_from_message(semantic_command_to_message(semantic)) == semantic
    assert deserialize_recorded_command(serialize_recorded_command(semantic)) == semantic
    intent.probe_point_id = ""
    with pytest.raises(ValueError):
        operational_intent_to_command(intent)


@pytest.mark.parametrize("updates", [{"retries": -1}, {"retries": True}, {"retries": 101},
                                      {"wait_time": 0}, {"wait_time": float("nan")}])
def test_invalid_execution_options_are_rejected(updates):
    with pytest.raises(ValueError):
        replace(command(), **updates)


def test_surface_plan_preserves_context_and_returns_via_saved_alignment():
    tag = TagElement()
    tag.id, tag.pose.header.frame_id, tag.pose.pose.orientation.w = 2, "body", 1.0
    source = Mock(reference_tag=Mock(return_value=tag))
    attachment = sensor()
    plan = probe_point_plan(command(motion_sensor_id=attachment.motion_sensor_id),
                            Mock(load=Mock(return_value=inspection_object())), source,
                            Mock(require_motion_attachment=Mock(return_value=attachment)),
                            ProbeSetupMotionCommandFactory())
    assert len(plan) == 5
    assert plan[2].command_id is CommandID.MOVE_CLOSE_TO_SURFACE
    assert plan[2].target_surface_distance_m == 0.03
    assert plan[3].offset == plan[1].offset
    assert plan[4].offset == plan[0].offset
    assert all(step.inspection == command().inspection for step in plan)
    source.reference_tag.assert_called_once_with(2)


def step(name):
    return SimpleNamespace(name=name, command_id=CommandID.MOVE_ARM_TO_TAG)


def action(retries=0):
    value = ExecuteProbePoint("Test probe execution")
    value.initialise()
    value._phase = "motion"
    value._last_command = lambda: SimpleNamespace(retries=retries)
    value._steps = [[step("safe")], [step("a"), step("b"), step("aligned")],
                    [step("m"), step("measure")], [step("m"), step("aligned")],
                    [step("b"), step("a"), step("safe")]]
    value.executor = Mock(active=False, safe_approach_speed=object())
    value.executor.tag_probe.return_value = ArmMovementUpdate(Outcome.SUCCESS, "arrived")
    value.executor.confirm_stop.return_value = ArmMovementUpdate(Outcome.SUCCESS, "stopped")
    value._surface = Mock(active=False)
    value._fail = lambda detail: Status.FAILURE
    return value


def run(value, limit=80):
    for _ in range(limit):
        result = value.update()
        if result is not Status.RUNNING:
            return result
    pytest.fail("Execution did not terminate")


def test_success_waits_for_recording_finalization_before_retraction():
    value = action()
    responses = iter(["starting", "recording", "stopping", "complete"])
    calls = []
    def rpc(operation):
        calls.append(operation)
        assert [c.args[0].name for c in value.executor.tag_probe.call_args_list] == [
            "safe", "a", "b", "aligned", "m", "measure"]
        return SimpleNamespace(recording_state=next(responses), detail="recording")
    value._rpc = rpc
    assert run(value) is Status.SUCCESS
    assert calls == [ProbePointExecutionStep.Request.RECORD] + [ProbePointExecutionStep.Request.POLL] * 3
    assert [c.args[0].name for c in value.executor.tag_probe.call_args_list][-5:] == ["m", "aligned", "b", "a", "safe"]


def test_retry_confirms_stop_then_retraces_only_reached_waypoints():
    value = action(retries=1)
    failed = [False]
    events = []
    def move(target, **kwargs):
        events.append(target.name)
        if target.name == "b" and not failed[0]:
            failed[0] = True
            return ArmMovementUpdate(Outcome.PLANNING_FAILED, "planning failed")
        return ArmMovementUpdate(Outcome.SUCCESS, "arrived")
    value.executor.tag_probe.side_effect = move
    value.executor.confirm_stop.side_effect = lambda: (events.append("stop") or ArmMovementUpdate(Outcome.SUCCESS, "stopped"))
    value._rpc = lambda op: SimpleNamespace(recording_state="complete", detail="saved")
    assert run(value) is Status.SUCCESS
    assert events[:8] == ["safe", "a", "b", "stop", "a", "safe", "safe", "a"]
    assert value._attempt == 1


@pytest.mark.parametrize("outcome", [Outcome.STOP_UNCONFIRMED, Outcome.RETREAT_FAILED,
                                      Outcome.FORCE_STALE, Outcome.EXECUTION_ERROR])
def test_safety_failures_never_retry(outcome):
    value = action(retries=3)
    value.executor.tag_probe.return_value = ArmMovementUpdate(outcome, "failure")
    assert run(value) is Status.FAILURE
    assert value.executor.tag_probe.call_count == 1
    value.executor.confirm_stop.assert_not_called()


@pytest.mark.parametrize("stop_outcome", [Outcome.STOP_UNCONFIRMED, Outcome.SUCCESS])
def test_stop_or_recovery_failure_prevents_retry(stop_outcome):
    value = action(retries=3)
    value.executor.tag_probe.side_effect = [ArmMovementUpdate(Outcome.PLANNING_FAILED, "failed"),
                                           ArmMovementUpdate(Outcome.MOTION_FAILED, "recovery failed")]
    value.executor.confirm_stop.return_value = ArmMovementUpdate(stop_outcome, "stop")
    assert run(value) is Status.FAILURE
    assert value._attempt == 0
    assert value.executor.tag_probe.call_count == (1 if stop_outcome is Outcome.STOP_UNCONFIRMED else 2)


def test_retry_count_means_additional_attempts():
    value = action(retries=2)
    def move(target, **kwargs):
        return ArmMovementUpdate(Outcome.PLANNING_FAILED if value._phase == "motion" else Outcome.SUCCESS, "result")
    value.executor.tag_probe.side_effect = move
    assert run(value) is Status.FAILURE
    assert value._attempt == 2


def test_cancellation_stops_active_execution_without_retry():
    value = action(retries=2)
    value.executor.active = True
    value._surface.active = True
    value.terminate(Status.INVALID)
    value.executor.cancel.assert_called()
    value._surface.cancel.assert_called_once()
    assert value._attempt == 0


def api():
    value = ProbePointExecutionApi.__new__(ProbePointExecutionApi)
    value._lock = RLock()
    value._request = SimpleNamespace(request_id="current", command=command())
    value.controller = SimpleNamespace(active_request_id="current")
    value._timer = Mock()
    value.acquisition = Mock()
    value._recording_state = "starting"
    value._deadline = None
    value._detail = ""
    value._plan = None
    value.clock = lambda: 10.0
    return value


def test_recording_duration_starts_when_sensor_is_ready_and_waits_for_stop():
    value = api()
    value._acquisition_state(SimpleNamespace(status=Acquisition.RECORDING, detail="ready"))
    assert value._deadline == 12.5
    value._tick()
    value.acquisition.stop.assert_not_called()
    value.clock = lambda: 12.5
    value._tick()
    value.acquisition.stop.assert_called_once_with()
    assert value._recording_state == "stopping"
    value._acquisition_state(SimpleNamespace(status=Acquisition.IDLE, detail="saved"))
    assert value._recording_state == "complete"


def test_stale_recording_requests_are_rejected():
    value = api()
    response = value._handle(SimpleNamespace(request_id="old"), ProbePointExecutionStep.Response())
    assert not response.success
    value.acquisition.start.assert_not_called()


def test_failed_sensor_stop_is_not_complete():
    value = api()
    value._recording_state = "stopping"
    value._acquisition_state(SimpleNamespace(status=Acquisition.FAILED, detail="stop timeout"))
    assert value._recording_state == "failed"


def test_custom_plan_reverses_both_paths_and_preserves_waypoint_limits():
    from fault_detector_spot.inspection.model.models import PreApproachPathPoint
    from test_probe_execution_target import pose
    definition = inspection_object()
    point = definition.routines[0].probe_points[0]
    point.fully_custom = True
    point.final_probe_pose_object = pose(x=0.01)
    point.pre_approach_path = [PreApproachPathPoint("a", pose(x=0.25), .004, .4),
                               PreApproachPathPoint("b", pose(x=0.20), .003, .3)]
    point.final_probe_path = [PreApproachPathPoint("m", pose(x=0.05), .002, .2),
                              PreApproachPathPoint("n", pose(x=0.02), .001, .1)]
    tag = TagElement()
    tag.id, tag.pose.header.frame_id, tag.pose.pose.orientation.w = 2, "body", 1.0
    attachment = sensor()
    plan = probe_point_plan(command(motion_sensor_id=attachment.motion_sensor_id),
                            Mock(load=Mock(return_value=definition)),
                            Mock(reference_tag=Mock(return_value=tag)),
                            Mock(require_motion_attachment=Mock(return_value=attachment)),
                            ProbeSetupMotionCommandFactory())
    assert plan[2].command_id is CommandID.FOLLOW_MOVE_TO_TAG_PATH
    assert [p.position.x for p in plan[3].pre_approach_offsets] == [.02, .05]
    assert [p.position.x for p in plan[4].pre_approach_offsets] == [.20, .25]
    assert plan[3].pre_approach_tolerances_m == (.001, .002)
    assert plan[3].pre_approach_speed_scales == (.1, .2)
    assert plan[4].pre_approach_tolerances_m == (.003, .004)
    assert plan[4].pre_approach_speed_scales == (.3, .4)


def test_parent_cancellation_stops_recording_as_cancelled():
    from fault_detector_spot.application.controllers.command_controller import CommandControllerState
    value = api()
    value._recording_state = "recording"
    value._command_status(SimpleNamespace(request=value._request, request_id="current",
                                         state=CommandControllerState.CANCELLED))
    value.acquisition.stop.assert_called_once_with(cancelled=True)
    assert value._request is None


def test_record_request_uses_parent_context_not_ui_selection():
    value = api()
    value._recording_state = "idle"
    value._plan = (object(),)
    captured = object()
    value.recording_request = Mock(return_value=captured)
    value.acquisition.snapshot.return_value = SimpleNamespace(status=Acquisition.STARTING)
    request = ProbePointExecutionStep.Request(request_id="current", operation=ProbePointExecutionStep.Request.RECORD)
    response = value._handle(request, ProbePointExecutionStep.Response())
    assert response.success
    value.recording_request.assert_called_once_with(value._request.command)
    value.acquisition.start.assert_called_once_with(captured)


def test_unconfirmed_cancellation_times_out_without_recovery(monkeypatch):
    from fault_detector_spot.inspection.behaviours import execute_probe_point
    value = action(retries=2)
    value._phase = "stop"
    value._stop_deadline = 10.0
    value.executor.active = True
    monkeypatch.setattr(execute_probe_point.time, "monotonic", lambda: 11.0)
    assert value.update() is Status.FAILURE
    value.executor.confirm_stop.assert_not_called()
    value.executor.tag_probe.assert_not_called()


def test_recording_context_resolves_selected_routine_tag_without_global_selection():
    from fault_detector_spot.shared.geometry.models import PoseData
    value = api()
    value._plan = (SimpleNamespace(tag=SimpleNamespace(id=42)),)
    value.setup = SimpleNamespace(motion_state_source=Mock())
    value.setup.motion_state_source.object_pose_execution.return_value = PoseData.identity()
    request = value.recording_request(value._request.command)
    assert (request.object_id, request.routine_id, request.probe_point_id) == ("motor_a", "scan", "point_1")
    assert request.execution_frame == "odom"
    assert request.object_pose_execution == PoseData.identity()
    value.setup.motion_state_source.object_pose_execution.assert_called_once_with(42)


def test_surface_measurement_uses_surface_executor_and_records_only_after_success():
    from fault_detector_spot.manipulation.move_close_to_surface_execution import MoveCloseToSurfaceOutcome
    value = action()
    surface_step = SimpleNamespace(command_id=CommandID.MOVE_CLOSE_TO_SURFACE)
    value._steps[2] = [surface_step]
    value._surface.start.return_value = MoveCloseToSurfaceOutcome.RUNNING
    value._surface.poll.return_value = MoveCloseToSurfaceOutcome.SUCCESS
    value._surface.feedback_message = "distance verified"
    value._surface.retry_eligible = False
    value._rpc = Mock(return_value=SimpleNamespace(recording_state="complete", detail="saved"))
    assert run(value) is Status.SUCCESS
    value._surface.start.assert_called_once_with(surface_step)
    value._surface.poll.assert_called_once()
    assert all(c.args[0] is not surface_step for c in value.executor.tag_probe.call_args_list)
    value._rpc.assert_called_once_with(ProbePointExecutionStep.Request.RECORD)


def test_geometry_reference_uses_observation_time():
    from fault_detector_spot.inspection.setup.probe_setup_motion_state_source import ProbeSetupMotionStateSource
    from test_probe_execution_target import pose
    source = ProbeSetupMotionStateSource.__new__(ProbeSetupMotionStateSource)
    tag = TagElement()
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 12
    tag.pose.pose.position.x = 2.0
    tag.pose.pose.orientation.w = 1.0
    source.reference_tag = Mock(return_value=tag)
    source._lookup_pose = Mock(return_value=pose(x=3.0))
    result = source.object_pose_execution(42)
    assert result.position.x == 5.0
    source.reference_tag.assert_called_once_with(42)
    args, kwargs = source._lookup_pose.call_args
    assert args == ("odom", "body")
    assert kwargs["lookup_time"].nanoseconds == 12000000000
