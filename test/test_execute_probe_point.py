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


@pytest.mark.parametrize("ignore_environment_collisions", [False, True])
@pytest.mark.parametrize("distance", [0.0, 0.03])
def test_surface_plan_preserves_context_and_returns_via_saved_alignment(ignore_environment_collisions, distance):
    tag = TagElement()
    tag.id, tag.pose.header.frame_id, tag.pose.pose.orientation.w = 2, "body", 1.0
    source = Mock(reference_tag=Mock(return_value=tag))
    attachment = sensor()
    definition = inspection_object()
    definition.routines[0].probe_points[0].target_surface_distance_m = distance
    plan = probe_point_plan(command(motion_sensor_id=attachment.motion_sensor_id,
                                    ignore_environment_collisions=ignore_environment_collisions),
                            Mock(load=Mock(return_value=definition)), source,
                            Mock(require_motion_attachment=Mock(return_value=attachment)),
                            ProbeSetupMotionCommandFactory())
    assert len(plan) == 3
    assert plan[2].command_id is CommandID.MOVE_CLOSE_TO_SURFACE
    assert plan[2].target_surface_distance_m == distance
    assert all(step.inspection == command().inspection for step in plan)
    assert [step.ignore_environment_collisions for step in plan] == [
        False, False, True,
    ]
    source.reference_tag.assert_not_called()
    assert plan[0].offset.frame_id == "filtered_fiducial_2"
    assert plan[1].offset.frame_id == "filtered_fiducial_2"


def step(name):
    return SimpleNamespace(
        name=name, command_id=CommandID.MOVE_ARM_TO_TAG,
        ignore_environment_collisions=False,
    )


def response(state="complete", stopped=True):
    return SimpleNamespace(success=True, recording_state=state,
                           recording_stopped=stopped, detail=state)


def action(retries=0):
    value = ExecuteProbePoint("Test probe execution")
    value.initialise()
    value._phase = "motion"
    value._sensor_id = "probe"
    value._history = ["initial"]
    value._last_command = lambda: SimpleNamespace(retries=retries)
    value._steps = [[step("safe")], [step("a"), step("b"), step("aligned")],
                    [step("m"), step("measure")]]
    value.executor = Mock(active=False, safe_approach_speed=object())
    value.events = []
    value.position = "initial"
    value.failures = {}

    def move(kind, target):
        value.events.append((kind, target))
        failures = value.failures.get((kind, target), [])
        outcome = failures.pop(0) if failures else Outcome.SUCCESS
        value.position = target if outcome is Outcome.SUCCESS else "partial:" + target
        return ArmMovementUpdate(outcome, outcome.value)

    value.executor.tag_probe.side_effect = lambda target, **kw: move("move", target.name)
    value.executor.restore_probe_checkpoint.side_effect = lambda target, **kwargs: move("restore", target)
    value.executor.capture_probe_checkpoint.side_effect = lambda sensor: value.position
    value.executor.confirm_stop.side_effect = lambda: (
        value.events.append(("stop", None)) or ArmMovementUpdate(Outcome.SUCCESS, "stopped"))
    value._surface = Mock(active=False, failure_outcome=None)
    value._fail = Mock(return_value=Status.FAILURE)
    value._rpc = Mock(return_value=response())
    return value


def run(value, limit=120):
    for _ in range(limit):
        result = value.update()
        if result is not Status.RUNNING:
            return result
    pytest.fail("Execution did not terminate")


def test_success_waits_for_finalization_then_reverses_reached_path_to_initial_pose():
    value = action()
    responses = iter(["starting", "recording", "stopping", "complete"])
    calls = []
    def rpc(operation):
        calls.append(operation)
        assert all(kind == "move" for kind, _ in value.events)
        return response(next(responses))
    value._rpc = rpc
    assert run(value) is Status.SUCCESS
    assert calls == [ProbePointExecutionStep.Request.RECORD] + [ProbePointExecutionStep.Request.POLL] * 3
    assert value.events == [("move", name) for name in ("safe", "a", "b", "aligned", "m", "measure")] + [
        ("restore", name) for name in ("m", "aligned", "b", "a", "safe")]
    assert value.position == "safe"


@pytest.mark.parametrize("failure", [Outcome.CONTACT, Outcome.PLANNING_FAILED,
                                      Outcome.MOTION_FAILED, Outcome.FORCE_STALE,
                                      Outcome.EXECUTION_ERROR])
def test_failed_path_step_recovers_previous_checkpoint_and_resumes_same_step(failure):
    value = action(retries=1)
    value.failures[("move", "b")] = [failure]
    assert run(value) is Status.SUCCESS
    assert value.events[:7] == [("move", "safe"), ("move", "a"), ("move", "b"),
                                ("stop", None), ("restore", "a"), ("move", "b"), ("move", "aligned")]
    assert value._attempt == 1
    assert value.events[-1] == ("restore", "safe")


def test_exhausted_forward_failure_backtracks_only_reached_goals():
    value = action(retries=1)
    value.failures[("move", "b")] = [Outcome.CONTACT, Outcome.CONTACT]
    assert run(value) is Status.FAILURE
    assert value.events[-4:] == [("move", "b"), ("stop", None), ("restore", "a"),
                                 ("restore", "safe")]
    assert not any(target in ("aligned", "m", "measure") for _, target in value.events)
    assert value.position == "safe"
    assert "returned to safe approach" in value._fail.call_args.args[0]


def test_first_goal_failure_uses_initial_pose_as_checkpoint():
    value = action(retries=1)
    value.failures[("move", "safe")] = [Outcome.CONTACT]
    assert run(value) is Status.SUCCESS
    assert value.events[:4] == [("move", "safe"), ("stop", None),
                                ("restore", "initial"), ("move", "safe")]


@pytest.mark.parametrize("outcome", list(ExecuteProbePoint.UNSAFE_TO_RECOVER))
def test_unsafe_outcomes_never_recover_or_retry(outcome):
    value = action(retries=3)
    value.failures[("move", "safe")] = [outcome]
    assert run(value) is Status.FAILURE
    assert value.events == [("move", "safe")]


def test_unconfirmed_stop_prevents_recovery():
    value = action(retries=3)
    value.failures[("move", "safe")] = [Outcome.CONTACT]
    value.executor.confirm_stop.side_effect = None
    value.executor.confirm_stop.return_value = ArmMovementUpdate(Outcome.STOP_UNCONFIRMED, "no stop")
    assert run(value) is Status.FAILURE
    value.executor.restore_probe_checkpoint.assert_not_called()


def test_failed_checkpoint_recovery_never_skips_to_earlier_goal():
    value = action(retries=3)
    value.failures[("move", "b")] = [Outcome.CONTACT]
    value.failures[("restore", "a")] = [Outcome.CONTACT]
    assert run(value) is Status.FAILURE
    assert value.events[-1] == ("restore", "a")
    assert value._attempt == 1
    assert "Checkpoint recovery failed" in value._fail.call_args.args[0]


def test_retry_budget_is_shared_across_failed_steps():
    value = action(retries=2)
    for name in ("a", "b", "m"):
        value.failures[("move", name)] = [Outcome.CONTACT]
    assert run(value) is Status.FAILURE
    assert value._attempt == 2
    assert value.events.count(("move", "a")) == 2
    assert value.events.count(("move", "b")) == 2
    assert value.events.count(("move", "m")) == 1
    assert value.position == "safe"


def test_return_failure_recovers_last_successful_return_goal_then_retries():
    value = action(retries=1)
    value.failures[("restore", "b")] = [Outcome.CONTACT]
    assert run(value) is Status.SUCCESS
    i = value.events.index(("restore", "b"))
    assert value.events[i:i+4] == [("restore", "b"), ("stop", None),
                                   ("restore", "aligned"), ("restore", "b")]
    assert value.position == "safe"


def test_exhausted_return_failure_stops_at_last_checkpoint_without_shortcut():
    value = action(retries=0)
    value.failures[("restore", "b")] = [Outcome.CONTACT]
    assert run(value) is Status.FAILURE
    assert value.events[-3:] == [("restore", "b"), ("stop", None), ("restore", "aligned")]
    assert "return blocked" in value._fail.call_args.args[0]


def test_record_failure_aborts_before_checkpoint_recovery_and_new_recording():
    value = action(retries=1)
    operations = []
    def rpc(operation):
        operations.append(operation)
        if operations == [ProbePointExecutionStep.Request.RECORD]:
            return response("failed", False)
        if operation == ProbePointExecutionStep.Request.ABORT_RECORDING:
            assert all(kind == "move" for kind, _ in value.events)
            return response("aborting", False)
        if operations[-2:] == [ProbePointExecutionStep.Request.ABORT_RECORDING, ProbePointExecutionStep.Request.POLL]:
            return response("failed", True)
        assert value.events[-1] == ("restore", "measure")
        return response()
    value._rpc = rpc
    assert run(value) is Status.SUCCESS
    assert operations == [ProbePointExecutionStep.Request.RECORD, ProbePointExecutionStep.Request.ABORT_RECORDING,
                          ProbePointExecutionStep.Request.POLL, ProbePointExecutionStep.Request.RECORD]
    assert value._attempt == 1
    assert value.events.count(("move", "measure")) == 1


def test_exhausted_recording_failure_backtracks_after_confirmed_acquisition_stop():
    value = action(retries=0)
    value._rpc.side_effect = [response("failed", False), response("failed", True)]
    assert run(value) is Status.FAILURE
    assert value.position == "safe"
    assert value.events[-6:] == [("restore", name) for name in
                                ("measure", "m", "aligned", "b", "a", "safe")]


def test_unconfirmed_sensor_stop_prohibits_retry_and_return():
    value = action(retries=2)
    value._rpc.return_value = response("failed", False)
    assert run(value) is Status.FAILURE
    value.executor.restore_probe_checkpoint.assert_not_called()
    value.executor.confirm_stop.assert_not_called()


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
    value.acquisition = Mock(recording_stopped=True)
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


@pytest.mark.parametrize("bypass", [False, True])
def test_custom_plan_preserves_forward_waypoint_limits_for_checkpoint_execution(bypass):
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
    plan = probe_point_plan(command(motion_sensor_id=attachment.motion_sensor_id,
                                    ignore_environment_collisions=bypass),
                            Mock(load=Mock(return_value=definition)),
                            Mock(reference_tag=Mock(return_value=tag)),
                            Mock(require_motion_attachment=Mock(return_value=attachment)),
                            ProbeSetupMotionCommandFactory())
    assert plan[2].command_id is CommandID.FOLLOW_MOVE_TO_TAG_PATH
    assert len(plan) == 3
    assert [p.position.x for p in plan[1].pre_approach_offsets] == [.25, .20]
    assert [p.position.x for p in plan[2].pre_approach_offsets] == [.05, .02]
    assert plan[2].pre_approach_tolerances_m == (.002, .001)
    assert plan[2].pre_approach_speed_scales == (.2, .1)
    assert [step.ignore_environment_collisions for step in plan] == [False, False, True]
    from test_execution_command_translation import subscriber
    builder = subscriber()
    expanded = [builder.fire_command_sequence(stage) for stage in plan]
    assert [len(stage) for stage in expanded] == [1, 3, 3]
    assert [[step.ignore_environment_collisions for step in stage] for stage in expanded] == [
        [False], [False, False, False], [True, True, True],
    ]


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
    surface_step = SimpleNamespace(
        command_id=CommandID.MOVE_CLOSE_TO_SURFACE,
        ignore_environment_collisions=True,
    )
    value._steps[2] = [surface_step]
    value._surface.start.return_value = MoveCloseToSurfaceOutcome.RUNNING
    value._surface.poll.return_value = MoveCloseToSurfaceOutcome.SUCCESS
    value._surface.feedback_message = "distance verified"
    value._surface.failure_outcome = None
    value._rpc = Mock(return_value=response())
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


def test_failed_recording_can_restart_only_after_confirmed_stop_with_new_deadline():
    value = api()
    value._recording_state = "failed"
    value._deadline = 2.5
    value._plan = (object(),)
    value.recording_request = Mock(return_value=object())
    value.acquisition.snapshot.return_value = SimpleNamespace(status=Acquisition.STARTING)
    request = ProbePointExecutionStep.Request(request_id="current", operation=ProbePointExecutionStep.Request.RECORD)
    result = value._handle(request, ProbePointExecutionStep.Response())
    assert result.success
    assert value._deadline is None
    value._acquisition_state(SimpleNamespace(status=Acquisition.RECORDING, detail="new samples"))
    assert value._deadline == 12.5
    value._recording_state = "failed"
    value.acquisition.recording_stopped = False
    result = value._handle(request, ProbePointExecutionStep.Response())
    assert not result.success
    assert value.acquisition.start.call_count == 1


def test_abort_recording_service_exposes_stop_confirmation():
    value = api()
    value.acquisition.recording_stopped = False
    value.acquisition.snapshot.return_value = SimpleNamespace(status=Acquisition.STOPPING)
    request = ProbePointExecutionStep.Request(request_id="current", operation=ProbePointExecutionStep.Request.ABORT_RECORDING)
    result = value._handle(request, ProbePointExecutionStep.Response())
    assert result.success
    assert not result.recording_stopped
    assert result.recording_state == "aborting"
    value.acquisition.abort_recording.assert_called_once()
    value.acquisition.recording_stopped = True
    value._acquisition_state(SimpleNamespace(status=Acquisition.FAILED, detail="partial attempt failed"))
    request.operation = request.POLL
    result = value._handle(request, ProbePointExecutionStep.Response())
    assert result.recording_stopped
    assert result.recording_state == "failed"


def test_initial_pose_is_captured_before_the_first_forward_goal():
    value = action()
    value._phase = "plan"
    value._history = []
    wire_plan = [semantic_command_to_message(SemanticCommand(
        CommandID.MOVE_ARM_TO_TAG, motion_sensor_id="probe")) for _ in range(3)]
    value._rpc.return_value = SimpleNamespace(success=True, plan=wire_plan, recording_state="idle")
    value._builder = Mock()
    value._builder.fire_command_sequence.side_effect = [[step("safe")], [step("aligned")], [step("measure")]]
    assert value.update() is Status.RUNNING
    assert value._history == ["initial", "safe"]
    assert value.events == [("move", "safe")]


def test_surface_recovery_failure_blocks_combined_retry():
    from fault_detector_spot.manipulation.move_close_to_surface_execution import MoveCloseToSurfaceOutcome
    value = action(retries=3)
    value._stage = 2
    value._history = ["initial", "safe", "aligned"]
    value._steps[2] = [SimpleNamespace(command_id=CommandID.MOVE_CLOSE_TO_SURFACE)]
    value._surface.start.return_value = MoveCloseToSurfaceOutcome.FAILURE
    value._surface.failure_outcome = Outcome.RECOVERY_FAILED
    value._surface.feedback_message = "local recovery failed"
    assert value.update() is Status.FAILURE
    value.executor.confirm_stop.assert_not_called()
    value.executor.restore_probe_checkpoint.assert_not_called()


def test_missing_tag_waits_then_continues_without_spending_retry(monkeypatch):
    value = action(retries=1)
    value._steps[0][0].tag_id = 42
    value.executor.tag_state_source.usable_tag.return_value = None
    monkeypatch.setattr('fault_detector_spot.inspection.behaviours.execute_probe_point.time.monotonic', lambda: 10.0)
    assert value.update() is Status.RUNNING
    assert value.events == []
    assert value._attempt == 0
    value.executor.tag_state_source.usable_tag.return_value = object()
    assert value.update() is Status.RUNNING
    assert value.events == [('move', 'safe')]


def test_missing_tag_timeout_recovers_checkpoint_before_retry(monkeypatch):
    value = action(retries=1)
    value._steps[0][0].tag_id = 42
    value.executor.tag_state_source.usable_tag.return_value = None
    now = [10.0]
    monkeypatch.setattr('fault_detector_spot.inspection.behaviours.execute_probe_point.time.monotonic', lambda: now[0])
    value.update()
    now[0] = 15.1
    value.update()
    assert value._phase == 'stop'
    value.update()
    value.update()
    assert value._attempt == 1
    assert value.events == [('stop', None), ('restore', 'initial')]
    value.update()
    assert value._phase == 'motion'
    assert value._tag_wait_started == 15.1


def test_record_context_wait_does_not_switch_to_poll():
    value = action()
    value._phase = 'record'
    value._rpc.return_value = response('waiting_tag')
    assert value.update() is Status.RUNNING
    assert not value._record_started
    assert value.events == []


def test_api_waits_for_record_context_before_starting_acquisition():
    from fault_detector_spot.inspection.setup.stable_tag_pose import TagObservationUnavailable
    value = api()
    value._plan = (object(),)
    value._recording_state = 'idle'
    value.recording_request = Mock(side_effect=TagObservationUnavailable('No fresh tag'))
    request = ProbePointExecutionStep.Request()
    request.request_id = 'current'
    request.operation = request.RECORD
    result = value._handle(request, ProbePointExecutionStep.Response())
    assert result.success
    assert result.recording_state == 'waiting_tag'
    assert value._recording_state == 'idle'
    value.acquisition.start.assert_not_called()


def test_plan_tag_wait_exhausts_shared_retry_budget(monkeypatch):
    value = action(retries=1)
    value._phase = 'plan'
    value._history = []
    value._rpc.return_value = response('waiting_tag')
    now = [0.0]
    monkeypatch.setattr('fault_detector_spot.inspection.behaviours.execute_probe_point.time.monotonic', lambda: now[0])
    value.update()
    now[0] = 6.0
    value.update()
    assert value._attempt == 1
    value.update()
    now[0] = 12.0
    assert value.update() is Status.FAILURE
    assert value.events == []


@pytest.mark.parametrize("misses, expected", [(1, Status.SUCCESS), (2, Status.SUCCESS), (3, Status.FAILURE)])
def test_return_recovery_tolerance_misses_use_shared_budget(misses, expected):
    value = action(retries=3)
    value._phase = "return"
    value._history = ["initial", "safe", "aligned"]
    value.position = "aligned"
    value.failures[("restore", "safe")] = [Outcome.CHECKPOINT_TOLERANCE_FAILED]
    value.failures[("restore", "aligned")] = [Outcome.CHECKPOINT_TOLERANCE_FAILED] * misses
    assert run(value) is expected
    index = value.events.index(("restore", "safe"))
    recovery_events = value.events[index + 1:]
    for attempt in range(min(misses + 1, 3)):
        assert recovery_events[attempt * 2:attempt * 2 + 2] == [
            ("stop", None), ("restore", "aligned")]
    assert value._attempt <= 3
    if expected is Status.SUCCESS:
        assert value.position == "safe"
        assert value.events[-1] == ("restore", "safe")
    else:
        assert value.events.count(("restore", "safe")) == 1
        assert "retry budget exhausted (3/3)" in value._fail.call_args.args[0]


def test_recovery_tolerance_retry_requires_confirmed_stop():
    value = action(retries=3)
    value.failures[("move", "safe")] = [Outcome.CONTACT]
    value.failures[("restore", "initial")] = [Outcome.CHECKPOINT_TOLERANCE_FAILED]
    value.executor.confirm_stop.side_effect = [
        ArmMovementUpdate(Outcome.SUCCESS, "stopped"),
        ArmMovementUpdate(Outcome.STOP_UNCONFIRMED, "unknown"),
    ]
    assert run(value) is Status.FAILURE
    assert value.events.count(("restore", "initial")) == 1
    assert "Stop unconfirmed" in value._fail.call_args.args[0]


def test_first_safe_approach_exhaustion_recovers_initial_pose_only():
    value = action(retries=0)
    value.failures[("move", "safe")] = [Outcome.CONTACT]
    assert run(value) is Status.FAILURE
    assert value.events == [("move", "safe"), ("stop", None), ("restore", "initial")]
    assert value.position == "initial"


def test_backtracking_at_safe_approach_does_not_restore_precommand_pose():
    value = action()
    value._phase = "return"
    value._history = ["initial", "safe"]
    value.position = "safe"
    assert value.update() is Status.SUCCESS
    assert value.events == []


@pytest.mark.parametrize("budget, used", [(0, 0), (3, 3)])
def test_collision_at_fourth_waypoint_without_remaining_retries_backtracks_taken_path(budget, used):
    value = action(retries=budget)
    value._attempt = used
    value._steps = [[step("safe")],
                    [step(name) for name in ("p1", "p2", "p3", "p4", "aligned")],
                    [step("measure")]]
    value.failures[("move", "p4")] = [Outcome.CONTACT]
    assert run(value) is Status.FAILURE
    assert value.events == [
        ("move", "safe"), ("move", "p1"), ("move", "p2"),
        ("move", "p3"), ("move", "p4"), ("stop", None),
        ("restore", "p3"), ("restore", "p2"), ("restore", "p1"),
        ("restore", "safe"),
    ]
    assert value._history == ["initial", "safe"]
    assert value.position == "safe"
    assert value._attempt == used
    value._rpc.assert_not_called()
    assert "returned to safe approach" in value._fail.call_args.args[0]


def test_checkpoint_backtracking_preserves_each_forward_collision_policy():
    value = action()
    for motion in value._steps[2]:
        motion.ignore_environment_collisions = True
    assert run(value) is Status.SUCCESS
    returned = value.executor.restore_probe_checkpoint.call_args_list
    assert [call.kwargs["ignore_environment_collisions"] for call in returned] == [
        True, True, False, False, False,
    ]


def test_failed_contact_path_recovery_keeps_bypass_for_same_step_retry():
    value = action(retries=1)
    value._steps[2][0].ignore_environment_collisions = True
    value.failures[("move", "m")] = [Outcome.PLANNING_FAILED]
    assert run(value) is Status.SUCCESS
    recovery = value.executor.restore_probe_checkpoint.call_args_list[0]
    assert recovery.args == ("aligned",)
    assert recovery.kwargs["ignore_environment_collisions"] is True


@pytest.mark.parametrize("stage,bypass", [(0, False), (1, False), (2, True)])
def test_standoff_checkpoint_policy_bypasses_only_surface_edge(stage, bypass):
    value = action()
    value._steps = [[step("safe")], [step("aligned")], [SimpleNamespace(
        command_id=CommandID.MOVE_CLOSE_TO_SURFACE,
        target_surface_distance_m=0.03,
        ignore_environment_collisions=False,
    )]]
    value._stage, value._index = stage, 0
    value._resume_phase = "motion"
    reached = ["initial", "safe", "aligned", "surface"]
    value._history = reached[:stage + 1]
    assert value._checkpoint_collision_bypass() is bypass

    value._history = reached[:stage + 2]
    assert value._checkpoint_collision_bypass(returning=True) is bypass
