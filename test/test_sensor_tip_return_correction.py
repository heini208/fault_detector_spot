"""Fresh settled poses gate bounded, improving corrections without relaxing accuracy."""

from copy import deepcopy
from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.inspection.geometry.pose import compose_poses, relative_pose
from fault_detector_spot.inspection.model.sensor_models import BARE_HAND_MOTION_ID
from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome, ArmMovementUpdate
from fault_detector_spot.manipulation.commands.measure_sensor_tip_distance_command import MeasureSensorTipDistanceCommand
from fault_detector_spot.manipulation.measure_sensor_tip_distance_execution import (
    MeasureSensorTipDistanceConfig, MeasureSensorTipDistanceExecution,
    MeasureSensorTipDistanceOutcome,
)
from fault_detector_spot.manipulation.move_close_to_surface_execution import MoveCloseToSurfaceConfig
from fault_detector_spot.shared.geometry.rotation import multiply_quaternions, quaternion_from_euler
from fault_detector_spot.shared.geometry.transforms import pose_to_pose_data
from test_measure_sensor_tip_distance_execution import Clock, Executor, Source, finish, pose


class ReturnExecutor(Executor):
    """Achieve each requested hand pose with a controlled residual rigid transform."""

    def __init__(self, clock):
        super().__init__(clock)
        self.clock = clock
        self.motion_errors = {2: pose(x=0.0078)}
        self.completed_at = {}
        self.hand_to_probe = relative_pose(
            self.probe_motion_planner.hand, self.probe_motion_planner.tip,
        )

    def poll(self):
        assert self.active
        self.active = False
        if self.pending is None:
            return ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "stop confirmed")
        index = len(self.calls)
        outcome = self.outcomes.get(index, ArmMovementOutcome.SUCCESS)
        if outcome is ArmMovementOutcome.SUCCESS:
            hand = pose_to_pose_data(self.pending.pose)
            error = self.motion_errors.get(index, pose())
            for axis in ("x", "y", "z"):
                setattr(hand.position, axis,
                        getattr(hand.position, axis) + getattr(error.position, axis))
            hand.orientation = multiply_quaternions(error.orientation, hand.orientation)
            self.probe_motion_planner.hand = hand
            self.probe_motion_planner.tip = compose_poses(hand, self.hand_to_probe)
        self.completed_at[index] = self.clock.now
        self.pending = None
        return ArmMovementUpdate(outcome, outcome.value)


def harness(**config):
    config.setdefault("return_position_tolerance_m", 0.005)
    clock = Clock()
    executor = ReturnExecutor(clock)
    source = Source(executor, clock, visible=False)
    execution = MeasureSensorTipDistanceExecution(
        executor, source, MeasureSensorTipDistanceConfig(**config),
        monotonic_clock=clock,
    )
    command = MeasureSensorTipDistanceCommand(
        CommandID.MEASURE_SENSOR_TIP_DISTANCE, object(), expected_distance_m=0.25,
    )
    return SimpleNamespace(
        clock=clock, executor=executor, source=source,
        execution=execution, command=command,
    )


def test_default_10_mm_return_tolerance_accepts_settled_7_43_mm_offset():
    case = harness(return_position_tolerance_m=MeasureSensorTipDistanceConfig().return_position_tolerance_m)
    case.executor.motion_errors[2] = pose(z=-0.00743)
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.executor.calls) == 2
    assert case.execution.config.return_position_tolerance_m == 0.010
    assert case.execution.config.tip_alignment_position_tolerance_m == 0.010
    assert case.execution.config.tip_alignment_start_position_tolerance_m == 0.005


def returned(case):
    outcome = case.execution.start(case.command)
    for _ in range(100):
        if len(case.executor.calls) == 2 and not case.executor.active:
            return outcome
        assert outcome is MeasureSensorTipDistanceOutcome.RUNNING
        case.clock.now += 0.25
        outcome = case.execution.poll()
    pytest.fail("Observation return did not finish")


def begin_correction(case):
    outcome = returned(case)
    for _ in range(40):
        if len(case.executor.calls) == 3:
            return outcome
        assert outcome is MeasureSensorTipDistanceOutcome.RUNNING
        case.clock.now += 0.25
        outcome = case.execution.poll()
    pytest.fail("Slow return correction was not started")


def test_7_8_mm_return_error_gets_one_slow_guarded_correction_to_same_saved_hand_pose():
    case = harness()
    original_hand = deepcopy(case.executor.probe_motion_planner.hand)
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.executor.calls) == 3
    outward, returning, correction = case.executor.calls
    assert returning[0] == correction[0]
    assert correction[0].header.frame_id == "odom"
    assert pose_to_pose_data(correction[0].pose) == original_hand
    assert returning[1] == correction[1] == BARE_HAND_MOTION_ID
    for _, _, options in (outward, returning, correction):
        assert options["cartesian_path"] is True
        assert options["ignore_environment_collisions"] is False
        assert options["force_threshold_n"] == case.execution.config.force_contact_threshold_n
        assert options["planning_frame_position_tolerance_m"] == pytest.approx(0.001)
        assert options["planning_frame_orientation_tolerance_rad"] == pytest.approx(math.radians(0.2))
    speed = correction[2]["speed"]
    previous_speed = returning[2]["speed"]
    assert speed.linear_speed_mps == pytest.approx(previous_speed.linear_speed_mps * 0.25)
    assert speed.angular_speed_rad_s == pytest.approx(previous_speed.angular_speed_rad_s * 0.25)
    assert case.execution.result.distance_m == pytest.approx(0.25)
    assert "1 slow return correction(s)" in case.execution.feedback_message
    assert case.source.snapshot_calls[-1] >= (
        case.executor.completed_at[3] + case.execution.config.settle_sec
    )
    assert case.source.pose_sample_calls[-1][1] >= (
        case.executor.completed_at[3] + case.execution.config.settle_sec
    )
    assert case.executor.cancel_count == case.executor.stop_count == 0


def test_result_uses_achieved_corrected_tip_and_original_tolerances():
    case = harness()
    case.executor.motion_errors[3] = pose(x=0.003)
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.executor.calls) == 3
    assert case.execution.result.distance_m == pytest.approx(0.247)
    assert case.execution.config.return_position_tolerance_m == 0.005
    assert case.execution.config.return_orientation_tolerance_rad == pytest.approx(math.radians(2.0))


def test_converging_7_8_to_5_7_to_3_mm_return_succeeds_after_two_corrections():
    case = harness()
    case.executor.motion_errors.update({3: pose(x=0.0057), 4: pose(x=0.003)})
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.executor.calls) == 4
    assert all(call[0] == case.executor.calls[1][0] for call in case.executor.calls[2:])
    assert case.execution.result.distance_m == pytest.approx(0.247)
    assert "2 slow return correction(s)" in case.execution.feedback_message
    assert case.executor.cancel_count == case.executor.stop_count == 0


@pytest.mark.parametrize("residual", [0.0078, 0.0085, 0.0071])
def test_constant_worse_or_insufficiently_improved_return_stops_after_one_correction(residual):
    case = harness()
    case.executor.motion_errors[3] = pose(x=residual)
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 3
    assert case.execution.result is None
    assert case.execution.failure_outcome is ArmMovementOutcome.RECOVERY_FAILED
    assert "did not improve enough" in case.execution.feedback_message


def test_rotation_can_prevent_progress_even_when_translation_improves():
    case = harness()
    case.executor.motion_errors.update({
        2: pose(x=0.010),
        3: pose(x=0.006, orientation=quaternion_from_euler("y", math.radians(3.8))),
    })
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 3
    assert "did not improve enough" in case.execution.feedback_message


def test_improving_return_stops_after_three_corrections_if_still_outside_tolerance():
    case = harness()
    case.executor.motion_errors = {
        index: pose(x=error)
        for index, error in enumerate((0.012, 0.010, 0.008, 0.006), start=2)
    }
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 5
    assert case.execution.result is None
    assert "Return correction limit reached" in case.execution.feedback_message


@pytest.mark.parametrize("rotation", [False, True])
def test_total_travel_budget_limits_otherwise_improving_corrections(rotation):
    case = harness()
    residuals = (4.5, 4.0, 3.5) if rotation else (0.019, 0.016, 0.012)
    case.executor.motion_errors = {
        index: (pose(orientation=quaternion_from_euler("y", math.radians(value)))
                if rotation else pose(x=value))
        for index, value in enumerate(residuals, start=2)
    }
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 4
    assert case.execution.result is None
    assert "Total return correction travel budget exceeded" in case.execution.feedback_message


@pytest.mark.parametrize("error", [0.0, 0.0049])
def test_acceptable_first_return_does_not_make_an_extra_movement(error):
    case = harness()
    case.executor.motion_errors[2] = pose(x=error)
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.executor.calls) == 2
    assert case.execution.result.distance_m == pytest.approx(0.25 - error)


def test_small_rotation_miss_gets_one_correction_and_reverifies_tip_offset():
    case = harness()
    case.executor.motion_errors[2] = pose(
        orientation=quaternion_from_euler("y", math.radians(3.0)),
    )
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.executor.calls) == 3
    assert case.executor.probe_motion_planner.tip == pose(x=0.06, z=-0.10)


@pytest.mark.parametrize("frame", ["hand", "tip"])
@pytest.mark.parametrize("kind", ["translation", "rotation"])
def test_bounded_correction_requires_both_hand_and_tip_errors_within_limit(frame, kind):
    case = harness()
    snapshot = case.source.sensor_tip_pose_snapshot

    def excessive_frame_error(*args, **kwargs):
        hand, tip, body = snapshot(*args, **kwargs)
        if len(case.executor.calls) == 2:
            target = hand if frame == "hand" else tip
            if kind == "translation":
                target.position.y += 0.021
            else:
                target.orientation = quaternion_from_euler("z", math.radians(5.1))
        return hand, tip, body

    case.source.sensor_tip_pose_snapshot = excessive_frame_error
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 2
    assert case.execution.result is None
    assert "outside the bounded" in case.execution.feedback_message


@pytest.mark.parametrize("reason", ["fresh depth timestamp unavailable", "stale hand transform"])
def test_missing_or_stale_return_snapshot_never_authorizes_a_correction(reason):
    case = harness()
    returned(case)
    case.source.snapshot_error = reason
    outcome = finish(case, MeasureSensorTipDistanceOutcome.RUNNING)
    assert outcome is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 2
    assert reason in case.execution.feedback_message


@pytest.mark.parametrize("stage", ["original", "return"])
@pytest.mark.parametrize("frame", ["hand", "probe", "body"])
@pytest.mark.parametrize("kind", ["translation", "rotation"])
def test_oscillating_pose_never_authorizes_original_motion_or_return_correction(stage, frame, kind):
    case = harness()
    outcome = returned(case) if stage == "return" else case.execution.start(case.command)
    read = case.source.sensor_tip_pose_sample
    count = 0

    def oscillating(*args, **kwargs):
        nonlocal count
        sample = read(*args, **kwargs)
        count += 1
        target = getattr(sample, frame)
        sign = 1.0 if count % 2 else -1.0
        if kind == "translation":
            target.position.y += sign * 0.0006
        else:
            target.orientation = quaternion_from_euler("z", sign * math.radians(0.11))
        return sample

    case.source.sensor_tip_pose_sample = oscillating
    assert finish(case, outcome) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == (2 if stage == "return" else 0)
    assert case.execution.result is None
    assert "stable hand, sensor tip and body poses" in case.execution.feedback_message


@pytest.mark.parametrize("rotation", [False, True])
def test_stability_limits_the_whole_pose_window_not_only_adjacent_samples(rotation):
    case = harness()
    read = case.source.sensor_tip_pose_sample
    count = 0

    def drifting(*args, **kwargs):
        nonlocal count
        sample = read(*args, **kwargs)
        count += 1
        if rotation:
            sample.hand.orientation = quaternion_from_euler("z", count * math.radians(0.11))
        else:
            sample.hand.position.y += count * 0.0006
        return sample

    case.source.sensor_tip_pose_sample = drifting
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.executor.calls == []
    assert case.execution.result is None
    assert "stable hand, sensor tip and body poses" in case.execution.feedback_message


@pytest.mark.parametrize("stage,first_return_error", [
    ("original", 0.0), ("return", 0.0), ("return", 0.0078),
])
@pytest.mark.parametrize("timestamp", ["repeated", "before_settle", "invalid"])
def test_unusable_acquisition_stamps_never_authorize_motion_or_success(stage, first_return_error, timestamp):
    case = harness()
    case.executor.motion_errors[2] = pose(x=first_return_error)
    outcome = returned(case) if stage == "return" else case.execution.start(case.command)
    read = case.source.sensor_tip_pose_sample

    def unusable_stamp(*args, **kwargs):
        sample = read(*args, **kwargs)
        stamp = kwargs["acquisition_not_before"]
        if timestamp == "before_settle":
            stamp -= 0.001
        elif timestamp == "invalid":
            stamp = float("nan")
        return replace(sample, stamp_seconds=stamp)

    case.source.sensor_tip_pose_sample = unusable_stamp
    assert finish(case, outcome) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == (2 if stage == "return" else 0)
    assert case.execution.result is None


def test_sparse_return_pose_acquisitions_cannot_form_a_stable_window():
    case = harness()
    outcome = returned(case)
    for _ in range(20):
        if outcome is not MeasureSensorTipDistanceOutcome.RUNNING:
            break
        case.clock.now += 0.6
        outcome = case.execution.poll()
    assert outcome is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 2
    assert case.execution.result is None
    assert "stable hand, sensor tip and body poses" in case.execution.feedback_message


@pytest.mark.parametrize("invalid", ["missing", "nan_position", "zero_quaternion"])
def test_transient_invalid_pose_resets_entire_return_stability_window(invalid):
    case = harness()
    outcome = returned(case)
    read = case.source.sensor_tip_pose_sample
    count = 0

    def temporarily_invalid(*args, **kwargs):
        nonlocal count
        sample = read(*args, **kwargs)
        count += 1
        if count == 3:
            if invalid == "missing":
                raise ValueError("temporarily unavailable transform")
            if invalid == "nan_position":
                sample.hand.position.x = float("nan")
            else:
                sample.probe.orientation.x = sample.probe.orientation.y = 0.0
                sample.probe.orientation.z = sample.probe.orientation.w = 0.0
        return sample

    case.source.sensor_tip_pose_sample = temporarily_invalid
    for _ in range(20):
        if len(case.executor.calls) == 3:
            break
        assert outcome is MeasureSensorTipDistanceOutcome.RUNNING
        case.clock.now += 0.25
        outcome = case.execution.poll()
        if count < 6:
            assert len(case.executor.calls) == 2
    assert count == 6
    assert len(case.executor.calls) == 3
    assert finish(case, outcome) is MeasureSensorTipDistanceOutcome.SUCCESS


def test_original_pose_needs_three_distinct_samples_spanning_required_duration():
    case = harness()
    assert case.execution.start(case.command) is MeasureSensorTipDistanceOutcome.RUNNING
    for instant in (100.5, 100.6, 100.7):
        case.clock.now = instant
        assert case.execution.poll() is MeasureSensorTipDistanceOutcome.RUNNING
        assert case.executor.calls == []
    case.clock.now = 100.81
    assert case.execution.poll() is MeasureSensorTipDistanceOutcome.RUNNING
    assert len(case.executor.calls) == 1


def test_initial_attachment_change_restarts_original_pose_stability_window():
    case = harness()
    assert case.execution.start(case.command) is MeasureSensorTipDistanceOutcome.RUNNING
    for instant in (100.5, 100.7):
        case.clock.now = instant
        assert case.execution.poll() is MeasureSensorTipDistanceOutcome.RUNNING
        assert not case.executor.calls
    case.source.attachment = ("probe", 2)
    for instant in (100.9, 101.1):
        case.clock.now = instant
        assert case.execution.poll() is MeasureSensorTipDistanceOutcome.RUNNING
        assert not case.executor.calls
    case.clock.now = 101.3
    assert case.execution.poll() is MeasureSensorTipDistanceOutcome.RUNNING
    assert len(case.executor.calls) == 1


def test_return_diagnostics_include_signed_odom_axis_errors_for_hand_tip_and_body():
    case = harness()
    case.executor.motion_errors[2] = pose(x=0.006, y=-0.004, z=0.003)
    messages = []
    case.execution.logger = SimpleNamespace(info=messages.append)
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    mismatch = next(message for message in messages if "Original pose not restored" in message)
    assert "hand odom delta=(+6.00, -4.00, +3.00) mm" in mismatch
    assert "tip odom delta=(+6.00, -4.00, +3.00) mm" in mismatch
    assert "body odom delta=(+0.00, +0.00, +0.00) mm" in mismatch
    assert any("Original pose verified" in message for message in messages)


@pytest.mark.parametrize("change", ["attachment", "body_position", "body_orientation"])
def test_changed_return_reference_never_authorizes_correction(change):
    case = harness()
    returned(case)
    if change == "attachment":
        case.source.attachment = ("probe", 2)
    elif change == "body_position":
        case.executor.probe_motion_planner.body.position.x += 0.011
    else:
        case.executor.probe_motion_planner.body.orientation = quaternion_from_euler(
            "z", math.radians(2.1),
        )
    assert finish(case, MeasureSensorTipDistanceOutcome.RUNNING) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 2
    assert case.execution.result is None


@pytest.mark.parametrize("movement", [2, 3])
@pytest.mark.parametrize("failure", [ArmMovementOutcome.CONTACT, ArmMovementOutcome.MOTION_FAILED])
def test_unsuccessful_return_or_correction_does_not_start_another_motion(movement, failure):
    case = harness()
    case.executor.outcomes[movement] = failure
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == movement
    assert case.execution.result is None
    assert case.execution.failure_outcome is ArmMovementOutcome.RECOVERY_FAILED


def test_failed_observation_can_correct_return_but_cannot_turn_into_measurement_success():
    case = harness()
    case.executor.outcomes[1] = ArmMovementOutcome.MOTION_FAILED
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert len(case.executor.calls) == 3
    assert case.executor.stop_count == 1
    assert case.executor.probe_motion_planner.hand == pose()
    assert case.execution.result is None
    assert "Observation movement failed" in case.execution.feedback_message
    assert "returned to original pose" in case.execution.feedback_message


def test_cancel_during_correction_stops_without_another_return():
    case = harness()
    begin_correction(case)
    case.execution.cancel()
    assert case.executor.cancel_count == 1
    assert len(case.executor.calls) == 3
    assert not case.execution.active
    assert case.execution.result is None


def test_correction_does_not_extend_overall_measurement_deadline():
    case = harness()
    begin_correction(case)
    case.clock.now = 160.01
    assert case.execution.poll() is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.executor.cancel_count == 1
    assert len(case.executor.calls) == 3
    assert "timed out" in case.execution.feedback_message


@pytest.mark.parametrize("busy_at_dispatch", [False, True])
def test_busy_executor_prevents_correction_without_cancelling_other_owner(busy_at_dispatch):
    case = harness()
    returned(case)
    if busy_at_dispatch:
        def busy(*args, **kwargs):
            case.executor.active = True
            return ArmMovementUpdate(ArmMovementOutcome.BUSY, "Another operation owns the arm")
        case.executor.guarded_probe = busy
    else:
        case.executor.active = True
    assert finish(case, MeasureSensorTipDistanceOutcome.RUNNING) is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.execution.failure_outcome is ArmMovementOutcome.BUSY
    assert len(case.executor.calls) == 2
    assert case.executor.active
    assert case.executor.cancel_count == 0


@pytest.mark.parametrize("scale", [0.0, -0.1, 1.0, 1.1, float("nan"), float("inf")])
def test_correction_speed_must_remain_positive_finite_and_slower(scale):
    with pytest.raises(ValueError):
        MeasureSensorTipDistanceConfig(return_correction_speed_scale=scale).validate()


@pytest.mark.parametrize("config_type", [MeasureSensorTipDistanceConfig, MoveCloseToSurfaceConfig])
def test_node_configuration_preserves_pose_sample_and_correction_counts_as_integers(config_type):
    values = {
        "close_surface.pose_stability_minimum_samples": 4,
        "close_surface.maximum_return_corrections": 2,
    }
    node = SimpleNamespace(
        has_parameter=lambda name: name in values,
        declare_parameter=lambda name, value: values.setdefault(name, value),
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
    )
    config = config_type.from_node(node)
    assert config.pose_stability_minimum_samples == 4
    assert type(config.pose_stability_minimum_samples) is int
    assert config.maximum_return_corrections == 2
    assert type(config.maximum_return_corrections) is int


@pytest.mark.parametrize("overrides", [
    {"maximum_return_corrections": 1.5},
    {"pose_stability_minimum_samples": 2},
    {"pose_stability_minimum_samples": 3.0},
    {"minimum_return_correction_improvement_fraction": 1.0},
    {"maximum_return_correction_total_translation_m": float("nan")},
    {"maximum_return_correction_total_rotation_rad": -0.1},
    {"pose_stability_minimum_span_sec": 6.0},
])
def test_invalid_stability_and_correction_limits_are_rejected(overrides):
    with pytest.raises(ValueError):
        MeasureSensorTipDistanceConfig(**overrides).validate()
