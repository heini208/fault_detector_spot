"""Saved observation returns retain calibrated geometry and one frozen target."""

from copy import deepcopy
import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.inspection.geometry.pose import compose_poses, inverse_pose
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import ManipulatorToTagCommand
from fault_detector_spot.manipulation.commands.measure_sensor_tip_distance_command import MeasureSensorTipDistanceCommand
from fault_detector_spot.manipulation.measure_sensor_tip_distance_execution import (
    MeasureSensorTipDistanceConfig, MeasureSensorTipDistanceExecution, MeasureSensorTipDistanceOutcome,
)
from fault_detector_spot.manipulation.probe_motion_planner import ProbeMotionPlanner, ResolvedProbeTarget
from fault_detector_spot.shared.geometry.rotation import quaternion_from_euler, rotation_distance_rad
from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose, pose_to_pose_data
from test_measure_sensor_tip_distance_execution import Clock, Planner, Source, begin_excursion, finish, pose
from test_sensor_tip_return_correction import ReturnExecutor


def stamped(value, frame="odom"):
    result = PoseStamped()
    result.header.frame_id = frame
    result.pose = pose_data_to_pose(value)
    return result


class AnchorPlanner(Planner):
    normalize_target = ProbeMotionPlanner.normalize_target
    probe_target_to_hand_target = ProbeMotionPlanner.probe_target_to_hand_target
    probe_pose_to_hand_pose = staticmethod(ProbeMotionPlanner.probe_pose_to_hand_pose)

    def __init__(self, clock, calibration, nominal_hand):
        super().__init__(clock)
        self.calibration = calibration
        self.nominal_probe = compose_poses(nominal_hand, calibration)
        self.frame_pose = pose(x=0.4, y=-0.2, z=0.1,
                               orientation=quaternion_from_euler("z", math.radians(20.0)))
        self.resolve_calls = []
        self.resolved_sensor_id = "probe"
        self.resolve_error = None
        self.transform_error = None
        self.tf_listener = SimpleNamespace(lookup_a_tform_b=self.lookup_transform)

    def resolve_tag(self, command, tag_source):
        self.resolve_calls.append((command, tag_source))
        if self.resolve_error:
            raise ValueError(self.resolve_error)
        local_probe = compose_poses(inverse_pose(self.frame_pose), self.nominal_probe)
        return ResolvedProbeTarget(stamped(local_probe, "flat_body"), self.resolved_sensor_id)

    def lookup_transform(self, target_frame, source_frame, **_kwargs):
        assert (target_frame, source_frame) == ("odom", "flat_body")
        if self.transform_error:
            raise ValueError(self.transform_error)
        result = TransformStamped()
        result.header.frame_id = target_frame
        result.child_frame_id = source_frame
        for axis in ("x", "y", "z"):
            setattr(result.transform.translation, axis, getattr(self.frame_pose.position, axis))
        for axis in ("x", "y", "z", "w"):
            setattr(result.transform.rotation, axis, getattr(self.frame_pose.orientation, axis))
        return result

    def hand_to_probe_pose(self, sensor_id):
        assert sensor_id == "probe"
        return pose_data_to_pose(self.calibration)


def harness(*, nominal_hand=None, visible=False, precision=False, tolerance=0.005,
            orientation_tolerance=math.radians(2.0)):
    nominal_hand = nominal_hand or pose(x=0.012)
    clock = Clock()
    executor = ReturnExecutor(clock)
    executor.motion_errors = {}
    executor.tag_state_source = object()
    planner = AnchorPlanner(clock, executor.hand_to_probe, nominal_hand)
    executor.probe_motion_planner = planner
    source = Source(executor, clock, visible)
    execution = MeasureSensorTipDistanceExecution(
        executor, source, MeasureSensorTipDistanceConfig(), monotonic_clock=clock,
    )
    target = ManipulatorToTagCommand(
        CommandID.MOVE_ARM_TO_TAG, Time(), PoseStamped(), 7,
        motion_sensor_id="probe", precision_mode=precision,
        tag_position_tolerance_m=tolerance,
        precision_orientation_tolerance_rad=orientation_tolerance,
    )
    command = MeasureSensorTipDistanceCommand(
        CommandID.MEASURE_SENSOR_TIP_DISTANCE, Time(), expected_distance_m=0.25, return_target=target,
    )
    return SimpleNamespace(clock=clock, executor=executor, source=source, execution=execution,
                           command=command, planner=planner, nominal_hand=nominal_hand)


def assert_pose(actual, expected):
    for axis in ("x", "y", "z"):
        assert getattr(actual.position, axis) == pytest.approx(getattr(expected.position, axis), abs=1e-10)
    assert rotation_distance_rad(actual.orientation, expected.orientation) < 1e-7


def test_saved_nominal_probe_anchors_sampling_and_calibrated_return_but_excursion_starts_at_actual_hand():
    nominal_hand = pose(x=0.012, orientation=quaternion_from_euler("y", math.radians(1.5)))
    case = harness(nominal_hand=nominal_hand)
    case.executor.motion_errors[2] = pose(x=0.003)
    nominal_probe = deepcopy(case.planner.nominal_probe)
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.executor.calls) == 2
    outward, returning = case.executor.calls
    assert_pose(pose_to_pose_data(outward[0]), pose(y=0.10))
    assert_pose(pose_to_pose_data(returning[0]), nominal_hand)
    assert all(call[0].header.frame_id == "odom" for call in case.executor.calls)
    assert_pose(case.source.view_calls[0][0], nominal_probe)
    assert_pose(case.source.sample_calls[0][0], nominal_probe)
    assert case.execution.result.distance_m == pytest.approx(0.25 - 0.003 * math.cos(math.radians(1.5)))
    assert_pose(case.execution.result.returned_probe_pose, case.planner.tip)
    assert len(case.planner.resolve_calls) == 1
    assert case.source.view_calls[0][1] == 0.25
    assert case.source.sample_calls[0][1] == 0.25


def test_tag_update_after_acquisition_cannot_move_the_frozen_return_goal():
    case = harness()
    nominal = deepcopy(case.nominal_hand)
    outcome = begin_excursion(case)
    case.planner.nominal_probe.position.x += 0.20
    assert finish(case, outcome) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.planner.resolve_calls) == 1
    assert_pose(pose_to_pose_data(case.executor.calls[-1][0]), nominal)
    assert case.execution.result.distance_m == pytest.approx(0.25)


@pytest.mark.parametrize("nominal_x,precision,tolerance,moves", [
    (0.008, False, 0.005, 0),
    (0.012, False, 0.005, 1),
    (0.008, True, 0.005, 1),
    (0.012, True, 0.020, 1),
])
def test_visible_saved_target_only_moves_when_selected_effective_tolerance_requires_it(
    nominal_x, precision, tolerance, moves,
):
    case = harness(nominal_hand=pose(x=nominal_x), visible=True,
                   precision=precision, tolerance=tolerance)
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.executor.calls) == moves
    assert case.execution.result.distance_m == pytest.approx(0.25 if moves else 0.25 + nominal_x)


def test_precision_can_tighten_saved_return_orientation_without_relaxing_position():
    case = harness(nominal_hand=pose(orientation=quaternion_from_euler("y", math.radians(1.0))),
                   visible=True, precision=True, orientation_tolerance=math.radians(0.5))
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert len(case.executor.calls) == 1
    assert_pose(case.planner.hand, case.nominal_hand)


@pytest.mark.parametrize("nominal", [
    pose(x=0.021), pose(orientation=quaternion_from_euler("y", math.radians(5.1))),
])
def test_distant_saved_target_is_rejected_before_observation_or_return_motion(nominal):
    case = harness(nominal_hand=nominal)
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.executor.calls == []
    assert case.source.view_calls == case.source.sample_calls == []
    assert "outside the bounded observation-return range" in case.execution.feedback_message


@pytest.mark.parametrize("unavailable", ["attachment", "tag", "transform"])
def test_invalid_nominal_target_reference_never_starts_motion(unavailable):
    case = harness()
    if unavailable == "attachment":
        case.planner.resolved_sensor_id = "different_probe"
    elif unavailable == "tag":
        case.planner.resolve_error = "Tag 7 is not currently usable"
    else:
        case.planner.transform_error = "Nominal target transform unavailable"
    assert finish(case) is MeasureSensorTipDistanceOutcome.FAILURE
    assert case.executor.calls == []
    assert case.execution.result is None


def test_standalone_measurement_does_not_resolve_or_follow_a_saved_tag_target():
    case = harness()
    case.command.return_target = None
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    assert case.planner.resolve_calls == []
    assert_pose(pose_to_pose_data(case.executor.calls[-1][0]), pose())
    assert case.execution.result.distance_m == pytest.approx(0.25)
