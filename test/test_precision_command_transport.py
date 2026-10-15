"""Optional endpoint precision survives command boundaries and each path leg."""

from dataclasses import replace
import math

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import OperationalIntent
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand, SemanticTag, StampedPose,
)
from fault_detector_spot.application.recording.semantic_command_codec import (
    deserialize_recorded_command, serialize_recorded_command,
)
from fault_detector_spot.application.ros.operational_intent_adapter import operational_intent_to_command
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message, semantic_command_to_message,
)
from fault_detector_spot.manipulation.commands.manipulator_move_relative_command import ManipulatorMoveRelativeCommand
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import ManipulatorToTagCommand
from fault_detector_spot.manipulation.commands.measure_sensor_tip_distance_command import MeasureSensorTipDistanceCommand
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import MoveCloseToSurfaceCommand
from fault_detector_spot.shared.geometry.movement_frames import OrientationModes
from test_pre_approach_path_transport import executable_path


def tag_command():
    return SemanticCommand(
        command_id=CommandID.MOVE_ARM_TO_TAG,
        tag=SemanticTag(7, StampedPose(frame_id="odom")),
        offset=StampedPose(frame_id="filtered_fiducial_7"),
        orientation_mode=OrientationModes.CUSTOM_ORIENTATION.value,
        motion_sensor_id="probe",
        tag_position_tolerance_m=0.006,
        precision_mode=True,
        precision_orientation_tolerance_rad=math.radians(1.5),
    )


def test_precision_survives_wire_recording_and_individual_path_execution():
    command = replace(
        tag_command(), command_id=CommandID.FOLLOW_MOVE_TO_TAG_PATH,
        pre_approach_offsets=(StampedPose(frame_id="odom"), StampedPose(frame_id="odom")),
        pre_approach_precision_modes=(False, True),
        pre_approach_tolerances_m=(0.020, 0.008),
    )
    wire = semantic_command_to_message(command)
    assert wire.precision_mode is True
    assert list(wire.pre_approach_precision_modes) == [False, True]
    assert semantic_command_from_message(wire) == command
    restored = deserialize_recorded_command(serialize_recorded_command(command))
    restored = replace(restored, motion_sensor_id=command.motion_sensor_id)
    assert restored == command
    steps = executable_path(restored)
    assert [step.precision_mode for step in steps] == [False, True, True]
    assert [step.tag_position_tolerance_m for step in steps] == [0.020, 0.008, 0.006]
    assert all(step.precision_orientation_tolerance_rad == math.radians(1.5) for step in steps)


def test_missing_path_precision_values_use_explicit_command_policy():
    command = replace(
        tag_command(), command_id=CommandID.FOLLOW_MOVE_TO_TAG_PATH,
        pre_approach_offsets=(StampedPose(frame_id="odom"),),
    )
    assert [step.precision_mode for step in executable_path(command)] == [True, True]


def test_old_recordings_and_ordinary_commands_default_to_existing_motion_behavior():
    recorded = serialize_recorded_command(tag_command())
    for name in ("precision_mode", "precision_orientation_tolerance_rad", "pre_approach_precision_modes"):
        recorded.pop(name)
    restored = deserialize_recorded_command(recorded)
    assert restored.precision_mode is False
    assert restored.pre_approach_precision_modes == ()
    assert restored.precision_orientation_tolerance_rad == math.radians(2.0)
    assert SemanticCommand(CommandID.MOVE_ARM_RELATIVE).precision_mode is False
    relative = ManipulatorMoveRelativeCommand(CommandID.MOVE_ARM_RELATIVE, Time())
    tag = ManipulatorToTagCommand(CommandID.MOVE_ARM_TO_TAG, Time(), PoseStamped(), 7)
    assert relative.precision_mode is tag.precision_mode is False


@pytest.mark.parametrize("operation", [
    OperationalIntent.INTENT_MOVE_ARM_TO_TAG, OperationalIntent.INTENT_MOVE_ARM_RELATIVE,
])
def test_public_precision_intent_reaches_executable_tag_or_relative_command(operation):
    intent = OperationalIntent()
    intent.intent = operation
    intent.tag.id = 7
    intent.tag.pose.header.frame_id = "odom"
    intent.offset.header.frame_id = "hand"
    intent.orientation_mode = OrientationModes.CUSTOM_ORIENTATION.value
    intent.precision_mode = True
    intent.tag_position_tolerance_m = 0.004
    intent.precision_orientation_tolerance_rad = math.radians(1.0)
    semantic = replace(operational_intent_to_command(intent), motion_sensor_id="probe")
    steps = executable_path(semantic_command_from_message(semantic_command_to_message(semantic)))
    assert len(steps) == 1
    assert steps[0].precision_mode is True
    assert steps[0].tag_position_tolerance_m == 0.004
    assert steps[0].precision_orientation_tolerance_rad == math.radians(1.0)
    assert isinstance(steps[0], ManipulatorToTagCommand if operation ==
                      OperationalIntent.INTENT_MOVE_ARM_TO_TAG else ManipulatorMoveRelativeCommand)


@pytest.mark.parametrize("value", [1, "true", None])
def test_precision_requires_a_boolean_policy(value):
    with pytest.raises(TypeError, match="Precision mode"):
        replace(tag_command(), precision_mode=value)


@pytest.mark.parametrize("value", [0.0, -0.1, float("nan"), float("inf")])
def test_precision_orientation_tolerance_must_be_positive_finite(value):
    with pytest.raises(ValueError, match="Precision orientation tolerance"):
        replace(tag_command(), precision_orientation_tolerance_rad=value)


@pytest.mark.parametrize("values", [[True], (1,), (True, False)])
def test_path_precision_values_are_boolean_and_match_path_length(values):
    with pytest.raises((TypeError, ValueError), match="Path precision modes"):
        replace(tag_command(), command_id=CommandID.FOLLOW_MOVE_TO_TAG_PATH,
                pre_approach_offsets=(StampedPose(frame_id="odom"),),
                pre_approach_precision_modes=values)


@pytest.mark.parametrize("operation", [CommandID.MOVE_CLOSE_TO_SURFACE, CommandID.MEASURE_SENSOR_TIP_DISTANCE])
def test_optional_observation_return_target_uses_existing_tag_and_offset_transport(operation):
    command = replace(tag_command(), command_id=operation,
                      target_surface_distance_m=0.03, aligned_preapproach_distance_m=0.25)
    restored = semantic_command_from_message(semantic_command_to_message(command))
    movement = executable_path(restored)[0]
    target = movement.return_target
    assert isinstance(target, ManipulatorToTagCommand)
    assert target.command_id == CommandID.MOVE_ARM_TO_TAG
    assert target.tag_id == command.tag.id
    assert target.offset.header.frame_id == command.offset.frame_id
    assert target.motion_sensor_id == "probe"
    assert target.precision_mode is True
    assert target.tag_position_tolerance_m == 0.006
    assert movement.expected_distance_m == 0.25


@pytest.mark.parametrize("operation", [CommandID.MOVE_CLOSE_TO_SURFACE, CommandID.MEASURE_SENSOR_TIP_DISTANCE])
def test_unanchored_measurement_has_no_return_target(operation):
    command = SemanticCommand(command_id=operation, target_surface_distance_m=0.03)
    assert executable_path(command)[0].return_target is None


def test_setup_close_intent_preserves_optional_saved_object_pose_as_return_target():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
    intent.tag.id = 7
    intent.offset.header.frame_id = "filtered_fiducial_7"
    intent.offset.pose.position.z = 0.15
    intent.offset.pose.orientation.w = 1.0
    intent.orientation_mode = OrientationModes.CUSTOM_ORIENTATION.value
    intent.target_surface_distance_m = 0.03
    intent.precision_mode = True
    command = replace(operational_intent_to_command(intent), motion_sensor_id="probe")
    target = executable_path(command)[0].return_target
    assert target.tag_id == 7
    assert target.offset.header.frame_id == "filtered_fiducial_7"
    assert target.offset.pose.position.z == 0.15
    assert target.orientation_mode == OrientationModes.CUSTOM_ORIENTATION.value
    assert target.precision_mode is True


@pytest.mark.parametrize("frame", ["odom", "filtered_fiducial_8"])
def test_setup_close_anchor_rejects_wrong_tag_or_non_object_frame(frame):
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
    intent.tag.id = 7
    intent.offset.header.frame_id = frame
    with pytest.raises(ValueError, match="selected tag's object frame"):
        operational_intent_to_command(intent)


@pytest.mark.parametrize("command_type", [MoveCloseToSurfaceCommand, MeasureSensorTipDistanceCommand])
def test_observation_return_target_rejects_untyped_geometry(command_type):
    options = {"target_surface_distance_m": 0.03} if command_type is MoveCloseToSurfaceCommand else {}
    with pytest.raises(TypeError, match="tag-relative probe command"):
        command_type(CommandID.MEASURE_SENSOR_TIP_DISTANCE, Time(), return_target=PoseStamped(), **options)


@pytest.mark.parametrize("command_id", [CommandID.MOVE_ARM_TO_POSE, CommandID.MOVE_BASE_TO_POSE])
def test_absolute_retry_command_dispatch_preserves_target_and_settings(command_id):
    from fault_detector_spot.application.commanding.semantic_command import CommandQuaternion, CommandVector3
    from fault_detector_spot.application.behaviour_tree.commands.move_to_pose_command import MoveToPoseCommand

    command = SemanticCommand(
        command_id, offset=StampedPose(frame_id="odom", position=CommandVector3(x=2.0, y=-3.0),
                                      orientation=CommandQuaternion(w=1.0)),
        precision_mode=True, arm_speed_scale=0.4, walking_profile="precision",
        ignore_environment_collisions=True,
    )
    restored = semantic_command_from_message(semantic_command_to_message(command))
    execution, = executable_path(restored)
    assert isinstance(execution, MoveToPoseCommand)
    assert execution.semantic_command == command
    assert execution.offset.header.frame_id == "odom"
    assert execution.offset.pose.position.x == 2.0
    assert execution.offset.pose.position.y == -3.0
    assert execution.arm_speed_scale == 0.4
    assert execution.precision_mode
    assert execution.walking_profile == "precision"
    assert execution.ignore_environment_collisions


@pytest.mark.parametrize("command_id", [CommandID.MOVE_ARM_TO_POSE, CommandID.MOVE_BASE_TO_POSE])
def test_absolute_retry_rejects_moving_frame_and_invalid_orientation(command_id):
    from fault_detector_spot.application.commanding.semantic_command import CommandQuaternion

    with pytest.raises(ValueError, match="odom"):
        SemanticCommand(command_id, offset=StampedPose(frame_id="body", orientation=CommandQuaternion(w=1)))
    with pytest.raises(ValueError, match="unit quaternion"):
        SemanticCommand(command_id, offset=StampedPose(frame_id="odom"))
