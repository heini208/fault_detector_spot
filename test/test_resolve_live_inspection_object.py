"""Tests for the mapless behavior-tree object resolver."""

from copy import deepcopy

import py_trees
import tf2_ros
from fault_detector_msgs.msg import TagElement
from rclpy.time import Time

from fault_detector_spot.behaviour_tree.nodes.inspection import (
    resolve_live_inspection_object,
)
from fault_detector_spot.inspection.inspection_repository import (
    InspectionRepository,
)
from fault_detector_spot.inspection.models import (
    InspectionDefinition,
    ObjectDefinition,
    PoseData,
)
from fault_detector_spot.inspection.object_pose_resolver import (
    ObjectPoseState,
)
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)

ResolveLiveInspectionObject = (
    resolve_live_inspection_object.ResolveLiveInspectionObject
)


class FakeClock:
    """Provide a fixed ROS time."""

    def now(self):
        """Return the current test time."""
        return Time(seconds=10.2)


class FakeNode:
    """Provide the clock required by the behavior."""

    def get_clock(self):
        """Return the fake clock."""
        return FakeClock()


class FakeTfBuffer:
    """Transform body poses into odom for tests."""

    def __init__(self, available=True):
        self.available = available

    def transform(self, pose, target_frame, timeout=None):
        """Return a transformed pose or emulate missing TF."""
        if not self.available:
            raise tf2_ros.LookupException("missing transform")

        transformed = deepcopy(pose)
        transformed.header.frame_id = target_frame
        transformed.pose.position.x += 2.0
        return transformed


def make_tag(stamp_sec=10) -> TagElement:
    """Create a fresh body-frame base tag."""
    tag = TagElement()
    tag.id = 7
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = stamp_sec
    tag.pose.header.stamp.nanosec = 100_000_000
    tag.pose.pose.position.x = 1.0
    tag.pose.pose.orientation.w = 1.0
    return tag


def make_behavior(tf_available=True):
    """Create a configured behavior without real ROS resources."""
    behavior = ResolveLiveInspectionObject(
        object_id="panel",
        inspection_id="phase3",
    )
    behavior.node = FakeNode()
    behavior.tf_buffer = FakeTfBuffer(tf_available)
    behavior.object_definition = ObjectDefinition(
        object_id="panel",
        display_name="Panel",
        tag_id=7,
        marker_to_object=PoseData(),
        calibration_revision=1,
    )
    behavior.inspection_definition = InspectionDefinition(
        inspection_id="phase3",
        object_id="panel",
        object_calibration_revision=1,
    )
    behavior.configuration_error = ""
    behavior.blackboard.register_key(
        "base_tag_observations",
        access=py_trees.common.Access.READ,
    )
    behavior.blackboard.register_key(
        "live_inspection_object",
        access=py_trees.common.Access.WRITE,
    )
    behavior.blackboard.register_key(
        "live_inspection_object_error",
        access=py_trees.common.Access.WRITE,
    )
    return behavior


def create_writer():
    """Create camera-source blackboard inputs."""
    writer = py_trees.blackboard.Client(
        name="LiveObjectTestWriter"
    )
    writer.register_key(
        "base_tag_observations",
        access=py_trees.common.Access.WRITE,
    )
    writer.register_key(
        "hand_tag_observations",
        access=py_trees.common.Access.WRITE,
    )
    return writer


def setup_function():
    """Clear blackboard state before each test."""
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    """Clear blackboard state after each test."""
    py_trees.blackboard.Blackboard.clear()


def test_base_tag_produces_live_odom_object():
    """A fresh base tag is transformed and resolved in odom."""
    writer = create_writer()
    writer.base_tag_observations = {7: make_tag()}
    writer.hand_tag_observations = {}
    behavior = make_behavior()

    status = behavior.update()
    result = behavior.blackboard.live_inspection_object

    assert status == py_trees.common.Status.SUCCESS
    assert result.state == ObjectPoseState.LIVE
    assert result.selected_pose.header.frame_id == "odom"
    assert result.selected_pose.pose.position.x == 3.0
    assert result.observation_source == "base"


def test_hand_only_tag_is_ignored():
    """A hand observation cannot create a local object pose."""
    writer = create_writer()
    writer.base_tag_observations = {}
    writer.hand_tag_observations = {7: make_tag()}
    behavior = make_behavior()

    behavior.update()
    result = behavior.blackboard.live_inspection_object

    assert result.state == ObjectPoseState.UNAVAILABLE
    assert result.selected_pose is None


def test_missing_odom_transform_is_unavailable():
    """Missing local TF does not fall back to body or map."""
    writer = create_writer()
    writer.base_tag_observations = {7: make_tag()}
    writer.hand_tag_observations = {}
    behavior = make_behavior(tf_available=False)

    behavior.update()
    result = behavior.blackboard.live_inspection_object

    assert result.state == ObjectPoseState.UNAVAILABLE
    assert "Cannot transform" in result.message


def test_calibration_revision_mismatch_rejects_configuration(
    tmp_path,
):
    """An inspection cannot run after object recalibration."""
    object_root = tmp_path / "objects"
    inspection_root = tmp_path / "inspections"
    ObjectRepository(object_root).save(
        ObjectDefinition(
            object_id="panel",
            display_name="Panel",
            tag_id=7,
            calibration_revision=2,
        )
    )
    InspectionRepository(inspection_root).save(
        InspectionDefinition(
            inspection_id="phase3",
            object_id="panel",
            object_calibration_revision=1,
        )
    )
    behavior = ResolveLiveInspectionObject(
        object_id="panel",
        inspection_id="phase3",
        object_root=object_root,
        inspection_root=inspection_root,
    )

    behavior._load_configuration()

    assert (
        behavior.configuration_error
        == "Inspection object calibration revision 1 does not match "
        "current object calibration revision 2"
    )
