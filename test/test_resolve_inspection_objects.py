"""Tests for runtime inspection object resolution."""

import py_trees
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.behaviour_tree.nodes.sensing.resolve_inspection_objects import (
    ResolveInspectionObjects,
)
from fault_detector_spot.inspection.map_repository import (
    MapRepository,
)
from fault_detector_spot.inspection.models import (
    InspectionObject,
    LandmarkDefinition,
    MapDefinition,
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.object_pose_resolver import (
    ObjectPoseState,
)
from fault_detector_spot.inspection.tag_element_adapter import (
    tag_elements_to_pose_stamped,
)


class FakeLogger:
    """Minimal logger used by behavior tests."""

    def warning(self, message):
        """Accept warning messages."""
        return message


class FakeNode:
    """Minimal ROS node replacement."""

    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        """Return the fake logger."""
        return self.logger


class FakeSlamHelper:
    """Minimal SLAM helper replacement."""

    def __init__(self, maps_dir):
        self.maps_dir = str(maps_dir)


def create_map_definition():
    """Create one remembered inspection object."""
    return MapDefinition(
        landmarks=[
            LandmarkDefinition(
                name="Tag_23",
                tag_id=23,
                pose=PoseData(
                    position=Vector3Data(
                        x=1.0,
                        y=2.0,
                        z=0.0,
                    ),
                    orientation=QuaternionData(),
                ),
            )
        ],
        objects=[
            InspectionObject(
                object_id="motor_a",
                display_name="Motor A",
                tag_id=23,
                landmark_name="Tag_23",
            )
        ],
    )


def create_blackboard_writer():
    """Create a writer for behavior input keys."""
    writer = py_trees.blackboard.Client(
        name="InspectionTestWriter"
    )

    writer.register_key(
        "active_map_name",
        access=py_trees.common.Access.WRITE,
    )
    writer.register_key(
        "visible_tags_map_frame",
        access=py_trees.common.Access.WRITE,
    )

    return writer


def setup_function():
    """Reset global blackboard state before each test."""
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    """Reset global blackboard state after each test."""
    py_trees.blackboard.Blackboard.clear()


def test_tag_element_adapter():
    """TagElement poses are converted by tag ID."""
    tag = TagElement()
    tag.id = 23
    tag.pose.header.frame_id = "map"
    tag.pose.pose.orientation.w = 1.0
    tag.pose.pose.position.x = 4.0

    result = tag_elements_to_pose_stamped({
        23: tag,
    })

    assert set(result.keys()) == {23}
    assert result[23].header.frame_id == "map"
    assert result[23].pose.position.x == 4.0


def test_behavior_resolves_remembered_and_live(
    tmp_path,
):
    """Behavior selects remembered then live poses."""
    repository = MapRepository(tmp_path)
    repository.save(
        "laboratory",
        create_map_definition(),
    )

    writer = create_blackboard_writer()
    writer.active_map_name = "laboratory"
    writer.visible_tags_map_frame = {}

    behavior = ResolveInspectionObjects(
        FakeSlamHelper(tmp_path)
    )
    behavior.setup(node=FakeNode())

    status = behavior.update()

    assert status == py_trees.common.Status.SUCCESS

    remembered = (
        behavior.blackboard.inspection_objects[
            "motor_a"
        ]
    )

    assert (
        remembered.state
        == ObjectPoseState.REMEMBERED
    )
    assert (
        remembered.selected_pose.pose.position.x
        == 1.0
    )

    tag = TagElement()
    tag.id = 23
    tag.pose.header.frame_id = "map"
    tag.pose.pose.orientation.w = 1.0
    tag.pose.pose.position.x = 4.0
    tag.pose.pose.position.y = 5.0

    writer.visible_tags_map_frame = {
        23: tag,
    }

    status = behavior.update()

    assert status == py_trees.common.Status.SUCCESS

    live = (
        behavior.blackboard.inspection_objects[
            "motor_a"
        ]
    )

    assert live.state == ObjectPoseState.LIVE
    assert live.selected_pose.pose.position.x == 4.0
    assert live.selected_pose.pose.position.y == 5.0


def test_behavior_clears_without_active_map(
    tmp_path,
):
    """No active map produces no object states."""
    writer = create_blackboard_writer()
    writer.active_map_name = None
    writer.visible_tags_map_frame = {}

    behavior = ResolveInspectionObjects(
        FakeSlamHelper(tmp_path)
    )
    behavior.setup(node=FakeNode())

    status = behavior.update()

    assert status == py_trees.common.Status.SUCCESS
    assert behavior.blackboard.inspection_objects == {}
    assert behavior.blackboard.inspection_object_error == ""