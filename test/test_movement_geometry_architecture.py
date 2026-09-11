"""Lock live movement geometry below the behavior-tree adapter."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_geometry_waiting_is_executor_state_not_behavior_state():
    parent = read(
        "fault_detector_spot/shared/execution/movement_executor.py"
    )
    geometry = read(
        "fault_detector_spot/shared/geometry/movement_geometry.py"
    )
    behaviour = read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "movement_behaviour.py"
    )

    assert "MovementGeometryUnavailable" in geometry
    assert "except MovementGeometryUnavailable" in parent
    assert "_pending_goal_builder" in parent
    assert "self._pending_goal_builder" in parent
    assert "MovementGeometryResolver" not in behaviour
    assert "_prepare_operation" not in behaviour
    assert "rclpy" not in behaviour
    assert "PoseStamped" not in behaviour


def test_geometry_resolver_owns_tag_alias_and_offset_resolution():
    geometry = read(
        "fault_detector_spot/shared/geometry/movement_geometry.py"
    )

    assert "def prepare_move_command(" in geometry
    assert "def resolve_and_transform_offset_if_tag(" in geometry
    assert "def resolve_tag_alias(" in geometry
    assert "def can_transform(" in geometry
    assert "command.offset = transformed" in geometry
