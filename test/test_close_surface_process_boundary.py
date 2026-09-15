"""Lock close-surface execution into the shared arm movement boundary."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_launch_has_no_dedicated_close_surface_process():
    launch = read("launch/fault_detector_launch.py")
    setup = read("setup.py")

    assert 'executable="move_close_to_surface_node"' not in launch
    assert "manipulation.move_close_to_surface_node:main" not in setup


def test_close_surface_is_one_bt_behaviour_over_shared_executor():
    behaviour = read(
        "fault_detector_spot/manipulation/behaviours/"
        "move_close_to_surface_behaviour.py"
    )

    assert "class MoveCloseToSurfaceBehaviour(ArmMovementBehaviour)" in behaviour
    assert "guarded_probe(" in behaviour
    assert ".probe(" in behaviour
    assert "ArmMovementOutcome.CONTACT" in behaviour
    assert "MoveCloseToSurfaceOperation" not in behaviour
    assert "WorkflowActionBehaviour" not in behaviour
    assert "RobotCommandBuilder" not in behaviour
    assert "send_goal_async" not in behaviour
    assert "cancel_goal_async" not in behaviour
    assert "end_effector_force" not in behaviour


def test_probe_surface_source_only_owns_surface_sensing_and_attachment():
    source = read(
        "fault_detector_spot/inspection/sensing/probe_surface_source.py"
    )

    assert "class ProbeSurfaceSource(RuntimeSource)" in source
    assert "surface_distance_samples" in source
    assert "active_attachment" in source
    assert "END_EFFECTOR_FORCE_TOPIC" not in source
    assert "Vector3Stamped" not in source
    assert "current_hand_pose_execution" not in source
    assert "current_probe_pose_execution" not in source


def test_close_surface_uses_executor_planner_for_live_arm_geometry():
    behaviour = read(
        "fault_detector_spot/manipulation/behaviours/"
        "move_close_to_surface_behaviour.py"
    )

    assert "probe_motion_planner.current_pose(" in behaviour
    assert "probe_motion_planner.current_hand_pose(" in behaviour
