"""Lock AprilTag observation and reachability ownership outside the BT."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_launch_starts_dedicated_tag_observation_process():
    launch = read("launch/fault_detector_launch.py")
    setup = read("setup.py")

    assert 'executable="tag_observation_node"' in launch
    assert "fault_detector_spot.sensing.tag_observation_node:main" in setup


def test_behavior_tree_only_consumes_authoritative_tag_snapshots():
    runner = read(
        "fault_detector_spot/application/behaviour_tree/runner.py"
    )

    assert "TagStateSubscriber" in runner
    assert "DetectVisibleTags" not in runner
    assert "HandCameraTagDetection" not in runner
    assert "PublishTagStates" not in runner
    assert "CheckTagReachability" not in runner
    assert "PublishReachableTags" not in runner


def test_tag_process_owns_one_tf_listener_and_all_tag_publishers():
    node = read("fault_detector_spot/sensing/tag_observation_node.py")

    assert node.count("tf2_ros.TransformListener(") == 1
    assert '"fault_detector/state/base_tags"' in node
    assert '"fault_detector/state/visible_tags"' in node
    assert '"fault_detector/state/reachable_tags"' in node
    assert "TagReachabilityFilter" in node
    assert "merge_tag_observations" in node


def test_bt_subscriber_has_no_tf_detection_or_reachability_logic():
    subscriber = read(
        "fault_detector_spot/sensing/behaviours/"
        "tag_state_subscriber.py"
    )

    assert "tf2_ros" not in subscriber
    assert "AprilTagDetection" not in subscriber
    assert "TagReachabilityFilter" not in subscriber
    assert "base_tag_observations" in subscriber
    assert "visible_tags" in subscriber
    assert "reachable_tags" in subscriber
    assert "time.monotonic()" in subscriber
