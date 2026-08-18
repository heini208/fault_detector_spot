"""Validate base-fiducial freshness configuration."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_tag_sensing_configuration_has_no_leading_overlay_artifact():
    config = (ROOT / "config/tag_sensing.yaml").read_text(encoding="utf-8")

    assert config.startswith("/**:")
    assert not config.startswith("\\")


def test_tag_sensing_uses_raw_fiducial_frame_for_freshness():
    config = (ROOT / "config/tag_sensing.yaml").read_text(encoding="utf-8")

    assert (
        "tag_sensing.base_frame_pattern: "
        "'(?<!filtered_)fiducial_(\\d+)'"
    ) in config
    assert (
        "tag_sensing.base_frame_pattern: "
        "'filtered_fiducial_(\\d+)'"
    ) not in config


def test_probe_and_base_tag_freshness_windows_are_aligned():
    config = (ROOT / "config/tag_sensing.yaml").read_text(encoding="utf-8")

    assert "tag_sensing.base_max_age_sec: 1.5" in config
    assert "tag_sensing.probe_max_age_sec: 1.5" in config


def test_tag_node_defaults_match_tag_sensing_configuration():
    tag_node = (
        ROOT
        / "fault_detector_spot"
        / "sensing"
        / "tag_observation_node.py"
    ).read_text(encoding="utf-8")

    assert 'r"(?<!filtered_)fiducial_(\\d+)"' in tag_node
    assert (
        'self._parameter("tag_sensing.base_max_age_sec", 1.5)'
    ) in tag_node
    assert (
        'self._parameter("tag_sensing.publish_period_sec", 0.05)'
    ) in tag_node


def test_bt_runner_uses_only_the_tag_state_health_timeout():
    runner = (
        ROOT
        / "fault_detector_spot"
        / "application"
        / "behaviour_tree"
        / "runner.py"
    ).read_text(encoding="utf-8")

    assert '"tag_sensing.state_timeout_sec"' in runner
    assert '"tag_sensing.base_max_age_sec"' not in runner
    assert '"tag_sensing.hand_max_age_sec"' not in runner
    assert '"tag_sensing.hand_tf_pending_sec"' not in runner
