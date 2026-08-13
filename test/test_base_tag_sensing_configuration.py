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


def test_runner_defaults_match_tag_sensing_configuration():
    runner = (
        ROOT
        / "fault_detector_spot"
        / "application"
        / "behaviour_tree"
        / "runner.py"
    ).read_text(encoding="utf-8")

    assert 'r"(?<!filtered_)fiducial_(\\d+)"' in runner
    assert (
        '"tag_sensing.probe_max_age_sec",\n'
        '            1.5,'
    ) in runner
