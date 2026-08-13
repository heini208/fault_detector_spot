\
"""Validate raw base-fiducial freshness configuration."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


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
