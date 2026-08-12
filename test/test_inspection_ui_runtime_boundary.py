"""Guard the inspection UI robot-runtime boundary."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
INSPECTION_UI = ROOT / "fault_detector_spot" / "ui" / "inspection"


def test_inspection_ui_does_not_own_robot_sensing_runtime():
    source = (INSPECTION_UI / "controls.py").read_text(encoding="utf-8")
    forbidden = (
        "tf2_ros",
        "qos_profile_sensor_data",
        "sensor_msgs.msg",
        "/depth_registered/hand/image",
        "/depth_registered/hand/camera_info",
        "measure_probe_surface_distance",
        "aggregate_surface_distance_samples",
        "stabilize_tag_pose",
        "TagPoseSample",
        "_tf_buffer",
        "_tf_listener",
        "_hand_depth_history",
        "_base_tag_histories",
        "_process_live_hand_depth",
        "_measure_live_surface_distance",
        "_lookup_pose",
        "_stable_reference_tag",
        "_live_reference_tag",
        "handle_base_tags",
    )

    for value in forbidden:
        assert value not in source

    assert "execute_probe_surface_verification" in source
    assert "ProbeSetupMotionIntent" in source


def test_sensor_registry_transport_is_defined_once():
    base = (INSPECTION_UI / "controls.py").read_text(encoding="utf-8")
    finalizing = (
        INSPECTION_UI / "finalizing_controls.py"
    ).read_text(encoding="utf-8")

    assert "fault_detector/sensors" in base
    assert "AddSensor" in base
    assert "RetireSensor" in base
    assert "def init_ros_communication" not in finalizing
