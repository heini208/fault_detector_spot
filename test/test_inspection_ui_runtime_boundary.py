"""Guard the inspection UI robot-runtime boundary."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
INSPECTION_UI = ROOT / "fault_detector_spot" / "ui" / "inspection"
MAIN_UI = ROOT / "fault_detector_spot" / "ui" / "fault_detector_ui.py"


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


def test_inspection_ui_does_not_consume_sensor_registry_state():
    base = (INSPECTION_UI / "controls.py").read_text(encoding="utf-8")
    finalizing = (
        INSPECTION_UI / "finalizing_controls.py"
    ).read_text(encoding="utf-8")
    main = MAIN_UI.read_text(encoding="utf-8")

    forbidden = (
        "fault_detector/sensors",
        "fault_detector/add_sensor",
        "fault_detector/retire_sensor",
        "AddSensor",
        "RetireSensor",
        "SensorDefinitionArray",
        "sensor_add_client",
        "sensor_retire_client",
        "sensor_list_subscription",
        "handle_add_sensor",
        "handle_retire_sensor",
        "def set_sensor_definitions",
        "sensor_id_field",
        "probe_frame_value_label",
    )
    for value in forbidden:
        assert value not in base

    assert "self.inspection_controls.set_sensor_definitions(" not in main
    assert "def init_ros_communication" not in finalizing
