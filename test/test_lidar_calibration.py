"""SDK calibration must use the physical sensor, not the world-frame cloud."""

import importlib.util
from pathlib import Path

from bosdyn.api import point_cloud_pb2
import pytest


spec = importlib.util.spec_from_file_location(
    "read_spot_lidar_calibration", Path(__file__).parents[1] / "scripts/read_spot_lidar_calibration.py",
)
calibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibration)


def lidar_response():
    response = point_cloud_pb2.PointCloudResponse()
    response.point_cloud.source.frame_name_sensor = "sensor_origin_velodyne-point-cloud"
    edges = response.point_cloud.source.transforms_snapshot.child_to_parent_edge_map
    edges["odom"].parent_frame_name = ""
    for child, parent in (("body", "odom"), ("sensor", "body"),
                          ("sensor_origin_velodyne-point-cloud", "odom")):
        edges[child].parent_frame_name = parent
        edges[child].parent_tform_child.rotation.w = 1.0
    edges["body"].parent_tform_child.position.x = 12.0
    pose = edges["sensor"].parent_tform_child
    pose.position.x, pose.position.z = -0.2, 0.3
    pose.rotation.z, pose.rotation.w = 0.6, 0.8
    return response


def test_export_uses_physical_body_to_sensor_pose_and_preserves_orientation():
    parameters = calibration.calibration_parameters(lidar_response())
    assert parameters["frame_id"] == "body"
    assert parameters["child_frame_id"] == "moveit_lidar_sensor"
    assert [parameters[f"translation.{a}"] for a in "xyz"] == pytest.approx([-0.2, 0., 0.3])
    assert [parameters[f"rotation.{a}"] for a in "xyzw"] == pytest.approx([0., 0., 0.6, 0.8])


def test_cloud_origin_is_not_a_fallback_when_physical_metadata_is_missing():
    response = lidar_response()
    del response.point_cloud.source.transforms_snapshot.child_to_parent_edge_map["sensor"]
    with pytest.raises(ValueError, match="physical sensor calibration"):
        calibration.calibration_parameters(response)


@pytest.mark.parametrize("invalid", [float("nan"), float("inf")])
def test_nonfinite_mount_is_not_exported(invalid):
    response = lidar_response()
    response.point_cloud.source.transforms_snapshot.child_to_parent_edge_map[
        "sensor"
    ].parent_tform_child.position.x = invalid
    with pytest.raises(ValueError, match="non-finite"):
        calibration.calibration_parameters(response)
