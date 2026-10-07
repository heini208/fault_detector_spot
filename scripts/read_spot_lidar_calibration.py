#!/usr/bin/env python3
"""Save physical lidar calibration from the SDK without changing the driver."""

import argparse
from pathlib import Path

import yaml
from bosdyn.client import create_standard_sdk
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.client.point_cloud import build_pc_request

from fault_detector_spot.shared.geometry.models import PoseData, QuaternionData, Vector3Data


def calibration_parameters(response):
    """Extract the physical sensor edge, never substitute the cloud's origin."""
    snapshot = response.point_cloud.source.transforms_snapshot
    if "sensor" not in snapshot.child_to_parent_edge_map:
        raise ValueError("The lidar response does not contain physical sensor calibration")
    mount = get_a_tform_b(snapshot, "body", "sensor")
    if mount is None:
        raise ValueError("The lidar response has no body-to-sensor transform")
    pose = PoseData(
        Vector3Data(mount.x, mount.y, mount.z),
        QuaternionData(mount.rot.x, mount.rot.y, mount.rot.z, mount.rot.w),
    )
    pose.validate()
    return {
        "frame_id": "body",
        "child_frame_id": "moveit_lidar_sensor",
        **{f"translation.{axis}": getattr(pose.position, axis) for axis in "xyz"},
        **{f"rotation.{axis}": getattr(pose.orientation, axis) for axis in "xyzw"},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", type=Path, required=True,
                        help="Existing Spot connection YAML with /** ros__parameters")
    parser.add_argument("--output", type=Path, required=True,
                        help="Calibration YAML to write; refresh after changing the lidar mount")
    args = parser.parse_args()
    try:
        credentials = yaml.safe_load(args.robot_config.read_text())["/**"]["ros__parameters"]
        robot = create_standard_sdk("fault-detector-lidar-calibration").create_robot(
            credentials["hostname"],
        )
        robot.authenticate(credentials["username"], credentials["password"], timeout=5)
        robot.sync_with_directory()
        client = robot.ensure_client("velodyne-point-cloud")
        response = client.get_point_cloud([build_pc_request("velodyne-point-cloud")], timeout=5)[0]
        parameters = calibration_parameters(response)
        document = {"/moveit_lidar_mount": {"ros__parameters": parameters}}
        args.output.write_text(
            "# Captured from this robot's SDK body-to-sensor calibration.\n"
            "# Refresh after changing the lidar mount or using another robot.\n"
            + yaml.safe_dump(document, sort_keys=False),
        )
    except Exception as error:
        # Authentication exceptions can include sensitive response details.
        parser.exit(1, f"Calibration export failed ({type(error).__name__}); no calibration written.\n")
    print(f"Saved lidar mount calibration to {args.output}; no robot commands sent.")


if __name__ == "__main__":
    main()
