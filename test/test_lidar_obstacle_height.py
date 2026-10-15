"""Offline low-obstacle checks with real TF history and no running ROS nodes."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer

from fault_detector_spot.mapping.ros import lidar_self_filter as filter_module
from fault_detector_spot.mapping.ros.lidar_self_filter import (
    LidarSelfFilter, obstacle_points,
)


def transform(parent, child, z=0.0, stamp=10.0, rotation=None):
    result = TransformStamped()
    result.header.frame_id = parent
    result.child_frame_id = child
    result.header.stamp = Time(seconds=stamp).to_msg()
    result.transform.translation.z = z
    rotation = Rotation.identity() if rotation is None else rotation
    (result.transform.rotation.x, result.transform.rotation.y,
     result.transform.rotation.z, result.transform.rotation.w) = rotation.as_quat()
    return result


def cloud(points, stamp=10.0):
    result = PointCloud2()
    result.header.frame_id = "lidar_sensor"
    result.header.stamp = Time(seconds=stamp).to_msg()
    result.height = 1
    result.width = len(points)
    result.fields = [PointField(
        name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1,
    ) for index, name in enumerate("xyz")]
    result.point_step = 12
    result.row_step = 12 * result.width
    result.data = np.asarray(points, dtype="<f4").tobytes()
    result.is_dense = True
    return result


@pytest.mark.parametrize("sensor_height", [0.35, 0.65, 0.95])
@pytest.mark.parametrize("tilt", [0.0, 0.3])
def test_ground_gate_retains_low_objects_at_different_body_heights_and_tilts(
    sensor_height, tilt,
):
    ground_points = np.array([
        [2.0, 0.0, 0.0],    # floor
        [2.1, 0.0, 0.04],   # below clearance
        [2.2, 0.0, 0.10],   # a low object, below body origin
        [2.3, 0.0, 0.20],
        [2.4, 0.0, 1.60],   # above body-navigation obstacle envelope
    ])
    orientation = Rotation.from_euler("y", tilt)
    sensor_points = orientation.inv().apply(ground_points - [0.0, 0.0, sensor_height])
    sensor_points = np.asarray(sensor_points, dtype="<f4")
    selected = obstacle_points(
        sensor_points, transform("feet_center", "lidar_sensor", sensor_height,
                                 rotation=orientation), 0.05, 1.50,
    )
    # The source coordinates are retained exactly, rather than projected to z=0.
    np.testing.assert_array_equal(selected, sensor_points[[2, 3]])


@pytest.mark.parametrize("floor_altitude", [-4.0, 0.0, 3.10, 20.0])
def test_ground_gate_does_not_assume_floor_is_at_map_or_odometry_zero(floor_altitude):
    points = np.asarray([
        [2.0, 0.0, floor_altitude],
        [2.1, 0.0, floor_altitude + 0.10],
    ], dtype="<f4")
    selected = obstacle_points(
        points, transform("feet_center", "odom", -floor_altitude), 0.05, 1.50,
    )
    np.testing.assert_array_equal(selected, points[[1]])


def filter_rig(*, body_stamp=10.0, body_height=0.50):
    buffer = Buffer()
    buffer.set_transform_static(transform("base_link", "lidar_sensor", 0.15), "test")
    buffer.set_transform(transform("feet_center", "base_link", body_height, body_stamp), "test")
    rig = SimpleNamespace(
        base_frame="base_link", ground_frame="feet_center", tf_buffer=buffer,
        minimum_obstacle_height=0.05, maximum_obstacle_height=1.50, tf_timeout_sec=0.0,
        max_cloud_age_sec=1.0,
        get_arm_box=lambda: np.array([0.25, 0.70, -0.23, 0.23, -1.0, 1.10]),
        cloud_publisher=Mock(), obstacle_publisher=Mock(), warn_throttled=Mock(),
        last_stats_ns=10_000_000_000,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10.0)),
    )
    rig.validate_cloud = lambda message: LidarSelfFilter.validate_cloud(rig, message)
    rig.check_acquisition_time = lambda message, stage: LidarSelfFilter.check_acquisition_time(
        rig, message, stage,
    )
    rig.publish_obstacles = lambda message, points: LidarSelfFilter.publish_obstacles(
        rig, message, points,
    )
    return rig


def coordinates(message):
    return np.frombuffer(message.data, dtype="<f4").reshape((-1, 3))


def test_callback_keeps_arm_mask_and_full_mapping_scan_but_marks_low_objects():
    rig = filter_rig()
    # Body height changes after the image; selection must use capture-time TF.
    rig.tf_buffer.set_transform(transform("feet_center", "base_link", 0.90, 10.3), "test")
    source = cloud([
        [2.0, 0.0, -0.65],  # floor at acquisition
        [2.1, 0.0, -0.55],  # 10 cm obstacle
        [2.2, 0.0, -0.45],  # 20 cm obstacle
        [0.5, 0.0, 0.0],    # arm exclusion is still body-relative
    ])

    LidarSelfFilter.cloud_callback(rig, source)

    mapping = rig.cloud_publisher.publish.call_args.args[0]
    marking = rig.obstacle_publisher.publish.call_args.args[0]
    np.testing.assert_array_equal(coordinates(mapping), coordinates(source)[:3])
    np.testing.assert_array_equal(coordinates(marking), coordinates(source)[1:3])
    assert marking.header == mapping.header == source.header
    assert marking.is_dense


def test_missing_historical_ground_tf_withholds_marking_without_reporting_clear_scene():
    rig = filter_rig(body_stamp=11.0)
    LidarSelfFilter.cloud_callback(rig, cloud([[2.0, 0.0, -0.55]]))
    assert rig.cloud_publisher.publish.call_count == 1
    assert rig.obstacle_publisher.publish.call_count == 0
    assert "Skipping obstacle cloud" in rig.warn_throttled.call_args.args[0]


def test_normal_sdk_capture_delay_preserves_both_observations_and_original_stamps():
    rig = filter_rig()
    rig.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=10.4))
    source = cloud([[2.0, 0.0, -0.55]])
    LidarSelfFilter.cloud_callback(rig, source)
    for publisher in (rig.cloud_publisher, rig.obstacle_publisher):
        published = publisher.publish.call_args.args[0]
        assert published.header == source.header
        assert published.width == 1


def test_filter_keeps_only_latest_pending_scan_while_allowing_bounded_tf_wait(monkeypatch):
    parameters, publishers, subscriptions = {}, {}, {}
    # Inspect initialization without constructing a ROS node or DDS participant.
    monkeypatch.setattr(filter_module.Node, "__init__", lambda self, *args, **kwargs: None)
    monkeypatch.setattr(LidarSelfFilter, "declare_parameter",
                        lambda self, name, value: parameters.setdefault(name, value))
    monkeypatch.setattr(LidarSelfFilter, "get_parameter",
                        lambda self, name: SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(LidarSelfFilter, "create_publisher",
                        lambda self, message, topic, qos: publishers.setdefault(topic, qos))
    monkeypatch.setattr(LidarSelfFilter, "create_subscription",
                        lambda self, message, topic, callback, qos:
                        subscriptions.setdefault(topic, qos))
    monkeypatch.setattr(LidarSelfFilter, "create_timer", lambda *args: None)
    monkeypatch.setattr(filter_module.tf2_ros, "Buffer", Mock())
    monkeypatch.setattr(filter_module.tf2_ros, "TransformListener", Mock())

    filtering = LidarSelfFilter()

    assert subscriptions["/velodyne/points"].depth == 1
    assert publishers["/velodyne/points_filtered"].depth == 1
    assert publishers["/velodyne/points_obstacles"].depth == 1
    assert filtering.tf_timeout_sec == 0.25
    assert filtering.max_cloud_age_sec == 1.0


def test_floor_only_cloud_is_valid_empty_marking_observation():
    rig = filter_rig()
    LidarSelfFilter.cloud_callback(rig, cloud([[2.0, 0.0, -0.65]]))
    marking = rig.obstacle_publisher.publish.call_args.args[0]
    assert marking.width == 0
    assert not marking.data


@pytest.mark.parametrize("bad", ["zero_stamp", "wrong_frame", "wrong_time", "nan_transform"])
def test_unreliable_ground_transform_never_becomes_a_cleared_marking_cloud(bad):
    rig = filter_rig()
    source = cloud([[2.0, 0.0, -0.55]])
    target = transform("feet_center", "lidar_sensor", 0.65)
    if bad == "zero_stamp":
        source.header.stamp = Time().to_msg()
    elif bad == "wrong_frame":
        target.header.frame_id = "base_link"
    elif bad == "wrong_time":
        target.header.stamp = Time(seconds=11.0).to_msg()
    else:
        target.transform.translation.z = float("nan")
    rig.tf_buffer = SimpleNamespace(lookup_transform=lambda *args, **kwargs: target)
    LidarSelfFilter.publish_obstacles(rig, source, coordinates(source))
    assert rig.obstacle_publisher.publish.call_count == 0
    assert rig.warn_throttled.call_count == 1


@pytest.mark.parametrize("costmap", ["local_costmap", "global_costmap"])
def test_costmaps_mark_ground_selected_points_and_clear_with_physical_rays(costmap):
    config = yaml.safe_load((Path(__file__).parents[1] / "config/nav2_lidar_params.yaml").read_text())
    parameters = config[costmap][costmap]["ros__parameters"]
    layer = parameters["obstacle_layer"]
    assert "voxel_layer" not in parameters["plugins"]
    assert layer["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    marking, clearing = layer["velodyne"], layer["velodyne_clearing"]
    assert set(layer["observation_sources"].split()) == {"velodyne", "velodyne_clearing"}
    assert marking["topic"] == "/velodyne/points_obstacles"
    assert marking["marking"] and not marking["clearing"]
    assert clearing["topic"] == "/velodyne/points_filtered"
    assert clearing["clearing"] and not clearing["marking"]
    assert layer["max_obstacle_height"] == np.finfo(float).max
    for observation in (marking, clearing):
        assert observation["min_obstacle_height"] == -np.finfo(float).max
        assert observation["max_obstacle_height"] == np.finfo(float).max
        # Normal 0.20--0.38 s arrival gaps must remain current. An actual
        # missing stream still expires after a bounded subsecond deadline.
        assert 0.38 < observation["expected_update_rate"] <= 0.5
        # A 0.3 s persistence interval purges a valid scan already 0.4 s old
        # on arrival. Zero explicitly means keep only the latest observation.
        assert observation["observation_persistence"] == 0.0


@pytest.mark.parametrize("capture_stamp", [0.0, 8.99, 10.06])
def test_invalid_or_aged_queued_scan_cannot_refresh_either_nav2_stream(capture_stamp):
    rig = filter_rig()
    rig.tf_buffer = Mock()
    LidarSelfFilter.cloud_callback(rig, cloud([[2.0, 0.0, -0.55]], capture_stamp))
    rig.tf_buffer.lookup_transform.assert_not_called()
    rig.cloud_publisher.publish.assert_not_called()
    rig.obstacle_publisher.publish.assert_not_called()
    assert "acquisition time" in rig.warn_throttled.call_args.args[0]


def test_capture_that_expires_during_body_tf_wait_is_not_published():
    rig = filter_rig()
    rig.now = 10.8
    rig.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=rig.now))

    def delayed_body_transform(*args, **kwargs):
        rig.now = 11.01
        return transform("base_link", "lidar_sensor", 0.15)

    rig.tf_buffer = SimpleNamespace(lookup_transform=delayed_body_transform)
    LidarSelfFilter.cloud_callback(rig, cloud([[2.0, 0.0, -0.55]]))
    rig.cloud_publisher.publish.assert_not_called()
    rig.obstacle_publisher.publish.assert_not_called()
    assert "filtered output" in rig.warn_throttled.call_args.args[0]


@pytest.mark.parametrize("after_wait", [10.95, 11.01])
def test_ground_tf_wait_preserves_fresh_capture_but_cannot_refresh_an_expired_scan(after_wait):
    rig = filter_rig()
    rig.now = 10.8
    rig.tf_timeout_sec = 0.25
    rig.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=rig.now))
    requests = []

    def delayed_ground_transform(target, source, stamp, timeout):
        requests.append((target, source, stamp, timeout))
        rig.now = after_wait
        return transform("feet_center", "lidar_sensor", 0.65)

    rig.tf_buffer = SimpleNamespace(lookup_transform=delayed_ground_transform)
    source = cloud([[2.0, 0.0, -0.55]])
    LidarSelfFilter.publish_obstacles(rig, source, coordinates(source))
    assert requests[0][2].to_msg() == source.header.stamp
    assert requests[0][3].nanoseconds == 250_000_000
    if after_wait <= 11.0:
        published = rig.obstacle_publisher.publish.call_args.args[0]
        assert published.header == source.header
        assert published.width == 1
    else:
        rig.obstacle_publisher.publish.assert_not_called()
        assert "obstacle output" in rig.warn_throttled.call_args.args[0]
