"""Offline contracts for converting Spot's world cloud into its lidar frame."""

from copy import deepcopy
import asyncio
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.clock import ROSClock
from rclpy.executors import Executor
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rclpy.time_source import TimeSource
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException

from fault_detector_spot.sensing import lidar_frame_adapter as adapter_module
from fault_detector_spot.sensing.lidar_frame_adapter import (
    LidarFrameAdapter,
    transform_lidar_cloud,
    validate_lidar_cloud,
)


def cloud(points=((1.0, 2.0, 3.0),), stamp=9.8):
    message = PointCloud2()
    message.header.frame_id = "sensor_origin_velodyne-point-cloud"
    message.header.stamp = Time(seconds=stamp).to_msg()
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate("xyz")
    ]
    message.point_step = 12
    message.row_step = message.width * 12
    message.is_bigendian = False
    message.is_dense = True
    message.data = np.asarray(points, dtype="<f4").tobytes()
    return message


def transform(target="lidar_sensor", source="sensor_origin_velodyne-point-cloud",
              stamp=9.8, translation=(0.0, 0.0, 0.0), yaw=0.0):
    message = TransformStamped()
    message.header.frame_id = target
    message.child_frame_id = source
    message.header.stamp = Time(seconds=stamp).to_msg()
    message.transform.translation.x = float(translation[0])
    message.transform.translation.y = float(translation[1])
    message.transform.translation.z = float(translation[2])
    message.transform.rotation.z = math.sin(yaw / 2.0)
    message.transform.rotation.w = math.cos(yaw / 2.0)
    return message


def coordinates(message):
    return np.frombuffer(message.data, dtype="<f4").reshape(-1, 3)


def test_real_standard_transform_preserves_source_and_acquisition_stamp():
    source = cloud(((1.0, 0.0, 0.0), (0.0, 2.0, -1.0)))
    original = deepcopy(source)
    target = transform(stamp=8.0, translation=(3.0, -2.0, 1.0), yaw=math.pi / 2)

    result = transform_lidar_cloud(source, target)

    np.testing.assert_allclose(
        coordinates(result), ((3.0, -1.0, 1.0), (1.0, -2.0, 0.0)), atol=1e-6,
    )
    assert result.header.frame_id == "lidar_sensor"
    assert result.header.stamp == source.header.stamp
    assert result.width == 2 and result.height == 1
    assert result.fields == source.fields
    assert result.is_dense is False
    assert source == original


def test_invalid_points_are_preserved_without_removing_valid_points():
    source = cloud(((1.0, 2.0, 3.0), (float("nan"), 0.0, 0.0), (4.0, 5.0, 6.0)))
    original_bytes = bytes(source.data)

    result = transform_lidar_cloud(source, transform(translation=(1.0, 0.0, 0.0)))

    assert result.width == 3
    np.testing.assert_allclose(coordinates(result)[[0, 2]], ((2.0, 2.0, 3.0), (5.0, 5.0, 6.0)))
    assert np.isnan(coordinates(result)[1]).any()
    assert bytes(source.data) == original_bytes
    assert result.is_dense is False


@pytest.mark.parametrize("fault", [
    "empty_frame", "organized", "empty", "big_endian", "point_padding", "row_padding",
    "short_buffer", "long_buffer", "missing_field", "extra_field", "wrong_order",
    "wrong_name", "wrong_offset", "wrong_count", "wrong_datatype",
])
def test_layout_validation_rejects_unsupported_clouds(fault):
    message = cloud()
    if fault == "empty_frame":
        message.header.frame_id = ""
    elif fault == "organized":
        message.height = 2
    elif fault == "empty":
        message.width = 0
        message.row_step = 0
        message.data = b""
    elif fault == "big_endian":
        message.is_bigendian = True
    elif fault == "point_padding":
        message.point_step = 16
    elif fault == "row_padding":
        message.row_step += 4
    elif fault == "short_buffer":
        message.data = bytes(message.data[:-1])
    elif fault == "long_buffer":
        message.data = bytes(message.data) + b"\x00"
    elif fault == "missing_field":
        message.fields = message.fields[:2]
    elif fault == "extra_field":
        message.fields.append(PointField(
            name="intensity", offset=12, datatype=PointField.FLOAT32, count=1,
        ))
    elif fault == "wrong_order":
        message.fields = list(reversed(message.fields))
    elif fault == "wrong_name":
        message.fields[0].name = "intensity"
    elif fault == "wrong_offset":
        message.fields[0].offset = 4
    elif fault == "wrong_count":
        message.fields[0].count = 2
    elif fault == "wrong_datatype":
        message.fields[0].datatype = PointField.INT32

    with pytest.raises((ValueError, TypeError)):
        validate_lidar_cloud(message)


def test_cloud_point_limit_is_inclusive():
    message = cloud(((1.0, 2.0, 3.0),) * 2)
    validate_lidar_cloud(message, max_points=2)
    with pytest.raises(ValueError):
        validate_lidar_cloud(message, max_points=1)


@pytest.mark.parametrize("fault", [
    "empty_target", "wrong_source", "zero_quaternion", "nan_quaternion", "infinite_translation",
])
def test_transform_validation_prevents_invalid_or_mismatched_geometry(fault):
    target = transform()
    if fault == "empty_target":
        target.header.frame_id = ""
    elif fault == "wrong_source":
        target.child_frame_id = "body"
    elif fault == "zero_quaternion":
        target.transform.rotation.w = 0.0
    elif fault == "nan_quaternion":
        target.transform.rotation.z = float("nan")
    else:
        target.transform.translation.x = float("inf")

    with pytest.raises(ValueError):
        transform_lidar_cloud(cloud(), target)


def test_real_tf_buffer_resolves_moving_body_at_each_scan_time():
    # No ROS initialization, node, listener, or live TF traffic is involved.
    buffer = Buffer()
    buffer.set_transform_static(transform("odom", "sensor_origin_velodyne-point-cloud"), "test")
    buffer.set_transform_static(transform(
        "body", "lidar_sensor", translation=(-0.2, 0.0, 0.0), yaw=math.pi / 2,
    ), "test")
    for stamp, x in ((10.0, 2.0), (11.0, 4.0)):
        buffer.set_transform(transform(
            "odom", "body", stamp=stamp, translation=(x, 0.0, 0.0),
        ), "test")

    results = []
    for stamp in (10.0, 11.0):
        message = cloud(((5.0, 0.0, 0.0),), stamp=stamp)
        target = buffer.lookup_transform(
            "lidar_sensor", message.header.frame_id, Time.from_msg(message.header.stamp),
        )
        results.append(coordinates(transform_lidar_cloud(message, target))[0])

    np.testing.assert_allclose(results, ((0.0, -3.2, 0.0), (0.0, -1.2, 0.0)), atol=1e-6)


class FakeNode:

    def __init__(self, parameters=None):
        self.parameters = parameters or {}
        self.declarations = {}
        self.now = 10.0
        self.published = []
        self.collision_published = []
        self.warnings = []
        self.publishers = []
        self.subscriptions = []
        self.services = []
        self.jump_unregister_count = 0
        self.clock = SimpleNamespace(
            now=lambda: Time(seconds=self.now),
            create_jump_callback=self.create_jump_callback,
        )

    def declare_parameter(self, name, default, descriptor=None):
        self.declarations[name] = descriptor
        return SimpleNamespace(value=self.parameters.get(name, default))

    def get_clock(self):
        return self.clock

    def create_jump_callback(self, threshold, post_callback):
        self.jump_threshold = threshold
        self.jump_callback = post_callback

        def unregister():
            self.jump_unregister_count += 1

        return SimpleNamespace(unregister=unregister)

    def resolve_topic_name(self, topic):
        return "/test/" + topic

    def get_logger(self):
        return SimpleNamespace(
            warning=lambda message, **kwargs: self.warnings.append((message, kwargs)),
            info=lambda *args, **kwargs: None,
        )

    def create_publisher(self, message_type, topic, qos):
        self.publishers.append((message_type, topic, qos))
        messages = self.published if topic == "output" else self.collision_published
        return SimpleNamespace(publish=messages.append)

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions.append((message_type, topic, callback, qos))
        return SimpleNamespace()

    def create_service(self, service_type, name, callback):
        service = SimpleNamespace(srv_type=service_type, name=name, callback=callback)
        self.services.append(service)
        return service


class FakeBuffer:

    def __init__(self):
        self.calls = []
        self.error = None

    def lookup_transform(self, target, source, stamp, timeout):
        self.calls.append((target, source, stamp, timeout))
        if self.error:
            raise self.error
        return transform(target, source, stamp=stamp.nanoseconds / 1e9)

    def clear(self):
        self.error = TransformException("No dynamic transforms after clock jump")


@pytest.fixture
def rig(monkeypatch):
    value = SimpleNamespace(now=100.0, buffer=FakeBuffer(), unregister_count=0)
    monkeypatch.setattr(adapter_module, "Buffer", lambda *args, **kwargs: value.buffer)

    def listener(*args, **kwargs):
        def unregister():
            value.unregister_count += 1
        return SimpleNamespace(unregister=unregister)

    monkeypatch.setattr(adapter_module, "TransformListener", listener)
    if hasattr(adapter_module, "monotonic"):
        monkeypatch.setattr(adapter_module, "monotonic", lambda: value.now)
    else:
        monkeypatch.setattr(adapter_module.time, "monotonic", lambda: value.now)

    def create(parameters=None, clock=None, tf_buffer=None):
        if tf_buffer is not None:
            value.buffer = tf_buffer
        value.node = FakeNode(parameters)
        if clock is not None:
            value.node.clock = clock
        value.adapter = LidarFrameAdapter(value.node)
        return value

    return create


def test_adapter_uses_explicit_acquisition_time_and_bounded_volatile_qos(rig):
    value = rig({"sensor_frame": "calibrated_lidar"})
    source = cloud()

    value.adapter.receive_cloud(source)

    target, frame, stamp, timeout = value.buffer.calls[0]
    assert target == "calibrated_lidar" and frame == source.header.frame_id
    assert stamp.nanoseconds == Time.from_msg(source.header.stamp).nanoseconds
    assert timeout.nanoseconds == 0
    assert value.node.published[0].header.stamp == source.header.stamp
    assert value.node.published[0].header.frame_id == "calibrated_lidar"
    input_qos = value.node.subscriptions[0][3]
    output_qos = value.node.publishers[0][2]
    assert input_qos.depth == output_qos.depth == 1
    assert input_qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert output_qos.reliability == ReliabilityPolicy.RELIABLE
    assert output_qos.durability == DurabilityPolicy.VOLATILE
    collision_qos = value.node.publishers[1][2]
    assert collision_qos.depth == 1
    assert collision_qos.reliability == ReliabilityPolicy.RELIABLE
    assert collision_qos.durability == DurabilityPolicy.VOLATILE
    assert value.node.collision_published[0] is value.node.published[0]
    for name in ("sensor_frame", "max_cloud_age_sec", "max_points", "max_rate_hz",
                 "collision_max_rate_hz"):
        assert value.node.declarations[name].read_only
    assert value.node.jump_threshold.min_backward.nanoseconds == -1
    assert value.node.jump_threshold.on_clock_change
    value.adapter.destroy()
    value.adapter.destroy()
    assert value.unregister_count == 1
    assert value.node.jump_unregister_count == 1


def test_stop_acknowledgement_is_sent_before_executor_and_node_shutdown(rig, monkeypatch):
    value = rig()
    service, = value.node.services
    assert service.srv_type is Trigger
    assert service.name == "/fault_detector/lidar_adapter/stop"
    events = []

    def send_response(response, _header):
        assert response.success
        assert response.message == "Lidar adapter stopping"
        assert value.adapter.stop_requested
        assert value.unregister_count == 0
        assert value.node.jump_unregister_count == 0
        events.append("response sent")

    service.send_response = send_response

    class FakeExecutor:
        def add_node(self, node):
            assert node is value.node

        def spin_once(self, timeout_sec):
            assert 0 < timeout_sec <= 0.2
            assert events == []
            # Real rclpy service execution, without initialization or ROS traffic.
            asyncio.run(Executor._execute_service(
                self, service, (Trigger.Request(), object()),
            ))
            assert events == ["response sent"]

        def shutdown(self):
            events.append("executor shutdown")

    monkeypatch.setattr(adapter_module.rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(adapter_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        adapter_module.rclpy, "try_shutdown", lambda: events.append("context shutdown"),
    )
    monkeypatch.setattr(adapter_module, "Node", lambda _name: value.node)
    monkeypatch.setattr(adapter_module, "LidarFrameAdapter", lambda _node: value.adapter)
    monkeypatch.setattr(adapter_module, "SingleThreadedExecutor", FakeExecutor)
    value.node.destroy_node = lambda: events.append("node destroyed")

    adapter_module.main()

    assert events == [
        "response sent", "executor shutdown", "node destroyed", "context shutdown",
    ]
    assert value.unregister_count == value.node.jump_unregister_count == 1


@pytest.mark.parametrize("clock_event", ["rewind", "source_change"])
def test_clock_epoch_change_requires_new_dynamic_tf_and_preserves_static_mount(rig, clock_event):
    # Exercise real clock callbacks and TF caches without any ROS node or traffic.
    clock = ROSClock()
    time_source = TimeSource()
    time_source.attach_clock(clock)
    time_source.ros_time_is_active = True
    clock.set_ros_time_override(Time(seconds=10))
    buffer = Buffer()
    buffer.set_transform_static(transform(
        "odom", "sensor_origin_velodyne-point-cloud",
    ), "test")
    buffer.set_transform_static(transform(
        "body", "lidar_sensor", translation=(-0.2, 0.0, 0.0),
    ), "test")
    for stamp in (9.0, 10.0):
        buffer.set_transform(transform(
            "odom", "body", stamp=stamp, translation=(1.0, 0.0, 0.0),
        ), "test")
    value = rig(clock=clock, tf_buffer=buffer)
    value.adapter.receive_cloud(cloud(((5.0, 0.0, 0.0),), stamp=10.0))
    np.testing.assert_allclose(coordinates(value.node.published[-1]), ((4.2, 0.0, 0.0),))

    if clock_event == "source_change":
        time_source.ros_time_is_active = False
        assert not buffer.can_transform("odom", "body", Time(seconds=9))
        time_source.ros_time_is_active = True
    clock.set_ros_time_override(Time(seconds=9))
    mount = buffer.lookup_transform("body", "lidar_sensor", Time(seconds=9))
    assert mount.transform.translation.x == pytest.approx(-0.2)
    value.now += 1.0
    value.adapter.receive_cloud(cloud(((5.0, 0.0, 0.0),), stamp=9.0))
    assert len(value.node.published) == 1

    buffer.set_transform(transform(
        "odom", "body", stamp=9.0, translation=(3.0, 0.0, 0.0),
    ), "test")
    value.now += 1.0
    value.adapter.receive_cloud(cloud(((5.0, 0.0, 0.0),), stamp=9.0))
    assert len(value.node.published) == 2
    np.testing.assert_allclose(coordinates(value.node.published[-1]), ((2.2, 0.0, 0.0),))

    value.adapter.destroy()
    clock.set_ros_time_override(Time(seconds=8))
    assert buffer.can_transform("odom", "body", Time(seconds=9))


@pytest.mark.parametrize("stamp,now,accepted", [
    (0.0, 10.0, False), (9.0, 10.0, False), (10.1, 10.0, False),
    (9.8, 0.0, False), (9.249, 10.0, False), (9.25, 10.0, True),
    (9.444, 10.0, True), (10.05, 10.0, True),
])
def test_freshness_bounds_checked_before_tf_lookup(rig, stamp, now, accepted):
    value = rig()
    value.node.now = now

    value.adapter.receive_cloud(cloud(stamp=stamp))

    assert bool(value.node.published) is accepted
    assert bool(value.node.collision_published) is accepted
    assert bool(value.buffer.calls) is accepted


def test_scan_that_expires_during_transform_is_not_published(rig, monkeypatch):
    value = rig()
    original = adapter_module.transform_lidar_cloud

    def delayed_transform(*args, **kwargs):
        output = original(*args, **kwargs)
        value.node.now = 10.6
        return output

    monkeypatch.setattr(adapter_module, "transform_lidar_cloud", delayed_transform)

    value.adapter.receive_cloud(cloud(stamp=9.8))

    assert len(value.buffer.calls) == 1
    assert value.node.published == []
    assert value.node.collision_published == []


def test_missing_tf_drops_scan_without_latest_time_fallback_and_recovers(rig):
    value = rig()
    value.buffer.error = TransformException("transform unavailable")
    value.adapter.receive_cloud(cloud())
    assert len(value.buffer.calls) == 1
    assert value.node.published == []
    assert value.node.collision_published == []

    value.now += 1.0
    value.buffer.error = None
    value.adapter.receive_cloud(cloud())
    assert len(value.node.published) == 1


def test_malformed_cloud_does_not_poison_subsequent_scan(rig):
    value = rig()
    malformed = cloud()
    malformed.data = b""
    value.adapter.receive_cloud(malformed)
    assert value.node.published == []
    assert value.buffer.calls == []
    assert value.node.warnings[0][1]["throttle_duration_sec"] == 2.0

    value.now += 1.0
    value.adapter.receive_cloud(cloud())
    assert len(value.node.published) == 1


def test_default_rate_cap_skips_work_and_resumes_after_ros_clock_rewind(rig):
    value = rig()
    value.adapter.receive_cloud(cloud())
    value.node.now = 5.0
    value.now += 0.049
    value.adapter.receive_cloud(cloud(stamp=5.0))
    assert len(value.buffer.calls) == len(value.node.published) == 1

    value.now += 0.002
    value.adapter.receive_cloud(cloud(stamp=5.0))
    assert len(value.buffer.calls) == len(value.node.published) == 2
    assert len(value.node.collision_published) == 1

    value.now += 0.15
    value.adapter.receive_cloud(cloud(stamp=5.0))
    assert len(value.node.published) == 3
    assert len(value.node.collision_published) == 2
    assert value.node.collision_published[-1] is value.node.published[-1]


def test_default_cadence_keeps_jittered_driver_clouds_within_nav2_deadline(rig):
    value = rig()
    received_at = []
    collision_received_at = []
    for index in range(30):
        # A nominal 10 Hz driver can alternate just before/after each deadline.
        # A strict cap of 5 Hz or even 10 Hz discards valid scans in this stream.
        elapsed = index * 0.1 + (0.001 if index % 2 else 0.0)
        value.now = 100.0 + elapsed
        value.node.now = 10.0 + elapsed
        source = cloud(stamp=9.8 + elapsed)
        collision_count = len(value.node.collision_published)

        value.adapter.receive_cloud(source)

        assert len(value.node.published) == index + 1
        assert value.node.published[-1].header.stamp == source.header.stamp
        received_at.append(elapsed)
        if len(value.node.collision_published) > collision_count:
            collision_received_at.append(elapsed)
            assert value.node.collision_published[-1] is value.node.published[-1]

    assert len(value.buffer.calls) == 30  # One conversion shared by both outputs.
    assert 1 < len(collision_received_at) < len(received_at)
    assert min(np.diff(collision_received_at)) >= 0.2 - 1e-12

    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/nav2_lidar_params.yaml").read_text()
    )
    max_gap = max(np.diff(received_at))
    for costmap, layer in (("local_costmap", "voxel_layer"),
                           ("global_costmap", "obstacle_layer")):
        observation = config[costmap][costmap]["ros__parameters"][layer]["velodyne"]
        assert max_gap < observation["expected_update_rate"]


def test_rejected_scans_also_consume_the_rate_budget(rig):
    value = rig({"max_rate_hz": 2.0})
    value.buffer.error = TransformException("transform unavailable")
    value.adapter.receive_cloud(cloud())
    value.now += 0.49
    value.adapter.receive_cloud(cloud())
    assert len(value.buffer.calls) == 1

    value.buffer.error = None
    value.now += 0.02
    value.adapter.receive_cloud(cloud())
    assert len(value.buffer.calls) == 2
    assert len(value.node.published) == 1


def test_callback_applies_configured_point_limit_before_tf_work(rig):
    value = rig({"max_points": 1})
    value.adapter.receive_cloud(cloud(((1.0, 2.0, 3.0),) * 2))
    assert value.buffer.calls == []
    assert value.node.published == []


@pytest.mark.parametrize("parameter,value", [
    ("sensor_frame", ""), ("max_cloud_age_sec", 0.0),
    ("max_cloud_age_sec", float("nan")), ("max_points", 0),
    ("max_rate_hz", 0.0), ("max_rate_hz", float("inf")),
    ("collision_max_rate_hz", 0.0), ("collision_max_rate_hz", float("nan")),
])
def test_invalid_configuration_fails_before_creating_traffic(rig, parameter, value):
    with pytest.raises(ValueError):
        rig({parameter: value})


@pytest.mark.parametrize("aliases", [
    ("input", "output"), ("input", "collision_output"), ("output", "collision_output"),
])
def test_topic_remap_loop_or_merged_outputs_are_rejected(monkeypatch, rig, aliases):
    monkeypatch.setattr(
        FakeNode, "resolve_topic_name",
        lambda self, topic: "/same_topic" if topic in aliases else "/" + topic,
    )
    with pytest.raises(ValueError):
        rig()
