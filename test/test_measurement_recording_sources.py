"""Focused tests for recording-only ROS and Spot geometry sources."""

import json

from fault_detector_spot.inspection.measurement import (
    MeasurementCompletionState,
    MeasurementRecording,
    MeasurementRepository,
    RosTopicRecordingSource,
    SpotGeometryRecordingSource,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    SensorChannel,
    SensorChannelSource,
)


class _Time:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class _Clock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return _Time(self.nanoseconds)


class _Node:
    def __init__(self, nanoseconds=7_000_000_011):
        self.clock = _Clock(nanoseconds)
        self.subscription = None
        self.subscription_callback = None
        self.timer = None
        self.timer_callback = None
        self.destroyed = []

    def get_clock(self):
        return self.clock

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscription = (message_type, topic, qos)
        self.subscription_callback = callback
        return self.subscription

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)

    def create_timer(self, period, callback):
        self.timer = (period, callback)
        self.timer_callback = callback
        return self.timer

    def destroy_timer(self, timer):
        self.destroyed.append(timer)


class _Stamp:
    sec = 5
    nanosec = 23


class _Header:
    stamp = _Stamp()


class _Message:
    header = _Header()
    value = 4.5


def _pose(x, y=0.0, z=0.0):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=QuaternionData.identity(),
    )


def _recording(channel, started_at_ns=1_788_371_731_438_271_923):
    return MeasurementRecording.start(
        object_id="motor_01",
        routine_id="magnetic_scan",
        probe_point_id="bearing_front",
        sensor_id="bmm150_probe",
        attachment_revision=4,
        started_at_ns=started_at_ns,
        configured_channels=(channel,),
    )


def test_ros_topic_source_subscribes_only_while_recording(tmp_path):
    channel = SensorChannel(
        channel_id="magnetic_field",
        topic="/sensors/probe/magnetic_field",
        message_type="example_msgs/msg/Reading",
    )
    recording = _recording(channel)
    repository = MeasurementRepository(tmp_path)
    repository.create(recording)
    node = _Node()
    first_samples = []
    resolved_types = []

    def resolve_type(type_name):
        resolved_types.append(type_name)
        return _Message

    source = RosTopicRecordingSource(
        node=node,
        repository=repository,
        recording=recording,
        channel=channel,
        on_first_sample=lambda channel_id, timestamp: first_samples.append(
            (channel_id, timestamp)
        ),
        message_type_resolver=resolve_type,
        message_converter=lambda message: {"value": message.value},
    )
    assert node.subscription is None
    source.start()
    assert resolved_types == ["example_msgs/msg/Reading"]
    assert node.subscription[0:2] == (
        _Message,
        "/sensors/probe/magnetic_field",
    )

    node.subscription_callback(_Message())
    node.subscription_callback(_Message())
    source.stop()
    node.subscription_callback(_Message())
    finalized = repository.finalize(
        recording,
        MeasurementCompletionState.COMPLETE,
        recording.started_at_ns + 1,
    )

    rows = [
        json.loads(line)
        for line in repository.get_channel_path(
            recording,
            channel.channel_id,
        ).read_text(encoding="utf-8").splitlines()
    ]
    assert rows == [
        {
            "receive_time_ns": 7_000_000_011,
            "source_time_ns": 5_000_000_023,
            "data": {"value": 4.5},
        },
        {
            "receive_time_ns": 7_000_000_011,
            "source_time_ns": 5_000_000_023,
            "data": {"value": 4.5},
        },
    ]
    assert first_samples == [("magnetic_field", 7_000_000_011)]
    assert finalized.sample_counts == {"magnetic_field": 2}
    assert node.destroyed == [node.subscription]


def test_geometry_source_records_poses_and_deduplicates_tf_errors(tmp_path):
    channel = SensorChannel(
        channel_id="spot_geometry",
        topic="",
        message_type="",
        source_kind=SensorChannelSource.SPOT_GEOMETRY,
    )
    recording = _recording(
        channel,
        started_at_ns=1_788_371_731_438_271_924,
    )
    repository = MeasurementRepository(tmp_path)
    repository.create(recording)
    node = _Node(nanoseconds=8_000_000_019)
    first_samples = []
    errors = []
    lookup_attempts = {"count": 0}

    def lookup(_buffer, target, source, timeout_sec):
        assert target == "odom"
        assert timeout_sec == 0.02
        if lookup_attempts["count"] < 2:
            lookup_attempts["count"] += 1
            raise LookupError("TF is warming up")
        return {
            "body": _pose(1.0),
            "hand": _pose(2.0),
            "bmm150_probe_probe": _pose(12.0),
        }[source]

    source = SpotGeometryRecordingSource(
        node=node,
        tf_buffer=object(),
        repository=repository,
        recording=recording,
        channel=channel,
        object_pose_execution=_pose(10.0),
        probe_frame="bmm150_probe_probe",
        on_first_sample=lambda channel_id, timestamp: first_samples.append(
            (channel_id, timestamp)
        ),
        on_error=lambda channel_id, exception: errors.append(
            (channel_id, str(exception))
        ),
        pose_lookup=lookup,
    )
    source.start()
    assert node.timer[0] == 0.1
    node.timer_callback()
    node.timer_callback()
    node.timer_callback()
    source.stop()
    finalized = repository.finalize(
        recording,
        MeasurementCompletionState.COMPLETE,
        recording.started_at_ns + 1,
    )

    row = json.loads(
        repository.get_channel_path(
            recording,
            channel.channel_id,
        ).read_text(encoding="utf-8")
    )
    assert row["source_time_ns"] == 8_000_000_019
    assert row["data"]["frames"] == {
        "execution": "odom",
        "body": "body",
        "hand": "hand",
        "probe": "bmm150_probe_probe",
        "object": "motor_01",
    }
    assert row["data"]["object_pose_execution"]["position"]["x"] == 10.0
    assert row["data"]["probe_pose_execution"]["position"]["x"] == 12.0
    assert row["data"]["probe_pose_object"]["position"]["x"] == 2.0
    assert errors == [("spot_geometry", "TF is warming up")]
    assert first_samples == [("spot_geometry", 8_000_000_019)]
    assert finalized.sample_counts == {"spot_geometry": 1}
    assert node.destroyed == [node.timer]
