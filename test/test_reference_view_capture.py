"""Tests for composed reference-view capture."""

from copy import deepcopy
from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import TagElement
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection import reference_view_capture
from fault_detector_spot.inspection.models import (
    InspectionObject,
    InspectionRoutine,
    PoseData,
    ReferenceTag,
    ReferenceView,
)


def make_object():
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(
            tag_id=23,
            tag_family="36h11",
        ),
        routines=[
            InspectionRoutine(
                routine_id="magnetic_scan",
                display_name="Magnetic scan",
                sensor_id="bmm150",
                probe_frame="sensor_tip",
            )
        ],
    )


def make_inputs():
    rgb_image = Image()
    rgb_image.header.frame_id = "hand_color_image_sensor"
    rgb_image.header.stamp.sec = 10
    rgb_image.header.stamp.nanosec = 200_000_000
    rgb_image.width = 2
    rgb_image.height = 2
    rgb_image.encoding = "rgb8"
    rgb_image.step = 6
    rgb_image.data = bytes(range(12))

    depth_image = Image()
    depth_image.header.frame_id = "hand_color_image_sensor"
    depth_image.header.stamp.sec = 10
    depth_image.header.stamp.nanosec = 210_000_000
    depth_image.width = 2
    depth_image.height = 2
    depth_image.encoding = "16UC1"
    depth_image.step = 4
    depth_image.data = bytes(range(8))

    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.width = 2
    camera_info.height = 2
    camera_info.k = [
        100.0,
        0.0,
        1.0,
        0.0,
        100.0,
        1.0,
        0.0,
        0.0,
        1.0,
    ]

    reference_tag = TagElement()
    reference_tag.id = 23
    reference_tag.pose.header.frame_id = "body"
    reference_tag.pose.header.stamp.sec = 10
    reference_tag.pose.header.stamp.nanosec = 100_000_000
    reference_tag.pose.pose.position.x = 1.0
    reference_tag.pose.pose.orientation.w = 1.0
    return rgb_image, depth_image, camera_info, reference_tag


class FakeSynchronizer:
    def __init__(self, inputs):
        self.inputs = inputs
        self.requests = []

    def best_snapshot(
        self,
        reference_tag_id,
        minimum_image_sequence,
    ):
        self.requests.append(
            (reference_tag_id, minimum_image_sequence)
        )
        return deepcopy(self.inputs)


class FakeRepository:
    def __init__(self, definition=None, save_error=None):
        self.definition = definition or make_object()
        self.save_error = save_error
        self.loaded_object_ids = []
        self.saved = []

    def load(self, object_id):
        self.loaded_object_ids.append(object_id)
        return self.definition

    def save_reference_dataset(self, *arguments):
        self.saved.append(deepcopy(arguments))
        if self.save_error is not None:
            raise self.save_error
        return self.definition


def now():
    return SimpleNamespace(nanoseconds=10_300_000_000)


def resolved_view():
    return ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
    )


def install_resolver(monkeypatch, error=None):
    calls = []

    def resolve(*arguments, **keywords):
        calls.append((arguments, keywords))
        if error is not None:
            raise error
        return resolved_view()

    monkeypatch.setattr(
        reference_view_capture,
        "resolve_reference_view_pose",
        resolve,
    )
    return calls


def capture(repository, synchronizer, **keywords):
    return reference_view_capture.capture_reference_view(
        repository,
        synchronizer,
        object(),
        "motor_a",
        "magnetic_scan",
        now(),
        **keywords,
    )


def test_capture_consumes_completed_collection(monkeypatch):
    repository = FakeRepository()
    synchronizer = FakeSynchronizer(make_inputs())
    resolver_calls = install_resolver(monkeypatch)

    result = capture(
        repository,
        synchronizer,
        minimum_image_sequence=4,
    )

    assert result is repository.definition
    assert synchronizer.requests == [(23, 4)]
    assert repository.loaded_object_ids == ["motor_a"]
    assert len(resolver_calls) == 1
    assert resolver_calls[0][1] == {
        "fixed_frame": "odom",
        "transform_timeout_sec": 0.05,
    }
    saved = repository.saved[0]
    assert saved[0:2] == ("motor_a", "magnetic_scan")
    assert saved[2] == resolved_view()
    assert saved[3].header.stamp.nanosec == 200_000_000
    assert saved[4].header.stamp.nanosec == 210_000_000


def test_no_valid_candidate_is_rejected_before_tf(monkeypatch):
    repository = FakeRepository()
    synchronizer = FakeSynchronizer(None)
    resolver_calls = install_resolver(monkeypatch)

    with pytest.raises(
        reference_view_capture.ReferenceViewCaptureNotReady,
        match="No valid synchronized",
    ):
        capture(repository, synchronizer)

    assert resolver_calls == []
    assert repository.saved == []


def test_missing_routine_is_rejected_before_collection_read(
    monkeypatch,
):
    definition = make_object()
    definition.routines = []
    repository = FakeRepository(definition=definition)
    synchronizer = FakeSynchronizer(make_inputs())
    install_resolver(monkeypatch)

    with pytest.raises(KeyError, match="Routine does not exist"):
        capture(repository, synchronizer)

    assert synchronizer.requests == []


def test_existing_reference_requires_explicit_replacement(monkeypatch):
    definition = make_object()
    definition.routines[0].reference_view = resolved_view()
    repository = FakeRepository(definition=definition)
    synchronizer = FakeSynchronizer(make_inputs())
    install_resolver(monkeypatch)

    with pytest.raises(FileExistsError, match="already has"):
        capture(repository, synchronizer)

    assert synchronizer.requests == []


def test_existing_reference_can_be_replaced(monkeypatch):
    definition = make_object()
    definition.routines[0].reference_view = resolved_view()
    repository = FakeRepository(definition=definition)
    synchronizer = FakeSynchronizer(make_inputs())
    install_resolver(monkeypatch)

    capture(repository, synchronizer, replace_existing=True)

    assert synchronizer.requests == [(23, 0)]
    assert len(repository.saved) == 1


@pytest.mark.parametrize(
    "input_index,name",
    [
        (0, "RGB image"),
        (1, "Depth image"),
        (3, "Base-camera tag observation"),
    ],
)
def test_stale_dynamic_input_is_rejected(
    monkeypatch,
    input_index,
    name,
):
    inputs = list(make_inputs())
    target = inputs[input_index]
    header = target.pose.header if input_index == 3 else target.header
    header.stamp.sec = 8
    header.stamp.nanosec = 0
    repository = FakeRepository()
    resolver_calls = install_resolver(monkeypatch)

    with pytest.raises(
        reference_view_capture.ReferenceViewCaptureNotReady,
        match=f"{name} is stale",
    ):
        capture(repository, FakeSynchronizer(tuple(inputs)))

    assert resolver_calls == []
    assert repository.saved == []


def test_future_input_is_rejected_before_tf(monkeypatch):
    inputs = list(make_inputs())
    inputs[0].header.stamp.sec = 11
    repository = FakeRepository()
    resolver_calls = install_resolver(monkeypatch)

    with pytest.raises(ValueError, match="timestamp is in the future"):
        capture(repository, FakeSynchronizer(tuple(inputs)))

    assert resolver_calls == []


@pytest.mark.parametrize("maximum_age", [-0.01, float("inf")])
def test_invalid_freshness_configuration_is_rejected(maximum_age):
    repository = FakeRepository()
    synchronizer = FakeSynchronizer(make_inputs())

    with pytest.raises(ValueError, match="Maximum input age"):
        capture(
            repository,
            synchronizer,
            maximum_input_age_sec=maximum_age,
        )

    assert repository.loaded_object_ids == []
    assert synchronizer.requests == []


def test_tf_failure_prevents_persistence(monkeypatch):
    repository = FakeRepository()
    synchronizer = FakeSynchronizer(make_inputs())
    install_resolver(monkeypatch, RuntimeError("TF unavailable"))

    with pytest.raises(RuntimeError, match="TF unavailable"):
        capture(repository, synchronizer)

    assert repository.saved == []


def test_persistence_failure_propagates(monkeypatch):
    repository = FakeRepository(save_error=OSError("write failed"))
    synchronizer = FakeSynchronizer(make_inputs())
    install_resolver(monkeypatch)

    with pytest.raises(OSError, match="write failed"):
        capture(repository, synchronizer)
