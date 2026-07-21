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


def make_object() -> InspectionObject:
    """Create one object with one capture target routine."""
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
    """Create a fresh synchronized camera and tag snapshot."""
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
    """Return one configured snapshot and record tag selection."""

    def __init__(self, inputs):
        """Store the configured snapshot."""
        self.inputs = inputs
        self.requested_tag_ids = []
        self.minimum_sequences = []

    def snapshot(self, reference_tag_id, minimum_image_sequence=0):
        """Return an isolated configured snapshot."""
        self.requested_tag_ids.append(reference_tag_id)
        self.minimum_sequences.append(minimum_image_sequence)
        return deepcopy(self.inputs)


class FakeRepository:
    """Record object loads and aggregate dataset saves."""

    def __init__(self, definition=None, save_error=None):
        """Configure the stored object and optional save failure."""
        self.definition = definition or make_object()
        self.save_error = save_error
        self.loaded_object_ids = []
        self.saved = []

    def load(self, object_id):
        """Return the configured object definition."""
        self.loaded_object_ids.append(object_id)
        return self.definition

    def save_reference_dataset(self, *arguments):
        """Record or reject the aggregate save operation."""
        self.saved.append(deepcopy(arguments))
        if self.save_error is not None:
            raise self.save_error
        return self.definition


def now():
    """Return a time shortly after every valid input."""
    return SimpleNamespace(nanoseconds=10_300_000_000)


def resolved_view() -> ReferenceView:
    """Return the capture-time camera pose produced by TF."""
    return ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
    )


def install_resolver(monkeypatch, error=None):
    """Replace TF resolution with one recorded deterministic result."""
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
    """Run the standard capture operation."""
    return reference_view_capture.capture_reference_view(
        repository,
        synchronizer,
        object(),
        "motor_a",
        "magnetic_scan",
        now(),
        **keywords,
    )


def test_capture_composes_existing_operations(monkeypatch):
    """One operation resolves, validates, and persists the snapshot."""
    repository = FakeRepository()
    synchronizer = FakeSynchronizer(make_inputs())
    resolver_calls = install_resolver(monkeypatch)

    result = capture(repository, synchronizer)

    assert result is repository.definition
    assert repository.loaded_object_ids == ["motor_a"]
    assert synchronizer.requested_tag_ids == [23]
    assert synchronizer.minimum_sequences == [0]
    assert len(resolver_calls) == 1
    assert resolver_calls[0][0][1].header.frame_id == (
        "hand_color_image_sensor"
    )
    assert resolver_calls[0][0][2].id == 23
    assert resolver_calls[0][1] == {
        "fixed_frame": "odom",
        "transform_timeout_sec": 0.05,
    }
    saved = repository.saved[0]
    assert saved[0:2] == ("motor_a", "magnetic_scan")
    assert saved[2] == resolved_view()
    assert saved[3].header.stamp.nanosec == 200_000_000
    assert saved[4].header.stamp.nanosec == 210_000_000
    assert saved[5].header.frame_id == "hand_color_image_sensor"


def test_unavailable_snapshot_is_rejected_before_tf(monkeypatch):
    """Capture cannot proceed without every synchronized input."""
    repository = FakeRepository()
    synchronizer = FakeSynchronizer(None)
    resolver_calls = install_resolver(monkeypatch)

    with pytest.raises(RuntimeError, match="unavailable for tag 23"):
        capture(repository, synchronizer)

    assert resolver_calls == []
    assert repository.saved == []


def test_missing_routine_is_rejected_before_snapshot(monkeypatch):
    """Capture must target a routine owned by the selected object."""
    definition = make_object()
    definition.routines = []
    repository = FakeRepository(definition=definition)
    synchronizer = FakeSynchronizer(make_inputs())
    resolver_calls = install_resolver(monkeypatch)

    with pytest.raises(KeyError, match="Routine does not exist"):
        capture(repository, synchronizer)

    assert synchronizer.requested_tag_ids == []
    assert resolver_calls == []
    assert repository.saved == []


def test_existing_reference_view_requires_explicit_replacement(
    monkeypatch,
):
    """Recapture cannot replace persisted teaching data by default."""
    definition = make_object()
    definition.routines[0].reference_view = resolved_view()
    repository = FakeRepository(definition=definition)
    synchronizer = FakeSynchronizer(make_inputs())
    resolver_calls = install_resolver(monkeypatch)

    with pytest.raises(FileExistsError, match="already has"):
        capture(repository, synchronizer)

    assert synchronizer.requested_tag_ids == []
    assert resolver_calls == []
    assert repository.saved == []


def test_existing_reference_view_can_be_replaced_explicitly(
    monkeypatch,
):
    """The explicit replacement flag permits a deliberate recapture."""
    definition = make_object()
    definition.routines[0].reference_view = resolved_view()
    repository = FakeRepository(definition=definition)
    synchronizer = FakeSynchronizer(make_inputs())
    install_resolver(monkeypatch)

    capture(repository, synchronizer, replace_existing=True)

    assert synchronizer.requested_tag_ids == [23]
    assert len(repository.saved) == 1


def test_capture_requires_the_requested_image_sequence(monkeypatch):
    """The command behavior can reject frames cached before its request."""
    repository = FakeRepository()
    synchronizer = FakeSynchronizer(make_inputs())
    install_resolver(monkeypatch)

    capture(
        repository,
        synchronizer,
        minimum_image_sequence=4,
    )

    assert synchronizer.minimum_sequences == [4]


@pytest.mark.parametrize(
    "input_index, name",
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
    """Cached dynamic inputs expire independently."""
    inputs = list(make_inputs())
    target = inputs[input_index]
    header = target.pose.header if input_index == 3 else target.header
    header.stamp.sec = 9
    header.stamp.nanosec = 0
    repository = FakeRepository()
    resolver_calls = install_resolver(monkeypatch)

    with pytest.raises(RuntimeError, match=f"{name} is stale"):
        capture(repository, FakeSynchronizer(tuple(inputs)))

    assert resolver_calls == []
    assert repository.saved == []


def test_future_input_is_rejected_before_tf(monkeypatch):
    """A clock mismatch cannot silently become a valid capture."""
    inputs = list(make_inputs())
    inputs[0].header.stamp.sec = 11
    repository = FakeRepository()
    resolver_calls = install_resolver(monkeypatch)

    with pytest.raises(ValueError, match="timestamp is in the future"):
        capture(repository, FakeSynchronizer(tuple(inputs)))

    assert resolver_calls == []
    assert repository.saved == []


@pytest.mark.parametrize("maximum_age", [-0.01, float("inf")])
def test_invalid_freshness_configuration_is_rejected(maximum_age):
    """Invalid freshness limits fail before repository access."""
    repository = FakeRepository()
    synchronizer = FakeSynchronizer(make_inputs())

    with pytest.raises(ValueError, match="Maximum input age"):
        capture(
            repository,
            synchronizer,
            maximum_input_age_sec=maximum_age,
        )

    assert repository.loaded_object_ids == []
    assert synchronizer.requested_tag_ids == []


def test_tf_failure_prevents_validation_and_persistence(monkeypatch):
    """A missing capture-time transform leaves storage untouched."""
    repository = FakeRepository()
    synchronizer = FakeSynchronizer(make_inputs())
    install_resolver(monkeypatch, RuntimeError("TF unavailable"))
    validation_calls = []
    monkeypatch.setattr(
        reference_view_capture,
        "validate_reference_view_inputs",
        lambda *args, **kwargs: validation_calls.append(args),
    )

    with pytest.raises(RuntimeError, match="TF unavailable"):
        capture(repository, synchronizer)

    assert validation_calls == []
    assert repository.saved == []


def test_validation_failure_prevents_persistence(monkeypatch):
    """Invalid resolved inputs cannot reach aggregate storage."""
    repository = FakeRepository()
    synchronizer = FakeSynchronizer(make_inputs())
    install_resolver(monkeypatch)

    def fail_validation(*arguments, **keywords):
        raise ValueError("invalid capture")

    monkeypatch.setattr(
        reference_view_capture,
        "validate_reference_view_inputs",
        fail_validation,
    )

    with pytest.raises(ValueError, match="invalid capture"):
        capture(repository, synchronizer)

    assert repository.saved == []


def test_persistence_failure_propagates_without_mutating_object(
    monkeypatch,
):
    """A failed aggregate save does not mutate the loaded definition."""
    definition = make_object()
    before = deepcopy(definition)
    repository = FakeRepository(
        definition=definition,
        save_error=OSError("write failed"),
    )
    install_resolver(monkeypatch)

    with pytest.raises(OSError, match="write failed"):
        capture(repository, FakeSynchronizer(make_inputs()))

    assert definition == before
    assert len(repository.saved) == 1
