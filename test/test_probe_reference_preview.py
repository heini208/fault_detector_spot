"""Tests for server-owned reference preview loading."""

from types import SimpleNamespace

from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.setup.probe_reference_preview import (
    ProbeReferencePreviewSource,
)
from fault_detector_spot.inspection.setup.probe_setup_context import (
    ProbeSetupSnapshot,
)


class FakeRepository:
    def __init__(self, capture):
        self.capture = capture
        self.requests = []

    def load_reference_views(self, object_id, routine_id):
        self.requests.append((object_id, routine_id))
        return [self.capture]


def image(width=4, height=3, encoding="rgb8"):
    message = Image()
    message.width = width
    message.height = height
    message.encoding = encoding
    message.step = width * (3 if encoding == "rgb8" else 2)
    message.data = (
        bytes(message.step * height)
        if encoding == "rgb8"
        else bytes([100, 0] * width * height)
    )
    return message


def camera_info(width=4, height=3):
    message = CameraInfo()
    message.width = width
    message.height = height
    message.k = [1.0, 0.0, 1.5, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0]
    return message


def snapshot():
    return ProbeSetupSnapshot(
        context=SimpleNamespace(),
        selected_object_id="motor",
        selected_routine_id="scan",
        selected_reference_view_id="",
        selected_reference_tag_id=7,
        selected_reference_tag_family="36h11",
        selected_sensor_id="hall_probe",
        object_ids=("motor",),
        routine_ids=("scan",),
        reference_view_ids=("slot1_hand",),
        reference_camera_ids=("hand",),
        sensor_ids=("hall_probe",),
        probe_point_ids=(),
        reference_pixel=None,
        geometry=None,
        setup=None,
        refinement=None,
        dirty=False,
        validation_error="",
    )


def test_preview_source_returns_only_rgb_and_selectable_region():
    capture = SimpleNamespace(
        reference_view=SimpleNamespace(view_id="slot1_hand"),
        camera_id="hand",
        slot_index=0,
        rgb_image=image(),
        depth_image=image(encoding="16UC1"),
        rgb_camera_info=camera_info(),
        depth_camera_info=camera_info(),
    )
    repository = FakeRepository(capture)

    preview = ProbeReferencePreviewSource(repository).load(
        snapshot(),
        "slot1_hand",
    )

    assert repository.requests == [("motor", "scan")]
    assert preview.reference_view_id == "slot1_hand"
    assert preview.camera_id == "hand"
    assert preview.slot_index == 0
    assert preview.image.width == 4
    assert preview.selectable_region.width == 4
    assert preview.selectable_region.height == 3
