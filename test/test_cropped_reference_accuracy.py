"""Native preview crops preserve source pixels and geometry exactly."""

import os
from copy import deepcopy
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QPoint

from test_probe_reference_preview import FakeRepository, snapshot, image
from test_reference_view_rgb_depth_mapping import make_camera_info, make_depth
from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.setup.probe_reference_preview import (
    ProbeReferencePreviewSource, _crop_preview,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion, rgb_depth_selectable_region,
)
from fault_detector_spot.inspection.setup.probe_setup_geometry import ProbeSetupGeometry
from fault_detector_spot.shared.geometry.models import PoseData
from fault_detector_spot.ui.inspection.reference_view_widget import ReferenceViewWidget


@pytest.fixture(scope="module")
def application():
    return QApplication.instance() or QApplication([])


@pytest.mark.parametrize("encoding,bpp", [("rgb8", 3), ("bgr8", 3), ("mono8", 1)])
def test_crop_preserves_pixel_bytes_and_removes_row_padding(encoding, bpp):
    original = image()
    original.encoding = encoding
    original.step = 4 * bpp + 5
    original.data = bytes(range(original.step * 3))
    saved = deepcopy(original)
    crop = _crop_preview(original, ImageRegion(1, 1, 2, 2))
    assert bytes(crop.data) == b"".join(
        bytes(original.data[row * original.step + bpp:row * original.step + 3 * bpp])
        for row in (1, 2)
    )
    assert crop.step == 2 * bpp
    assert original == saved


def test_crop_retains_isolated_depth_at_edges():
    values = np.zeros((201, 201))
    values[90:110, 90:110] = 1.0
    values[2, 3] = 1.0
    values[198, 197] = 1.0
    info = make_camera_info(201, 201)
    region = rgb_depth_selectable_region(
        (201, 201), make_depth(values.ravel(), 201, 201), info, info,
        include_sparse_support=True,
    )
    assert region == ImageRegion(3, 2, 195, 197)
    # Existing datasets still validate against their original metadata rule.
    assert rgb_depth_selectable_region(
        (201, 201), make_depth(values.ravel(), 201, 201), info, info,
    ) == ImageRegion(90, 90, 20, 20)


@pytest.mark.parametrize("rotation", [0, 90, 180, 270])
@pytest.mark.parametrize("mode", ["tag_x", "surface_fit"])
def test_cropped_selection_preserves_full_pose(application, rotation, mode):
    # RGB and depth have different calibrated resolutions, and the saved
    # camera has a non-identity pose relative to the reference tag/object.
    rgb = image(64, 48)
    depth_values = np.zeros((24, 32))
    depth_values[3:21, 4:28] = 1.2
    pose = PoseData.identity()
    pose.position.x, pose.position.y, pose.position.z = 0.3, -0.2, 0.4
    pose.orientation.z = pose.orientation.w = 2 ** -0.5
    capture = SimpleNamespace(
        reference_view=SimpleNamespace(
            view_id="slot1_hand", controlled_frame="hand_color_image_sensor",
            controlled_frame_pose_object=pose,
        ),
        camera_id="hand", slot_index=0, rgb_image=rgb,
        depth_image=make_depth(depth_values.ravel(), 32, 24),
        rgb_camera_info=make_camera_info(64, 48, 200.0),
        depth_camera_info=make_camera_info(32, 24, 100.0),
    )
    saved = deepcopy(capture)
    repository = FakeRepository(capture)
    preview = ProbeReferencePreviewSource(repository).load(snapshot(), "slot1_hand")
    region = preview.selectable_region
    assert len(preview.image.data) < len(rgb.data)
    widget = ReferenceViewWidget()
    widget.set_ros_image(preview.image, source_region=region)
    baseline_widget = ReferenceViewWidget()
    baseline_widget.set_ros_image(rgb, valid_region=region)
    for _ in range(rotation // 90):
        widget.rotate_clockwise()
        baseline_widget.rotate_clockwise()
    # The old client-side crop and new server-side crop must behave identically
    # at both small and large widget sizes, including letterboxing and dragging.
    for size in [(240, 180), (701, 503)]:
        for view in (widget, baseline_widget):
            view.resize(*size)
            view._update_display_pixmap()
        assert widget.displayed_image_rect == baseline_widget.displayed_image_rect
        for x in range(0, size[0], 11):
            for y in range(0, size[1], 13):
                position = QPoint(x, y)
                for clamp in (False, True):
                    assert widget._widget_to_image_point(position, clamp) == (
                        baseline_widget._widget_to_image_point(position, clamp)
                    )
    geometry = ProbeSetupGeometry(repository)
    # Verify every pixel, including all crop boundaries, maps exactly.
    for v in range(region.height):
        for u in range(region.width):
            display = widget._source_to_display_pixel(u, v)
            assert widget._display_to_source_point(*display) == ImagePoint(
                u=u + region.x, v=v + region.y,
            )
    for u, v in [(0, 0), (region.width - 1, region.height - 1),
                 (region.width // 2, region.height // 2)]:
        original = ImagePoint(u=u + region.x, v=v + region.y)
        restored = widget._display_to_source_point(*widget._source_to_display_pixel(u, v))
        results = [geometry.resolve(
            "motor", "scan", "slot1_hand", point, mode, 0.01, 0.1,
            PoseData.identity(),
        ) for point in (original, restored)]
        assert results[0].projected_point == results[1].projected_point
        assert results[0].surface_normal == results[1].surface_normal
        assert results[0].surface_target == results[1].surface_target
        target = results[1].surface_target
        surface = target.surface_point_object
        camera = results[1].projected_point.point_camera
        assert [surface.x, surface.y, surface.z] == pytest.approx(
            [0.3 - camera.y, -0.2 + camera.x, 0.4 + camera.z]
        )
        position = target.target_pose_object.position
        assert np.linalg.norm(np.array([position.x - surface.x,
                                       position.y - surface.y,
                                       position.z - surface.z])) == pytest.approx(0.01)
        widget.set_selected_image_point(original)
        assert widget._selected_marker_center() is not None
    assert capture.rgb_image == saved.rgb_image
    assert capture.depth_image == saved.depth_image
    assert capture.rgb_camera_info == saved.rgb_camera_info
    assert capture.depth_camera_info == saved.depth_camera_info
    assert capture.reference_view.controlled_frame_pose_object == (
        saved.reference_view.controlled_frame_pose_object
    )
