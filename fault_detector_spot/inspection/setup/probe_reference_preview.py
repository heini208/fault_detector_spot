"""Load read-only reference previews for remote probe authoring."""

from copy import deepcopy
from dataclasses import dataclass

from sensor_msgs.msg import Image

from fault_detector_spot.inspection.setup.probe_setup_context import (
    ProbeSetupSnapshot,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion,
    rgb_depth_selectable_region,
)


@dataclass(frozen=True)
class ProbeReferencePreview:
    """Contain one RGB preview and its depth-backed selection region."""

    reference_view_id: str
    camera_id: str
    slot_index: int
    image: Image
    selectable_region: ImageRegion


class ProbeReferencePreviewSource:
    """Read preview data without owning setup state or transport."""

    def __init__(self, repository):
        self.repository = repository

    def load(
        self,
        snapshot: ProbeSetupSnapshot,
        reference_view_id: str,
    ) -> ProbeReferencePreview:
        if not isinstance(snapshot, ProbeSetupSnapshot):
            raise TypeError("Expected a ProbeSetupSnapshot")
        if not snapshot.selected_object_id or not snapshot.selected_routine_id:
            raise ValueError("No inspection object and routine are selected")
        view_id = reference_view_id.strip()
        if not view_id:
            raise ValueError("Reference view ID must not be empty")
        captures = self.repository.load_reference_views(
            snapshot.selected_object_id,
            snapshot.selected_routine_id,
        )
        capture = next(
            (
                candidate
                for candidate in captures
                if candidate.reference_view.view_id == view_id
            ),
            None,
        )
        if capture is None:
            raise LookupError(f"Unknown reference view: {view_id}")
        region = rgb_depth_selectable_region(
            (capture.rgb_image.width, capture.rgb_image.height),
            capture.depth_image,
            capture.rgb_camera_info,
            capture.depth_camera_info,
        )
        return ProbeReferencePreview(
            reference_view_id=view_id,
            camera_id=capture.camera_id,
            slot_index=capture.slot_index,
            image=deepcopy(capture.rgb_image),
            selectable_region=region,
        )


__all__ = ["ProbeReferencePreview", "ProbeReferencePreviewSource"]
