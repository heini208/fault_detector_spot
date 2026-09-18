import inspect

from fault_detector_spot.application.api.probe_setup_api import ProbeSetupApi
from fault_detector_spot.application.coordinators.probe_setup_coordinator import (
    ProbeSetupCoordinator,
)
from fault_detector_spot.inspection.repository.object_repository import (
    ObjectRepository,
)
from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)


def test_saved_probe_delete_path_exists_end_to_end():
    assert hasattr(ObjectRepository, "delete_probe_point")
    assert hasattr(ProbeSetupCoordinator, "delete_probe_point")
    assert hasattr(ProbeSetupApi, "_delete_probe_point")
    assert hasattr(
        FinalizingInspectionControls,
        "handle_delete_saved_probe_point",
    )


def test_saved_probe_selection_is_explicitly_highlighted():
    source = inspect.getsource(
        FinalizingInspectionControls._configure_probe_point_entry_ui
    )
    assert "Delete Selected Probe Point" in source
    assert "Selected probe point: none" in source
    assert "QListWidget::item:selected" in source
    assert "font-weight: bold" in source
