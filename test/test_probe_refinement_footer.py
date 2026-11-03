"""Footer intents reach controls while closing retains its synchronous veto."""

from unittest.mock import Mock

from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from test_probe_point_guided_workflow import FakeUI, application


def test_footer_buttons_dispatch_each_intent_once(application, monkeypatch):
    handlers = {}
    for button, method in (
        ("back_button", "handle_refinement_back"),
        ("next_button", "handle_refinement_next"),
        ("abort_button", "handle_abort_probe_refinement"),
        ("emergency_stop_button", "handle_refinement_emergency_stop"),
    ):
        handler = Mock()
        monkeypatch.setattr(FinalizingInspectionControls, method, handler)
        handlers[button] = handler

    controls = FinalizingInspectionControls(FakeUI())
    for button, handler in handlers.items():
        getattr(controls.refinement_dialog, button).click()
        handler.assert_called_once_with()


def test_close_is_vetoed_until_allowed_or_completed(application, monkeypatch):
    request_close = Mock(return_value=False)
    monkeypatch.setattr(
        FinalizingInspectionControls, "request_close_refinement_workflow", request_close,
    )
    controls = FinalizingInspectionControls(FakeUI())
    dialog = controls.refinement_dialog
    dialog.show()
    assert not dialog.close()
    assert dialog.isVisible()
    request_close.assert_called_once_with()

    request_close.return_value = True
    assert dialog.close()
    assert not dialog.isVisible()
    assert request_close.call_count == 2

    request_close.return_value = False
    dialog.show()
    dialog.close_after_completion()
    assert not dialog.isVisible()
    assert request_close.call_count == 2
