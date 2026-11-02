"""The GUI yields while bounded setup cancellation keeps ROS spinning."""

from types import SimpleNamespace
from unittest.mock import Mock

import fault_detector_spot.ui.fault_detector_ui as ui_module


def closing_ui(monkeypatch):
    now = [100.0]
    callbacks = []
    monkeypatch.setattr(ui_module, "time", SimpleNamespace(monotonic=lambda: now[0]))
    monkeypatch.setattr(ui_module, "QTimer", SimpleNamespace(
        singleShot=lambda _delay, callback: callbacks.append(callback),
    ))
    client = Mock(close_pending=True)
    ui = SimpleNamespace(
        probe_setup_client=client, navigation_setup_client=None,
        navigation_setup_timer=Mock(), probe_setup_timer=Mock(), timer=Mock(),
        manipulation_controls=Mock(), posture_state_source=Mock(),
        inspection_controls=SimpleNamespace(routine_arm_movement_controls=None),
        sensor_registry_client=None, sensor_topic_suggestion_client=None,
        sensor_attachment_client=None, sensor_head_connection_client=None,
        sensor_acquisition_client=None, micro_ros_agent_status_client=None,
        application_client=None, close=Mock(),
    )
    ui._retry_setup_close = lambda: ui_module.Fault_Detector_UI._retry_setup_close(ui)
    return ui, client, now, callbacks


def test_gui_close_yields_and_preserves_callbacks_until_completion(monkeypatch):
    ui, client, now, callbacks = closing_ui(monkeypatch)
    event = Mock()
    ui_module.Fault_Detector_UI.closeEvent(ui, event)
    ui_module.Fault_Detector_UI.closeEvent(ui, event)
    assert len(callbacks) == 1
    client.close.assert_called_once()
    client.destroy.assert_not_called()
    ui.timer.stop.assert_not_called()
    event.accept.assert_not_called()
    assert ui._setup_close_deadline == 102.0

    client.close_pending = False
    now[0] = 100.5
    ui_module.Fault_Detector_UI.closeEvent(ui, event)
    ui.timer.stop.assert_called_once()
    client.destroy.assert_called_once()
    event.accept.assert_called_once()
    client.request_uncertain.emit.assert_not_called()


def test_gui_close_deadline_is_not_extended_and_reports_unconfirmed(monkeypatch):
    ui, client, now, callbacks = closing_ui(monkeypatch)
    event = Mock()
    ui_module.Fault_Detector_UI.closeEvent(ui, event)
    now[0] = 101.5
    callbacks[0]()
    ui_module.Fault_Detector_UI.closeEvent(ui, event)
    assert ui._setup_close_deadline == 102.0
    now[0] = 102.1
    ui_module.Fault_Detector_UI.closeEvent(ui, event)
    assert "unconfirmed" in client.request_uncertain.emit.call_args.args[0]
    client.destroy.assert_called_once()
    event.accept.assert_called_once()
