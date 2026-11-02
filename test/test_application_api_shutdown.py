"""API shutdown retains command callbacks and reports unresolved owners."""

from threading import Event, RLock
from types import SimpleNamespace
from unittest.mock import Mock

from rclpy.node import Node
from rclpy.task import Future
from fault_detector_msgs.msg import ApplicationCommandState

from fault_detector_spot.application.api.application_api_node import ApplicationApiNode
from fault_detector_spot.application.api.shutdown_callbacks import ShutdownCallbackActivity


SETUP = (
    'probe_reference_capture_api', 'probe_refinement_finalization_api',
    'probe_setup_motion_api', 'probe_setup_api', 'navigation_setup_api',
)


def make_node(monkeypatch, timeout=0.1):
    node = ApplicationApiNode.__new__(ApplicationApiNode)
    node._lock = RLock()
    node._shutdown = Event()
    node._shutdown_callbacks = ShutdownCallbackActivity()
    node._shutdown_coordinator = None
    node._shutdown_lock = RLock()
    node._shutdown_stop_request = ''
    node._shutdown_store = Mock()
    node._startup_unresolved = {}
    node._executions = {}
    node._state_publisher = Mock()
    for name in (*SETUP, 'sensor_acquisition_coordinator', 'sensor_acquisition_command_handler',
                 'probe_point_execution_api', 'sensor_acquisition_api', 'sensor_attachment_api',
                 'sensor_registry_api', 'sensor_attachment_controller', 'application_controller',
                 'command_controller', 'command_transport', 'probe_setup_motion_state',
                 'navigation_setup_state', '_operation_server'):
        setattr(node, name, Mock())
    for name in SETUP:
        getattr(node, name)._shutdown_callbacks = ShutdownCallbackActivity()
    node.command_controller.software_stop_status = SimpleNamespace(
        request_id='shutdown-request', latched=True, stop_confirmed=False, detail='Awaiting owner stop',
    )
    node.application_controller.emergency_stop.return_value = 'shutdown-request'
    node.sensor_acquisition_coordinator.shutdown_status.return_value = (True, 'Stopped')
    monkeypatch.setattr(Node, 'get_parameter', lambda _self, name: SimpleNamespace(
        value=timeout if name == 'shutdown.confirmation_timeout_sec' else 20.0,
    ))
    monkeypatch.setattr(Node, 'get_logger', lambda *args: Mock())
    monkeypatch.setattr(Node, 'destroy_node', Mock(return_value=True))
    node._failure_state = lambda execution, detail, public_state: ApplicationCommandState(
        detail=detail, state=public_state,
    )
    monkeypatch.setattr('fault_detector_spot.application.api.application_api_node.rclpy.try_shutdown', Mock())
    executor = Mock()
    executor.shutdown.return_value = True
    return node, executor


def test_callbacks_remain_available_until_stop_reply_and_terminal_action(monkeypatch):
    node, executor = make_node(monkeypatch)
    action = node.probe_setup_motion_api._shutdown_callbacks
    action.__enter__()
    progressed = Event()

    def progress(timeout_sec):
        if not node._shutdown_stop_request or progressed.is_set():
            return
        node.command_transport.close.assert_not_called()
        node.probe_setup_motion_state.close.assert_not_called()
        node.sensor_acquisition_command_handler.close.assert_not_called()
        executor.shutdown.assert_not_called()
        node.command_controller.software_stop_status.stop_confirmed = True
        action.__exit__()
        progressed.set()

    executor.spin_once.side_effect = progress
    report = node.shutdown(executor)
    assert progressed.is_set()
    assert report.confirmed
    assert not report.unresolved
    for name in SETUP:
        getattr(node, name).request_shutdown.assert_called_once()
        getattr(node, name).close.assert_called_once()
    node.sensor_acquisition_coordinator.close.assert_called_once_with(stop_unconfirmed=False)
    executor.shutdown.assert_called_once_with(timeout_sec=1.0)
    assert node.shutdown(executor) == report
    node.application_controller.emergency_stop.assert_called_once()


def test_missing_owner_reply_and_terminal_result_report_individual_resources(monkeypatch):
    node, executor = make_node(monkeypatch, timeout=0.02)
    node._shutdown_callbacks.__enter__()
    execution = SimpleNamespace(finished=Future(), state=None)
    node._executions['operation'] = execution
    node.sensor_acquisition_coordinator.shutdown_status.return_value = (False, 'Sensor reply missing')

    report = node.shutdown(executor)
    assert not report.confirmed
    assert set(report.unresolved) >= {'commands', 'operations', 'acquisition'}
    assert execution.state.state == ApplicationCommandState.STATE_STOP_UNCONFIRMED
    assert execution.finished.done()
    node._shutdown_store.record.assert_called_once_with(report.unresolved)
    node.sensor_acquisition_coordinator.close.assert_called_once_with(stop_unconfirmed=True)
    node.command_controller.software_stop_status.stop_confirmed = True
    assert not node.shutdown(executor).confirmed


def test_cleanup_error_does_not_skip_independent_cleanup_and_is_retryable(monkeypatch):
    node, executor = make_node(monkeypatch)
    node.command_controller.software_stop_status.stop_confirmed = True
    node.probe_reference_capture_api.close.side_effect = [RuntimeError('capture cleanup failed'), None]
    report = node.shutdown(executor)
    assert report.confirmed
    assert 'probe_reference_capture_api' in report.cleanup_errors
    node.navigation_setup_state.close.assert_called_once()
    executor.shutdown.assert_called_once()
    assert not node.shutdown(executor).cleanup_errors
    assert node.probe_reference_capture_api.close.call_count == 2
    node.navigation_setup_state.close.assert_called_once()


def test_late_operation_and_reset_are_rejected_after_admission_closes(monkeypatch):
    node, _ = make_node(monkeypatch)
    node._shutdown.set()
    from rclpy.action import GoalResponse
    assert node._accept_operation(Mock()) == GoalResponse.REJECT
    response = node._reset_software_stop(Mock(), SimpleNamespace())
    assert not response.accepted
    node.application_controller.reset_software_stop.assert_not_called()


def test_fresh_idle_owners_do_not_clear_retained_shutdown_uncertainty(monkeypatch):
    node, executor = make_node(monkeypatch, timeout=0.02)
    node._startup_unresolved = {'bt_runner/arm': 'Goal result missing'}
    node.command_controller.software_stop_status.stop_confirmed = True
    report = node.shutdown(executor)
    assert not report.confirmed
    assert 'prior shutdown/bt_runner/arm' in report.unresolved
    node._shutdown_store.record.assert_called_once_with({})


def test_publication_error_does_not_lose_durable_uncertainty_or_action_result(monkeypatch):
    node, executor = make_node(monkeypatch, timeout=0.02)
    execution = SimpleNamespace(finished=Future(), state=None)
    node._executions['operation'] = execution
    node._failure_state = Mock(side_effect=RuntimeError('publisher failed'))
    report = node.shutdown(executor)
    assert not report.confirmed
    node._shutdown_store.record.assert_called_once_with(report.unresolved)
    assert execution.finished.done()
    assert execution.state.state == ApplicationCommandState.STATE_STOP_UNCONFIRMED
    assert 'report stop-unconfirmed' in report.cleanup_errors
    executor.shutdown.assert_called_once()


def test_retained_shutdown_uncertainty_rejects_reset_without_contacting_bt(monkeypatch):
    node, _ = make_node(monkeypatch)
    node._startup_unresolved = {'bt_runner/arm': 'Goal result missing'}
    response = node._reset_software_stop(Mock(), SimpleNamespace())
    assert not response.accepted
    assert 'reconciliation' in response.detail
    node.application_controller.reset_software_stop.assert_not_called()


def test_navigation_api_cleanup_retries_only_failed_resource():
    from fault_detector_spot.application.api.navigation_setup_api import NavigationSetupApi
    api = NavigationSetupApi.__new__(NavigationSetupApi)
    api.node = Mock()
    api.coordinator = Mock()
    api._action_server = Mock()
    api._runtime_timer = object()
    api._runtime_subscription = object()
    api.node.destroy_timer.side_effect = [RuntimeError('timer busy'), True]
    import pytest
    with pytest.raises(RuntimeError, match='timer busy'):
        api.close()
    api.node.destroy_subscription.assert_called_once_with(api._runtime_subscription)
    api._action_server.destroy.assert_called_once()
    api.close()
    assert api.node.destroy_timer.call_count == 2
    api.node.destroy_subscription.assert_called_once()
    api.coordinator.remove_status_listener.assert_called_once()
    api._action_server.destroy.assert_called_once()


def test_capture_owner_cleanup_failure_still_releases_action_transport():
    from fault_detector_spot.application.api.probe_reference_capture_api import ProbeReferenceCaptureApi
    api = ProbeReferenceCaptureApi.__new__(ProbeReferenceCaptureApi)
    api.capture_coordinator = Mock()
    api._action_server = Mock()
    api.capture_coordinator.close.side_effect = [RuntimeError('capture busy'), None]
    import pytest
    with pytest.raises(RuntimeError, match='capture busy'):
        api.close()
    api._action_server.destroy.assert_called_once()
    api.close()
    api._action_server.destroy.assert_called_once()
    assert api.capture_coordinator.close.call_count == 2


def test_shutdown_session_marker_clears_only_after_all_cleanup_succeeds(monkeypatch):
    node, executor = make_node(monkeypatch)
    node.command_controller.software_stop_status.stop_confirmed = True
    node.probe_setup_motion_api.close.side_effect = [RuntimeError('action cleanup failed'), None]
    report = node.shutdown(executor)
    assert report.confirmed
    node._shutdown_store.confirm_session.assert_not_called()
    assert 'confirm shutdown persistence' in report.cleanup_errors
    report = node.shutdown(executor)
    assert not report.cleanup_errors
    node._shutdown_store.confirm_session.assert_called_once_with()


def test_stop_unconfirmed_retains_startup_session_marker_even_if_report_write_fails(monkeypatch):
    node, executor = make_node(monkeypatch, timeout=0.02)
    node._shutdown_store.record.side_effect = OSError('disk full')
    report = node.shutdown(executor)
    assert not report.confirmed
    assert 'report stop-unconfirmed' in report.cleanup_errors
    node._shutdown_store.confirm_session.assert_not_called()
    executor.shutdown.assert_called_once()


def test_failed_startup_marker_prevents_command_transport_or_service_admission(monkeypatch):
    from fault_detector_spot.application.api import application_api_node as module
    store = Mock()
    store.load.return_value = {}
    store.begin_session.side_effect = OSError('disk full')
    monkeypatch.setattr(Node, '__init__', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(module, 'startup_settings', Mock())
    monkeypatch.setattr(module, 'StopUnconfirmedStore', Mock(return_value=store))
    transport = Mock()
    controller = Mock()
    services = Mock()
    monkeypatch.setattr(module, 'RosCommandTransport', transport)
    monkeypatch.setattr(module, 'CommandController', controller)
    monkeypatch.setattr(Node, 'create_service', services)
    import pytest
    with pytest.raises(OSError, match='disk full'):
        ApplicationApiNode()
    store.load.assert_called_once_with()
    store.begin_session.assert_called_once_with()
    transport.assert_not_called()
    controller.assert_not_called()
    services.assert_not_called()


def test_losing_startup_marker_race_retains_admission_block_even_if_reload_is_empty(monkeypatch):
    from fault_detector_spot.application.api import application_api_node as module
    store = Mock()
    store.load.side_effect = [{}, {}]
    store.begin_session.return_value = False
    monkeypatch.setattr(Node, '__init__', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(module, 'startup_settings', Mock())
    monkeypatch.setattr(module, 'StopUnconfirmedStore', Mock(return_value=store))
    # Stop after the admission decision, before constructing ROS resources.
    monkeypatch.setattr(Node, 'declare_parameter', Mock(side_effect=RuntimeError('checkpoint')))
    node = ApplicationApiNode.__new__(ApplicationApiNode)
    import pytest
    with pytest.raises(RuntimeError, match='checkpoint'):
        node.__init__()
    assert store.load.call_count == 2
    assert 'application_api/active_session' in node._startup_unresolved
    node.application_controller = Mock()
    response = node._reset_software_stop(Mock(), SimpleNamespace())
    assert not response.accepted
    assert 'reconciliation' in response.detail
    node.application_controller.reset_software_stop.assert_not_called()
