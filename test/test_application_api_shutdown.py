"""API shutdown retains command callbacks and reports unresolved owners."""

from threading import Event, RLock
from types import SimpleNamespace
from unittest.mock import Mock

from rclpy.node import Node
from rclpy.task import Future
from fault_detector_msgs.msg import ApplicationCommandState

from fault_detector_spot.application.api.application_api_node import ApplicationApiNode
from fault_detector_spot.application.api.shutdown_callbacks import ShutdownCallbackActivity
from fault_detector_spot.application.api.async_operation import OperationAdmission


SETUP = (
    'probe_reference_capture_api', 'probe_refinement_finalization_api',
    'probe_setup_motion_api', 'probe_setup_api', 'navigation_setup_api',
)


def make_node(monkeypatch, timeout=0.1):
    node = ApplicationApiNode.__new__(ApplicationApiNode)
    node._lock = RLock()
    node._shutdown = Event()
    node._operation_admission = OperationAdmission()
    node._shutdown_callbacks = ShutdownCallbackActivity()
    node._shutdown_coordinator = None
    node._shutdown_lock = RLock()
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
    node.command_controller.shutdown_status.return_value = (False, 'Awaiting owner stop')
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


def test_callbacks_remain_available_until_local_actions_finish_without_robot_stop(monkeypatch):
    node, executor = make_node(monkeypatch)
    node.command_controller.shutdown_status.return_value = (True, 'Idle')
    action = node.probe_setup_motion_api._shutdown_callbacks
    action.__enter__()
    progressed = Event()

    def progress(timeout_sec):
        if not node._shutdown.is_set() or progressed.is_set():
            return
        node.command_transport.close.assert_not_called()
        node.probe_setup_motion_state.close.assert_not_called()
        node.sensor_acquisition_command_handler.close.assert_not_called()
        executor.shutdown.assert_not_called()
        node.command_controller.shutdown_status.return_value = (True, 'Idle')
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
    node.application_controller.emergency_stop.assert_not_called()


def test_idle_api_shutdown_closes_command_admission_without_requesting_robot_stop(monkeypatch):
    from fault_detector_spot.application.controllers.command_controller import CommandController
    from test_command_controller import make_request
    from fault_detector_spot.application.commanding.command_ids import CommandID
    import pytest

    node, executor = make_node(monkeypatch)
    sent = []
    node.command_controller = CommandController(dispatch_request=sent.append)

    report = node.shutdown(executor)

    assert report.confirmed
    assert sent == []
    assert not node.command_controller.software_stop_status.latched
    node.application_controller.emergency_stop.assert_not_called()
    node.sensor_acquisition_coordinator.request_shutdown.assert_called_once()
    node._shutdown_store.confirm_session.assert_called_once_with()
    with pytest.raises(RuntimeError, match='closed command admission'):
        node.command_controller.submit(make_request(CommandID.STAND_UP))


def test_active_shutdown_reports_immediately_without_cancel_and_releases_public_callback(monkeypatch):
    from time import monotonic
    from fault_detector_spot.application.controllers.command_controller import CommandController
    from fault_detector_spot.application.commanding.command_ids import CommandID
    from test_command_controller import make_request

    node, executor = make_node(monkeypatch, timeout=10.0)
    sent, persisted = [], []
    node.command_controller = CommandController(sent.append, session_work_changed=persisted.append)
    request = make_request(CommandID.STAND_UP)
    node.command_controller.submit(request)
    execution = SimpleNamespace(finished=Future())
    node._executions[request.request_id] = execution
    node._shutdown_callbacks.__enter__()
    released = []

    def progress(timeout_sec):
        if execution.finished.done() and not released:
            node._shutdown_callbacks.__exit__()
            released.append(True)

    executor.spin_once.side_effect = progress
    started = monotonic()
    report = node.shutdown(executor)
    assert monotonic() - started < 1.0
    assert set(report.unresolved) == {'commands'}
    assert execution.finished.result().state == ApplicationCommandState.STATE_UNKNOWN
    assert released == [True]
    assert sent == [request]
    assert persisted == [True]
    assert node.command_controller.active_request_id == request.request_id
    node.application_controller.emergency_stop.assert_not_called()
    node._shutdown_store.confirm_session.assert_not_called()


def test_inherited_remote_latch_does_not_make_local_shutdown_wait_or_send_stop(monkeypatch):
    from fault_detector_spot.application.controllers.command_controller import CommandController

    node, executor = make_node(monkeypatch, timeout=10.0)
    sent, persisted = [], []
    node.command_controller = CommandController(sent.append, session_work_changed=persisted.append)
    node.command_controller.observe_owner_software_stop(True, False)
    report = node.shutdown(executor)
    assert report.confirmed
    assert sent == persisted == []
    node.application_controller.emergency_stop.assert_not_called()
    node._shutdown_store.confirm_session.assert_called_once_with()


def test_shutdown_between_submit_and_cancellation_check_never_cancels_robot(monkeypatch):
    import asyncio

    node, _ = make_node(monkeypatch)
    operation = SimpleNamespace(request_id='operation')
    node.application_controller.prepare_operation.return_value = operation
    goal = SimpleNamespace(intent=object(), client_id='operator', context_id='context')
    handle = Mock(request=goal, is_cancel_requested=False)
    assert node._operation_admission.reserve(id(goal), context_id='context', lane='physical')
    assert node._operation_admission.claim(id(goal))

    def submit(_operation):
        node.request_shutdown()

    node.application_controller.submit.side_effect = submit
    result = asyncio.run(node._execute_admitted_operation(handle))
    assert result.state.state == ApplicationCommandState.STATE_UNKNOWN
    node.application_controller.cancel.assert_not_called()
    node.application_controller.emergency_stop.assert_not_called()
    handle.abort.assert_called_once_with()


def test_acquisition_state_keeps_persistent_work_after_recording_start_finishes(monkeypatch):
    from fault_detector_spot.application.controllers.command_controller import CommandController

    node, _ = make_node(monkeypatch)
    persisted = []
    node.command_controller = CommandController(session_work_changed=persisted.append)
    node.sensor_acquisition_coordinator.recording_stopped = False
    node._track_acquisition_ownership(None)
    assert persisted == [True]
    node.sensor_acquisition_coordinator.recording_stopped = True
    node._track_acquisition_ownership(None)
    assert persisted == [True, False]


def test_missing_owner_reply_and_terminal_result_report_individual_resources(monkeypatch):
    node, executor = make_node(monkeypatch, timeout=0.02)
    node._shutdown_callbacks.__enter__()
    execution = SimpleNamespace(finished=Future(), state=None)
    node._executions['operation'] = execution
    node.sensor_acquisition_coordinator.shutdown_status.return_value = (False, 'Sensor reply missing')

    report = node.shutdown(executor)
    assert not report.confirmed
    assert set(report.unresolved) >= {'commands', 'operations', 'acquisition'}
    assert execution.finished.result().state == ApplicationCommandState.STATE_UNKNOWN
    assert execution.finished.done()
    node._shutdown_store.record.assert_called_once_with(report.unresolved)
    node.sensor_acquisition_coordinator.close.assert_called_once_with(stop_unconfirmed=True)
    node.command_controller.shutdown_status.return_value = (True, 'Idle')
    assert not node.shutdown(executor).confirmed


def test_cleanup_error_does_not_skip_independent_cleanup_and_is_retryable(monkeypatch):
    node, executor = make_node(monkeypatch)
    node.command_controller.shutdown_status.return_value = (True, 'Idle')
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


def test_accepted_action_keeps_callbacks_available_before_execute_begins(monkeypatch):
    node, executor = make_node(monkeypatch)
    node.command_controller.shutdown_status.return_value = (True, 'Idle')
    goal = SimpleNamespace(context_id='context')
    handle = Mock(request=goal)
    node._abort_preparation = Mock(return_value='terminal')
    assert node._operation_admission.reserve(id(goal), context_id='context', lane='physical')

    def progress(timeout_sec):
        if not node._shutdown.is_set() or node._abort_preparation.called:
            return
        executor.shutdown.assert_not_called()
        callback = node._execute_operation(handle)
        import pytest
        with pytest.raises(StopIteration) as completed:
            callback.send(None)
        assert completed.value.value == 'terminal'

    executor.spin_once.side_effect = progress
    report = node.shutdown(executor)
    assert report.confirmed
    node._abort_preparation.assert_called_once()
    assert node._operation_admission.shutdown_status()[0]


def test_missing_accepted_callback_is_bounded_and_explicitly_unconfirmed(monkeypatch):
    node, executor = make_node(monkeypatch, timeout=0.02)
    node.command_controller.shutdown_status.return_value = (True, 'Idle')
    assert node._operation_admission.reserve('pending', context_id='context', lane='physical')
    report = node.shutdown(executor)
    assert not report.confirmed
    assert 'accepted operations' in report.unresolved
    executor.shutdown.assert_called_once()
    node._shutdown_store.confirm_session.assert_not_called()


def test_fresh_idle_owners_do_not_clear_retained_shutdown_uncertainty(monkeypatch):
    # This tests retained uncertainty, not thread scheduling latency. Allow all
    # independent owner callbacks to run before asserting only prior uncertainty.
    node, executor = make_node(monkeypatch, timeout=1.0)
    node._startup_unresolved = {'bt_runner/arm': 'Goal result missing'}
    node.command_controller.shutdown_status.return_value = (True, 'Idle')
    report = node.shutdown(executor)
    assert not report.confirmed
    assert 'prior shutdown/bt_runner/arm' in report.unresolved
    node._shutdown_store.record.assert_not_called()


def test_publication_error_does_not_lose_durable_uncertainty_or_action_result(monkeypatch):
    node, executor = make_node(monkeypatch, timeout=0.02)
    execution = SimpleNamespace(finished=Future(), state=None)
    node._executions['operation'] = execution
    node._failure_state = Mock(side_effect=RuntimeError('publisher failed'))
    report = node.shutdown(executor)
    assert not report.confirmed
    node._shutdown_store.record.assert_called_once_with(report.unresolved)
    assert execution.finished.done()
    assert execution.finished.result().state == ApplicationCommandState.STATE_UNKNOWN
    assert 'request/commands' in report.unresolved
    executor.shutdown.assert_called_once()


def test_retained_shutdown_uncertainty_rejects_reset_without_contacting_bt(monkeypatch):
    node, _ = make_node(monkeypatch)
    node._startup_unresolved = {'bt_runner/arm': 'Goal result missing'}
    response = node._reset_software_stop(Mock(), SimpleNamespace())
    assert not response.accepted
    assert 'session ownership' in response.detail
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
    node.command_controller.shutdown_status.return_value = (True, 'Idle')
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
    store.load.assert_not_called()
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
    store.load.assert_not_called()
    assert 'application_api/active_session' in node._startup_unresolved
    node.application_controller = Mock()
    response = node._reset_software_stop(Mock(), SimpleNamespace())
    assert not response.accepted
    assert 'session ownership' in response.detail
    node.application_controller.reset_software_stop.assert_not_called()


def test_fresh_startup_session_ignores_historical_markers_without_stopping(monkeypatch):
    from fault_detector_spot.application.api import application_api_node as module
    from fault_detector_spot.application.commanding.command_ids import CommandID
    from test_command_controller import make_request
    import pytest

    store = Mock()
    store.load.return_value = {'old-owner/arm': 'Old motion may still be active'}
    store.begin_session.return_value = True
    monkeypatch.setattr(Node, '__init__', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(module, 'startup_settings', Mock())
    monkeypatch.setattr(module, 'StopUnconfirmedStore', Mock(return_value=store))
    monkeypatch.setattr(Node, 'declare_parameter', Mock())
    monkeypatch.setattr(module.CommandLifecycleConfig, 'from_node', Mock(return_value=module.CommandLifecycleConfig()))
    monkeypatch.setattr(module, 'RosCommandTransport', Mock(side_effect=RuntimeError('transport checkpoint')))
    node = ApplicationApiNode.__new__(ApplicationApiNode)
    with pytest.raises(RuntimeError, match='transport checkpoint'):
        node.__init__()
    store.begin_session.assert_called_once_with()
    store.load.assert_not_called()
    assert not node._startup_unresolved
    assert not node.command_controller.software_stop_status.latched
    sent = []
    node.command_controller.configure_dispatch(sent.append)
    node.command_controller.poll()
    assert sent == []
    request = make_request(CommandID.STAND_UP)
    node.command_controller.submit(request)
    assert sent == [request]
    store.set_session_work.assert_called_once_with(True)
