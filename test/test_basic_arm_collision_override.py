"""A one-shot UI override is consumed by the next basic movement submission."""

from concurrent.futures import Future
from copy import deepcopy
import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication, QLabel, QWidget  # noqa: E402
from fault_detector_msgs.msg import (  # noqa: E402
    ApplicationCommandState,
    OperationalIntent,
)
import pytest  # noqa: E402

from fault_detector_spot.ui.manipulation.controls import (  # noqa: E402
    ManipulationControls,
)
from fault_detector_spot.ui.ros import application_client  # noqa: E402


@pytest.fixture(scope="module")
def application():
    return QApplication.instance() or QApplication([])


class ActionTransport:
    def __init__(self, *_args):
        self.ready = True
        self.requests = []
        self.error = None

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, *, feedback_callback):
        if self.error is not None:
            raise self.error
        future = Future()
        self.requests.append((deepcopy(goal), feedback_callback, future))
        return future


@pytest.fixture
def controls(application, monkeypatch):
    monkeypatch.setattr(application_client, "ActionClient", ActionTransport)
    node = SimpleNamespace(
        create_client=lambda *_: object(),
        create_subscription=lambda *_: object(),
    )
    client = application_client.ApplicationClient(node, "test-ui")
    parent = QWidget()
    parent.node = None
    parent.status_label = QLabel()
    parent.application_client = client
    parent.update_tags_dropdown = lambda dropdown: dropdown.addItem("7")
    parent.update_frames_dropdown = lambda dropdown: dropdown.addItem("body")
    parent.execute_operation = client.execute
    value = ManipulationControls(parent)
    yield value, client, application
    value.destroy()
    parent.close()


def intent(kind=OperationalIntent.INTENT_MOVE_ARM_RELATIVE):
    result = OperationalIntent()
    result.intent = kind
    return result


def state(kind, request_id="server-request"):
    result = ApplicationCommandState()
    result.request_id = request_id
    result.state = kind
    return result


def feedback(client, index, kind):
    callback = client._operation_client.requests[index][1]
    callback(SimpleNamespace(feedback=SimpleNamespace(state=state(kind))))


def accept(client, index=0):
    result_future = Future()
    client._operation_client.requests[index][2].set_result(
        SimpleNamespace(accepted=True, get_result_async=lambda: result_future)
    )
    return result_future


def test_double_click_consumes_once_before_any_server_response(controls):
    value, client, application = controls
    value.ignore_environment_checkbox.setChecked(True)
    first = value.execute_basic_operation(intent())
    second = value.execute_basic_operation(intent())
    assert first != second
    assert not value.ignore_environment_checkbox.isChecked()
    assert [request[0].intent.ignore_environment_collisions
            for request in client._operation_client.requests] == [True, False]
    result_future = accept(client)
    feedback(client, 0, ApplicationCommandState.STATE_QUEUED)
    application.processEvents()
    result_future.set_result(SimpleNamespace(result=SimpleNamespace(
        state=state(ApplicationCommandState.STATE_FAILED),
    )))
    application.processEvents()
    assert not value.ignore_environment_checkbox.isChecked()


@pytest.mark.parametrize("rejection", ["unavailable", "goal_rejected"])
def test_rejected_submission_leaves_choice_cleared(controls, rejection):
    value, client, application = controls
    value.ignore_environment_checkbox.setChecked(True)
    client._operation_client.ready = rejection != "unavailable"
    value.execute_basic_operation(intent())
    if rejection == "goal_rejected":
        client._operation_client.requests[0][2].set_result(
            SimpleNamespace(accepted=False)
        )
    application.processEvents()
    assert not value.ignore_environment_checkbox.isChecked()


@pytest.mark.parametrize("failure", ["send", "goal_reply", "terminal_without_feedback"])
def test_uncertain_submission_does_not_restore_choice(controls, failure):
    value, client, application = controls
    value.ignore_environment_checkbox.setChecked(True)
    if failure == "send":
        client._operation_client.error = RuntimeError("transport failed")
    value.execute_basic_operation(intent())
    if failure == "goal_reply":
        client._operation_client.requests[0][2].set_exception(
            RuntimeError("reply lost")
        )
    elif failure == "terminal_without_feedback":
        result_future = accept(client)
        result_future.set_result(SimpleNamespace(result=SimpleNamespace(
            state=state(ApplicationCommandState.STATE_FAILED),
        )))
    application.processEvents()
    assert not value.ignore_environment_checkbox.isChecked()


def test_late_rejection_does_not_change_new_checkbox_choice(controls):
    value, client, application = controls
    value.ignore_environment_checkbox.setChecked(True)
    value.execute_basic_operation(intent())
    value.ignore_environment_checkbox.setChecked(True)
    client._operation_client.requests[0][2].set_result(
        SimpleNamespace(accepted=False)
    )
    application.processEvents()
    assert value.ignore_environment_checkbox.isChecked()


def test_rechecking_applies_to_future_request_only(controls):
    value, client, application = controls
    value.ignore_environment_checkbox.setChecked(True)
    first = value.execute_basic_operation(intent())
    value.ignore_environment_checkbox.setChecked(True)
    second = value.execute_basic_operation(intent())
    feedback(client, 0, ApplicationCommandState.STATE_RUNNING)
    application.processEvents()
    assert first != second
    assert all(request[0].intent.ignore_environment_collisions
               for request in client._operation_client.requests)
    assert not value.ignore_environment_checkbox.isChecked()


@pytest.mark.parametrize("kind", [
    OperationalIntent.INTENT_WAIT,
    OperationalIntent.INTENT_TOGGLE_GRIPPER,
    OperationalIntent.INTENT_MOVE_BASE_RELATIVE,
    OperationalIntent.INTENT_MOVE_SAVED_PROBE_SAFE_APPROACH,
    OperationalIntent.INTENT_READY_ARM,
])
def test_unrelated_operations_do_not_consume_choice(controls, kind):
    value, client, _application = controls
    value.ignore_environment_checkbox.setChecked(True)
    value.execute_basic_operation(intent(kind))
    assert value.ignore_environment_checkbox.isChecked()
    assert not client._operation_client.requests[0][0].intent.ignore_environment_collisions
