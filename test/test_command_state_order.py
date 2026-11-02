"""Public command ordering must preserve request ownership across reconnects."""

from types import SimpleNamespace

from fault_detector_spot.ui.ros.command_state_order import CommandStateOrder


def state(sequence, owner="owner-a", code=3, request="request"):
    return SimpleNamespace(
        request_id=request, execution_instance_id=owner, status_sequence=sequence,
        state=code, STATE_SUCCEEDED=4, STATE_FAILED=5, STATE_CANCELLED=6,
        STATE_STOP_CONFIRMED=9,
    )


def test_owner_binds_after_admission_and_never_changes_for_that_request():
    order = CommandStateOrder()
    assert order.accept(state(1, owner="", code=1))
    assert order.accept(state(2, owner="", code=2))
    assert order.accept(state(3))
    assert not order.accept(state(4, owner="owner-b", code=9))
    assert not order.accept(state(4, owner="", code=9))
    assert order.accept(state(4, code=7))
    assert order.accept(state(5, code=9))


def test_duplicate_and_late_feedback_cannot_refresh_or_complete_a_request():
    order = CommandStateOrder()
    assert order.accept(state(5))
    assert not order.accept(state(5))
    assert not order.accept(state(4, code=9))
    assert order.accept(state(6, code=9))
    assert not order.accept(state(7, code=3))
    assert order.accept(state(1, owner="owner-b", request="next"))


def test_unsequenced_setup_snapshots_cannot_override_bound_execution():
    order = CommandStateOrder()
    assert order.accept(state(0, owner="", code=1))
    assert order.accept(state(1))
    assert not order.accept(state(0, owner="", code=5))
    assert not order.accept(state(0, code=9, request="invalid"))
