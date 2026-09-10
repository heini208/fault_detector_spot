"""Tests for authoritative arm stow-state tracking and UI rendering."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication, QPushButton  # noqa: E402, I100

from bosdyn_api_msgs.msg import ManipulatorStateStowState  # noqa: E402, I100
from fault_detector_msgs.msg import OperationalIntent  # noqa: E402, I100
import pytest  # noqa: E402, I100

from fault_detector_spot.manipulation.arm_state_source import (  # noqa: E402, I100
    ArmStateSource,
    ArmStowState,
)
from fault_detector_spot.ui.manipulation.controls import (  # noqa: E402, I100
    ManipulationControls,
)


class _Node:

    def __init__(self):
        self.callback = None
        self.destroyed = None

    def create_subscription(self, _type, _topic, callback, _qos):
        self.callback = callback
        return object()

    def destroy_subscription(self, subscription):
        self.destroyed = subscription


class _PresentationSource:

    def __init__(self, state, stale=False, received=True):
        self.state = state
        self.stale = stale
        self.last_received_at = 1.0 if received else None

    def stow_state(self):
        return None if self.stale else self.state

    def is_stale(self):
        return self.stale


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def _message(value):
    return SimpleNamespace(stow_state=SimpleNamespace(value=value))


def test_arm_state_source_tracks_driver_state_and_freshness():
    now = [10.0]
    node = _Node()
    source = ArmStateSource(
        node,
        stale_after_sec=1.0,
        monotonic_clock=lambda: now[0],
    )

    node.callback(
        _message(ManipulatorStateStowState.STOWSTATE_STOWED)
    )
    assert source.stow_state() is ArmStowState.STOWED

    now[0] = 10.5
    node.callback(
        _message(ManipulatorStateStowState.STOWSTATE_DEPLOYED)
    )
    assert source.stow_state() is ArmStowState.DEPLOYED

    now[0] = 11.6
    assert source.stow_state() is None
    assert source.is_stale()

    source.destroy()
    assert node.destroyed is not None


def test_arm_state_button_maps_state_and_dispatches_matching_action(application):
    calls = []
    controls = object.__new__(ManipulationControls)
    controls.ui = SimpleNamespace(handle_simple_operation=calls.append)
    controls.arm_state_button = QPushButton()

    controls.arm_state_source = _PresentationSource(ArmStowState.STOWED)
    controls.refresh_arm_state()
    assert controls.arm_state_button.text() == "● Ready Arm"
    assert "#C62828" in controls.arm_state_button.styleSheet()
    assert controls.arm_state_button.isEnabled()
    controls._handle_arm_state_toggle()
    assert calls[-1] == OperationalIntent.INTENT_READY_ARM

    controls.arm_state_source = _PresentationSource(ArmStowState.DEPLOYED)
    controls.refresh_arm_state()
    assert controls.arm_state_button.text() == "● Stow Arm"
    assert "#2E7D32" in controls.arm_state_button.styleSheet()
    assert controls.arm_state_button.isEnabled()
    controls._handle_arm_state_toggle()
    assert calls[-1] == OperationalIntent.INTENT_STOW_ARM

    controls.arm_state_source = _PresentationSource(
        ArmStowState.DEPLOYED,
        stale=True,
    )
    controls.refresh_arm_state()
    assert controls.arm_state_button.text() == "● Arm state unknown"
    assert "#757575" in controls.arm_state_button.styleSheet()
    assert not controls.arm_state_button.isEnabled()
