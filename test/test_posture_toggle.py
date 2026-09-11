"""Offline coverage of posture freshness and both synchronized buttons."""
import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QWidget
from fault_detector_msgs.msg import OperationalIntent
from spot_msgs.msg import Feedback

from fault_detector_spot.navigation.posture_state_source import (
    PostureState, PostureStateSource,
)
from fault_detector_spot.ui.shared.posture_toggle import PostureToggle


class Node:
    def create_subscription(self, message_type, topic, callback, qos):
        assert message_type is Feedback
        assert topic == "status/feedback"
        self.receive = callback
        self.subscription = object()
        return self.subscription

    def destroy_subscription(self, subscription):
        assert subscription is self.subscription
        self.destroyed = True


@pytest.fixture
def source():
    clock = SimpleNamespace(now=0.0)
    node = Node()
    source = PostureStateSource(node, monotonic_clock=lambda: clock.now)
    return source, node, clock


def test_posture_freshness_and_cleanup(source):
    state, node, clock = source
    assert state.posture() is None
    node.receive(Feedback(sitting=True))
    assert state.posture() is PostureState.SITTING
    node.receive(Feedback(standing=True))
    assert state.posture() is PostureState.STANDING
    for message in (Feedback(), Feedback(standing=True, sitting=True),
                    Feedback(sitting=True, moving=True)):
        node.receive(message)
        assert state.posture() is PostureState.UNKNOWN
    node.receive(Feedback(standing=True))
    clock.now = 2.0
    assert state.posture() is None
    state.destroy()
    state.destroy()
    assert node.destroyed


def test_both_buttons_follow_feedback_and_recheck_before_dispatch(source):
    app = QApplication.instance() or QApplication([])
    state, node, clock = source
    ui = QWidget()
    commands = []
    ui.handle_simple_operation = commands.append
    buttons = [PostureToggle(ui, state), PostureToggle(ui, state)]
    assert all(not button.isEnabled() for button in buttons)
    for sitting, intent, label in (
        (True, OperationalIntent.INTENT_STAND_UP, "Sitting"),
        (False, OperationalIntent.INTENT_SIT_DOWN, "Standing"),
    ):
        node.receive(Feedback(sitting=sitting, standing=not sitting))
        for button in buttons:
            button.refresh()
            assert label in button.text()
            button.click()
            assert commands[-1] == intent
    count = len(commands)
    clock.now = 2.0
    buttons[0]._toggle()
    assert len(commands) == count
    assert not buttons[0].isEnabled()
    for button in buttons:
        button.timer.stop()
    ui.close()


@pytest.mark.parametrize("sitting,success", [(True, True), (False, True), (True, False)])
def test_generic_driver_commands_register_posture_monitor(sitting, success):
    from bosdyn.client.robot_command import RobotCommandBuilder
    from spot_wrapper.wrapper import SpotWrapper

    wrapper = SimpleNamespace(
        _robot=SimpleNamespace(time_sync=SimpleNamespace(endpoint=None)),
        _robot_command=lambda *args, **kwargs: (success, "result", 42),
        is_standing=True, is_sitting=False,
        last_stand_command=7, last_sit_command=None,
    )
    command = (RobotCommandBuilder.synchro_sit_command() if sitting
               else RobotCommandBuilder.synchro_stand_command())
    result = SpotWrapper.robot_command(wrapper, command)
    assert result == (success, "result", 42)
    if success:
        assert not wrapper.is_standing and not wrapper.is_sitting
        assert wrapper.last_sit_command == (42 if sitting else None)
        assert wrapper.last_stand_command == (None if sitting else 42)
    else:
        assert wrapper.is_standing
        assert wrapper.last_stand_command == 7
