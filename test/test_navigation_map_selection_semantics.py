"""Tests for navigation map selection semantics."""

from pathlib import Path
from types import SimpleNamespace

from fault_detector_msgs.msg import NavigationSetupIntent
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.coordinators.navigation_setup_coordinator import (
    MODE_MAPPING,
    NavigationSetupCoordinator,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupCoordinator,
)
from fault_detector_spot.mapping.repository.map_artifact_store import (
    MapArtifactStore,
)
from fault_detector_spot.mapping.repository.map_repository import MapRepository


class FakeCommandController:
    def __init__(self):
        self.listeners = []
        self.submitted = []

    def add_status_listener(self, listener):
        self.listeners.append(listener)

    def remove_status_listener(self, listener):
        self.listeners.remove(listener)

    def submit(self, request):
        self.submitted.append(request)
        return request.request_id

    def cancel(self, request_id):
        return request_id


def _pose():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1.0
    return pose


def _coordinator(tmp_path: Path):
    command_controller = FakeCommandController()
    setup = SetupCoordinator(command_controller)
    coordinator = NavigationSetupCoordinator(
        setup_coordinator=setup,
        map_repository=MapRepository(tmp_path),
        map_artifacts=MapArtifactStore(tmp_path),
        current_pose=_pose,
        visible_tag_pose=lambda _tag_id: _pose(),
    )
    return coordinator, command_controller


def test_select_map_while_runtime_is_stopped_is_not_a_robot_command(tmp_path):
    navigation, command_controller = _coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    created = navigation.create_map_definition(context, "plant")

    selected = navigation.select_map(created.context, "plant")

    assert selected.active_map == "plant"
    assert selected.map_names == ("plant",)
    assert command_controller.submitted == []


def test_create_and_select_map_makes_new_map_current(tmp_path):
    navigation, command_controller = _coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context

    state = navigation.create_and_select_map(context, "new_map")

    assert state.active_map == "new_map"
    assert state.map_names == ("new_map",)
    assert command_controller.submitted == []


def test_select_map_requires_runtime_to_be_stopped(tmp_path):
    navigation, _ = _coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    created = navigation.create_map_definition(context, "plant")
    navigation._mode = MODE_MAPPING

    try:
        navigation.select_map(created.context, "plant")
    except RuntimeError as exception:
        assert "Stop mapping or localization" in str(exception)
    else:
        raise AssertionError("Expected selection to reject active runtime")


def test_select_map_is_not_a_runtime_navigation_operation():
    from fault_detector_spot.application.api import navigation_setup_api

    assert (
        NavigationSetupIntent.OPERATION_SELECT_MAP
        not in navigation_setup_api._RUNTIME_OPERATIONS
    )


def test_navigation_lists_preserve_selected_destination_and_disable_empty_actions():
    from PyQt5.QtWidgets import QApplication
    from fault_detector_msgs.msg import NavigationSetupState
    from fault_detector_spot.ui.navigation.controls import NavigationControls
    app = QApplication.instance() or QApplication([])
    sent = []
    ui = SimpleNamespace(
        node=None, visible_tags=[5],
        update_tags_dropdown=lambda widget: widget.addItem('5'),
        set_navigation_mode=lambda active: None,
        execute_operation=sent.append,
    )
    controls = NavigationControls(ui)
    assert not controls.move_to_waypoint_button.isEnabled()
    assert not controls.add_landmark_button.isEnabled()
    state = NavigationSetupState()
    state.active_map = 'plant'
    state.map_names = ['plant']
    state.waypoint_names = ['entrance', 'inspection']
    state.landmark_names = ['Tag_5']
    state.mode = state.MODE_LOCALIZATION
    controls.apply_setup_state(state)
    controls.waypoint_list.setCurrentRow(1)
    controls.apply_setup_state(state)
    controls.move_to_waypoint_button.click()
    assert len(sent) == 1
    assert sent[0].map_name == 'plant'
    assert sent[0].waypoint_name == 'inspection'
    assert controls.add_landmark_button.isEnabled()
    state.active_map = ''
    state.waypoint_names = []
    state.landmark_names = []
    controls.apply_setup_state(state)
    assert not controls.move_to_waypoint_button.isEnabled()
    assert not controls.delete_waypoint_button.isEnabled()
    assert not controls.delete_landmark_button.isEnabled()
    assert not controls.add_landmark_button.isEnabled()
    assert app is not None


def test_add_current_pose_click_submits_and_keeps_save_feedback_visible():
    from PyQt5.QtCore import Qt
    from PyQt5.QtTest import QTest
    from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget
    from unittest.mock import Mock
    from fault_detector_msgs.msg import NavigationSetupState
    from fault_detector_spot.ui.navigation.controls import NavigationControls

    app = QApplication.instance() or QApplication([])
    submit = Mock(return_value='request')
    ui = SimpleNamespace(
        node=None, visible_tags=[],
        update_tags_dropdown=lambda widget: None,
        set_navigation_mode=lambda active: None,
        execute_navigation_setup=submit,
    )
    controls = NavigationControls(ui)
    widget = QWidget()
    controls.add_rows(QVBoxLayout(widget))
    state = NavigationSetupState(
        active_map='plant', map_names=['plant'],
        mode=NavigationSetupState.MODE_LOCALIZATION,
    )
    controls.apply_setup_state(state)
    controls.poi_name_field.setText(' motor_front ')
    widget.show()
    app.processEvents()
    QTest.mouseClick(controls.add_waypoint_button, Qt.LeftButton)
    intent = submit.call_args.args[0]
    assert intent.operation == NavigationSetupIntent.OPERATION_ADD_CURRENT_WAYPOINT
    assert intent.map_name == 'plant'
    assert intent.waypoint_name == 'motor_front'
    assert 'Submitting' in controls.waypoint_save_status.text()

    state.operation = intent.operation
    state.state = state.STATE_FAILED
    state.detail = 'Localization estimate is stale'
    controls.apply_setup_state(state)
    controls.update_tags_dropdown()
    assert controls.waypoint_save_status.text() == state.detail

    submit.return_value = None
    QTest.mouseClick(controls.add_waypoint_button, Qt.LeftButton)
    assert 'not submitted' in controls.waypoint_save_status.text()
    widget.close()
