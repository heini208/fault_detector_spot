"""Mapping visualization preferences reach the existing runtime command lane."""
from types import SimpleNamespace

import py_trees
import pytest
from PyQt5.QtWidgets import QApplication
from fault_detector_msgs.msg import NavigationSetupIntent

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message, semantic_command_to_message,
)
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.mapping.behaviours.enable_slam import EnableSLAM
from fault_detector_spot.mapping.behaviours.enable_localization import EnableLocalization
from fault_detector_spot.shared.persistence.basic_settings import BasicSettingsRepository, startup_settings
from fault_detector_spot.ui.navigation.controls import NavigationControls
from test_navigation_setup_coordinator import coordinator


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('command_id,operation,leaf,method', [
    (CommandID.START_SLAM, NavigationSetupIntent.OPERATION_START_MAPPING, EnableSLAM, 'start_mapping'),
    (CommandID.START_LOCALIZATION, NavigationSetupIntent.OPERATION_START_LOCALIZATION, EnableLocalization, 'start_localization'),
])
def test_visualization_choice_reaches_runtime(tmp_path, enabled, command_id, operation, leaf, method):
    navigation, boundary = coordinator(tmp_path)
    context = navigation.open_context('ui').context
    context = navigation.create_and_select_map(context, 'plant').context
    navigation.submit_runtime_operation(context, operation, command_id, 'plant', mapping_rviz=enabled)
    command = semantic_command_from_message(semantic_command_to_message(boundary.submitted[-1].command))
    subscriber = SimpleNamespace(_create_command_stamp=lambda: None)
    execution = CommandSubscriber._map_command(subscriber, command)
    calls = []
    def launch(map_name, *, rviz):
        calls.append((map_name, rviz))
        return object()
    def begin(name, callback, *args):
        callback(*args)
        return True
    runtime = SimpleNamespace(begin_runtime_operation=begin, **{method: launch})
    behavior = leaf(runtime)
    behavior.blackboard = SimpleNamespace(active_map_name='plant', last_command=execution)
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert calls == [('plant', enabled)]


@pytest.mark.parametrize('default', [False, True])
def test_ui_startup_default_and_session_toggle_are_separate(default):
    app = QApplication.instance() or QApplication([])
    BasicSettingsRepository().save({'mapping_rviz': default})
    controls = NavigationControls.__new__(NavigationControls)
    controls.current_map = 'plant'
    intents = []
    controls.ui = SimpleNamespace(execute_navigation_setup=intents.append)
    row = controls._make_mode_row()
    assert controls.mapping_rviz.isChecked() is default
    controls.mapping_rviz.setChecked(not default)
    controls.handle_mode_mapping(True)
    controls.handle_mode_localization(True)
    assert [intent.mapping_rviz for intent in intents] == [not default, not default]
    assert startup_settings()['mapping_rviz'] is default
    assert BasicSettingsRepository().load()['mapping_rviz'] is default
    assert row.count() == 6
    assert app is not None


@pytest.mark.parametrize('mode', ['mode_mapping', 'mode_localization'])
def test_live_checkbox_submits_only_visualization_intent(mode):
    app = QApplication.instance() or QApplication([])
    controls = NavigationControls.__new__(NavigationControls)
    controls.current_map = 'plant'
    intents = []
    controls.ui = SimpleNamespace(execute_navigation_setup=intents.append)
    controls._make_mode_row()
    button = getattr(controls, mode)
    button.blockSignals(True)
    controls.mode_none.blockSignals(True)
    button.setChecked(True)
    controls.mapping_rviz.setChecked(False)
    controls.mapping_rviz.setChecked(True)
    assert [intent.operation for intent in intents] == [
        NavigationSetupIntent.OPERATION_SET_MAPPING_VISUALIZATION,
        NavigationSetupIntent.OPERATION_SET_MAPPING_VISUALIZATION,
    ]
    assert [intent.mapping_rviz for intent in intents] == [False, True]
    assert app is not None


@pytest.mark.parametrize('enabled', [False, True])
def test_live_toggle_reaches_visualization_leaf(tmp_path, enabled):
    from fault_detector_spot.mapping.behaviours.set_mapping_visualization import SetMappingVisualization
    navigation, boundary = coordinator(tmp_path)
    context = navigation.open_context('ui').context
    navigation.submit_runtime_operation(
        context, NavigationSetupIntent.OPERATION_SET_MAPPING_VISUALIZATION,
        CommandID.SET_MAPPING_VISUALIZATION, mapping_rviz=enabled,
    )
    command = semantic_command_from_message(semantic_command_to_message(boundary.submitted[-1].command))
    execution = CommandSubscriber._map_command(SimpleNamespace(_create_command_stamp=lambda: None), command)
    calls = []
    def begin(name, callback, *args):
        callback(*args)
        return True
    runtime = SimpleNamespace(begin_runtime_operation=begin,
                              set_visualization=calls.append,
                              poll_runtime_operation=lambda name: True)
    behavior = SetMappingVisualization(runtime)
    behavior.blackboard = SimpleNamespace(last_command=execution)
    behavior.initialise()
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior.update() == py_trees.common.Status.SUCCESS
    assert calls == [enabled]
