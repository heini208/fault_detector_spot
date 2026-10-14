"""Offline checks for advanced persistence, execution overrides and launch inputs."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from PyQt5.QtCore import QPoint, QPointF, Qt
from PyQt5.QtGui import QWheelEvent
from PyQt5.QtWidgets import QApplication

from fault_detector_spot.shared.persistence.basic_settings import (
    BasicSettingsRepository, factory_advanced_settings, startup_settings, _startup_settings,
)
from fault_detector_spot.shared.persistence.advanced_settings import KEYS
from fault_detector_spot.shared.persistence.settings_parameter_files import settings_parameter_file
from fault_detector_spot.ui.settings.basic_settings_page import BasicSettingsPage
from fault_detector_spot.ui.settings.advanced_settings_page import AdvancedSettingsPage
from fault_detector_spot.manipulation.arm_motion_parameters import ArmMotionParameters
from fault_detector_spot.manipulation.force_contact_policy import SpeedAwareForceContactPolicy
from fault_detector_spot.manipulation.move_close_to_surface_execution import MoveCloseToSurfaceConfig
from fault_detector_spot.manipulation.pose_precision import PoseCorrectionBudget
from fault_detector_spot.navigation.base_goal_verifier import BaseGoalVerificationConfig
from fault_detector_spot.navigation.base_correction_policy import BaseCorrectionConfig
from fault_detector_spot.sensing.observations.tag_observation_stability import TagObservationStabilityConfig
from test_arm_motion_parameters import FakeNode
from test_moveit_sensor_launch import launch_parameters, expand_configuration

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def isolated_settings(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros'))
    _startup_settings.cache_clear()
    yield
    _startup_settings.cache_clear()


@pytest.fixture
def app():
    return QApplication.instance() or QApplication([])


def test_both_tabs_preserve_other_saved_values_and_roundtrip_units(app):
    repo = BasicSettingsRepository()
    basic = BasicSettingsPage(repository=repo)
    advanced = AdvancedSettingsPage(repository=repo)
    assert set(advanced.fields) == KEYS
    advanced.fields['base.goal_verification.yaw_tolerance_rad'].setValue(6.)
    advanced.fields['base.correction.maximum_attempts'].setValue(0)
    advanced.fields['arm.contact.telemetry.raw_logging_enabled'].setChecked(False)
    advanced.save()
    saved = repo.load()
    assert saved['base.goal_verification.yaw_tolerance_rad'] == pytest.approx(.1047197551)
    assert saved['base.correction.maximum_attempts'] == 0
    assert type(saved['base.correction.maximum_attempts']) is int
    assert saved['arm.contact.telemetry.raw_logging_enabled'] is False
    basic.fields['arm_mode'].setCurrentIndex(1)
    basic.save()
    assert all(repo.load()[key] == value for key, value in saved.items())
    advanced.save()
    assert repo.load()['arm_mode'] == 'precision'
    advanced._show_values(advanced.defaults)
    assert repo.load()['base.correction.maximum_attempts'] == 0
    basic.close()
    advanced.close()


@pytest.mark.parametrize('values', [
    {'arm.contact.minimum_force_delta_threshold_n': 8.},
    {'arm.contact.reference_force_delta_threshold_n': 11.},
    {'close_surface.approach_near_speed_mps': .2},
    {'base.correction.maximum_attempts': -1},
    {'arm.correction.maximum_attempts': 1.5},
    {'base.tag_correction.stability.required_samples': 1},
    {'arm.motion.moveit_velocity_scaling': 1.1},
    {'arm.motion.moveit_acceleration_scaling': 0},
    {'moveit.octomap_resolution_m': float('nan')},
    {'arm.contact.telemetry.raw_logging_enabled': 1},
    {'navigation.reverse_speed_mps': -.3},
])
def test_invalid_advanced_values_leave_saved_file_intact(values):
    repo = BasicSettingsRepository()
    repo.save({'arm_mode': 'precision'})
    previous = repo.path.read_bytes()
    with pytest.raises(ValueError):
        repo.save(values)
    assert repo.path.read_bytes() == previous


def test_advanced_parameters_reach_existing_owners():
    values = {
        'arm.contact.minimum_force_delta_threshold_n': 4.,
        'arm.contact.reference_force_delta_threshold_n': 6.,
        'arm.contact.maximum_force_delta_threshold_n': 12.,
        'arm.contact.retreat_distance_m': .02,
        'arm.contact.retreat_speed_mps': .005,
        'arm.contact.telemetry.raw_logging_enabled': False,
        'arm.motion.moveit_velocity_scaling': .3,
        'arm.motion.moveit_acceleration_scaling': .2,
        'arm.correction.maximum_attempts': 0,
        'base.goal_verification.position_tolerance_m': .04,
        'base.goal_verification.yaw_tolerance_rad': .04,
        'base.correction.maximum_attempts': 0,
        'base.tag_correction.stability.required_samples': 5,
        'base.tag_correction.stability.maximum_position_span_m': .01,
        'base.tag_correction.stability.maximum_yaw_span_rad': .08,
        'base.tag_correction.stability.maximum_sample_span_sec': 3.,
        'close_surface.approach_far_speed_mps': .008,
        'close_surface.approach_near_speed_mps': .001,
        'close_surface.approach_slowdown_distance_m': .04,
        'close_surface.contact_search_force_threshold_n': 1.5,
    }
    BasicSettingsRepository().save(values)
    config = ArmMotionParameters(FakeNode())
    for key, value in values.items():
        if key.startswith('arm.'):
            assert config.get(key[4:]) == value
    policy = SpeedAwareForceContactPolicy(config=config)
    assert policy.threshold_for(0.) == 4.
    assert policy.threshold_for(.005) == 6.
    assert policy.threshold_for(1.) == 12.
    base = BaseGoalVerificationConfig.from_node(FakeNode())
    assert base.position_tolerance_m == base.yaw_tolerance_rad == .04
    assert BaseCorrectionConfig.from_node(FakeNode()).maximum_attempts == 0
    tags = TagObservationStabilityConfig.from_node(FakeNode())
    for key, value in values.items():
        if key.startswith('base.tag_correction.stability.'):
            assert getattr(tags, key.rsplit('.', 1)[1]) == value
    surface = MoveCloseToSurfaceConfig.from_node(FakeNode())
    for key, value in values.items():
        if key.startswith('close_surface.'):
            assert getattr(surface, key.split('.', 1)[1]) == value


def test_zero_arm_corrections_retains_error_and_travel_checks():
    budget = PoseCorrectionBudget(maximum_corrections=0)
    with pytest.raises(ValueError, match='limit reached'):
        budget.reserve(.01, 0., .005, .03)
    assert budget.corrections == 0


@pytest.mark.parametrize('kind,filename,root,values,paths', [
    ('moveit', 'moveit_sensors.yaml', '/move_group',
     {'moveit.octomap_resolution_m': .025, 'moveit.lidar_max_range_m': 4.},
     [('octomap_resolution',), ('lidar', 'max_range')]),
    ('navigation', 'nav2_lidar_params.yaml', 'controller_server',
     {'navigation.forward_speed_mps': .2, 'navigation.reverse_speed_mps': .1,
      'navigation.sideways_speed_mps': .15, 'navigation.turning_speed_rad_s': .4,
      'navigation.position_tolerance_m': .07, 'navigation.heading_tolerance_rad': .08},
     [('FollowPath', 'vx_max'), ('FollowPath', 'vx_min'), ('FollowPath', 'vy_max'),
      ('FollowPath', 'wz_max'), ('general_goal_checker', 'xy_goal_tolerance'),
      ('general_goal_checker', 'yaw_goal_tolerance')]),
])
def test_generated_parameter_files_override_only_selected_values(kind, filename, root, values, paths):
    source = ROOT / 'config' / filename
    original = source.read_bytes()
    assert settings_parameter_file(source, kind, {}) == str(source)
    output = settings_parameter_file(source, kind, values)
    assert output == settings_parameter_file(source, kind, values)
    actual = yaml.safe_load(Path(output).read_text())
    expected = yaml.safe_load(original)
    for (key, value), path in zip(values.items(), paths):
        target = expected[root]['ros__parameters']
        for part in path[:-1]:
            target = target[part]
        target[path[-1]] = -value if key == 'navigation.reverse_speed_mps' else value
    if kind == 'navigation':
        smoother = expected['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [.2, .15, .4]
        smoother['min_velocity'] = [-.1, -.15, -.4]
    assert actual == expected
    assert source.read_bytes() == original


def test_navigation_manager_uses_frozen_settings_without_starting_process(monkeypatch):
    from fault_detector_spot.navigation.runtime.nav2_runtime_manager import Nav2RuntimeManager
    BasicSettingsRepository().save({'navigation.forward_speed_mps': .23})
    manager = Nav2RuntimeManager(Mock(), Mock(), params_file=str(ROOT / 'config/nav2_lidar_params.yaml'))
    manager._launch = Mock()
    BasicSettingsRepository().save({'navigation.forward_speed_mps': .7})
    manager.start()
    args = manager._launch.call_args.args[1]
    output = next(value.split(':=', 1)[1] for value in args if value.startswith('params_file:='))
    assert yaml.safe_load(Path(output).read_text())['controller_server']['ros__parameters']['FollowPath']['vx_max'] == .23


def test_advanced_wheel_events_do_not_edit_values_even_when_focused(app):
    page = AdvancedSettingsPage()
    page.show()
    for key in ('arm.correction.maximum_attempts', 'arm.motion.moveit_velocity_scaling'):
        field = page.fields[key]
        field.setFocus()
        app.processEvents()
        previous = field.value()
        wheel = QWheelEvent(QPointF(5, 5), QPointF(5, 5), QPoint(), QPoint(0, 120),
                            Qt.NoButton, Qt.NoModifier, Qt.NoScrollPhase, False)
        QApplication.sendEvent(field, wheel)
        assert not wheel.isAccepted()
        assert field.value() == previous
    page.close()


def test_invalid_form_reports_problem_without_overwriting_saved_values(app):
    repo = BasicSettingsRepository()
    repo.save(factory_advanced_settings())
    previous = repo.path.read_bytes()
    page = AdvancedSettingsPage(repository=repo)
    page.fields['arm.contact.minimum_force_delta_threshold_n'].setValue(15.)
    page.save()
    assert 'minimum' in page.status.text()
    assert repo.path.read_bytes() == previous
    page.close()


def test_precision_execution_honours_zero_correction_setting(monkeypatch):
    from test_arm_precision_execution import precision_rig, finish_motion, stable_feedback
    from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome
    BasicSettingsRepository().save({'arm.correction.maximum_attempts': 0})
    value = precision_rig(monkeypatch)
    assert value.executor._tag_accuracy['budget'].maximum_corrections == 0
    finish_motion(value)
    update = stable_feedback(value, position=.99)
    assert update.outcome is ArmMovementOutcome.MOTION_FAILED
    assert 'limit reached' in update.detail
    assert len(value.client.sent_goals) == 1


def test_normal_tag_execution_honours_zero_correction_setting(monkeypatch):
    from test_arm_movement_executor import tag_verification_executor
    from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome
    BasicSettingsRepository().save({'arm.correction.maximum_attempts': 0})
    executor, feedback, _, corrections = tag_verification_executor(monkeypatch)
    feedback.header.stamp.sec += 1
    update = executor.poll()
    assert update.outcome is ArmMovementOutcome.MOTION_FAILED
    assert not corrections


def test_arm_editor_defaults_reach_command_without_overriding_user_tolerance(app, monkeypatch):
    from fault_detector_msgs.msg import OperationalIntent
    from fault_detector_spot.ui.manipulation.controls import ManipulationControls
    from test_probe_precision_settings import _Parent
    BasicSettingsRepository().save({
        'arm.motion.position_tolerance_m': .023,
        'arm.motion.orientation_tolerance_rad': .07,
    })
    controls = object.__new__(ManipulationControls)
    controls.ui = _Parent([2])
    controls._make_tag_input_row()
    controls._make_precision_mode_row()
    monkeypatch.setattr(controls, 'add_tag_element_to_intent', lambda intent, *_: intent)
    monkeypatch.setattr(controls, 'add_offset_to_intent', lambda intent: intent)
    assert controls.tag_tolerance_input.value() == .023
    controls.tag_tolerance_input.setValue(.015)
    intent = controls.add_tag_info_to_intent(OperationalIntent())
    assert intent.tag_position_tolerance_m == .015
    assert intent.precision_orientation_tolerance_rad == .07


def test_saved_moveit_map_settings_reach_move_group_only(launch_parameters):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, GroupAction
    module, _, captured, _, _ = launch_parameters
    BasicSettingsRepository().save({'moveit.octomap_resolution_m': .025, 'moveit.lidar_max_range_m': 4.})
    description = module.generate_launch_description()
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    group, = [entity for entity in description.entities if isinstance(entity, GroupAction)]
    expand_configuration(group, context)
    documents = captured[0][2]
    parameters = documents[0]['/move_group']['ros__parameters']
    assert parameters['octomap_resolution'] == .025
    assert parameters['lidar']['max_range'] == 4.
    assert list(documents[0]) == ['/move_group']
