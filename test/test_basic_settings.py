"""Persistent basic preferences reach existing execution owners only at startup."""

from types import SimpleNamespace

import pytest
from PyQt5.QtWidgets import QApplication

from fault_detector_spot.shared.persistence.basic_settings import (
    BasicSettingsRepository, factory_settings, startup_settings, _startup_settings,
)
from fault_detector_spot.manipulation.arm_motion_parameters import ArmMotionParameters
from fault_detector_spot.manipulation.force_contact_policy import SpeedAwareForceContactPolicy
from fault_detector_spot.navigation.walking_profile import WalkingProfiles
from fault_detector_spot.ui.settings.basic_settings_page import BasicSettingsPage
from test_arm_motion_parameters import FakeNode
from test_arm_collision_control import make_control
from test_sensor_attachment_controller import build_controller, sensor_definition
from fault_detector_spot.application.controllers.sensor_attachment_controller import (
    SensorAttachmentController, SensorAttachmentStatus,
)
from fault_detector_spot.inspection.repository.sensor_attachment_state_store import PersistedSensorAttachmentSelection
from fault_detector_msgs.msg import SensorAttachmentState


@pytest.fixture(autouse=True)
def isolated_settings(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros'))
    _startup_settings.cache_clear()
    yield
    _startup_settings.cache_clear()


def test_roundtrip_and_startup_snapshot():
    repo = BasicSettingsRepository()
    assert startup_settings() == {}
    values = factory_settings()
    values.update(arm_mode='precision', mount_approval_at_startup=True, force_threshold_percent=250.)
    repo.save(values)
    assert repo.load() == values
    assert startup_settings() == {}  # Saving does not reconfigure this process.
    _startup_settings.cache_clear()  # Simulate next process start.
    assert dict(startup_settings()) == values


@pytest.mark.parametrize('values', [
    {'arm_mode': 'fast'}, {'mount_approval_at_startup': 1},
    {'force_threshold_percent': 0}, {'force_threshold_percent': float('nan')},
    {'arm_normal_linear_speed_mps': -1}, {'base_normal_linear_speed_mps': True},
    {'unknown': 1},
])
def test_invalid_save_preserves_previous_file(values):
    repo = BasicSettingsRepository()
    repo.save({'arm_mode': 'normal'})
    before = repo.path.read_bytes()
    with pytest.raises(ValueError):
        repo.save(values)
    assert repo.path.read_bytes() == before


def test_corrupt_settings_are_not_silently_replaced():
    repo = BasicSettingsRepository()
    repo.path.parent.mkdir(parents=True)
    repo.path.write_text('version: 2\nsettings: {}\n')
    with pytest.raises(ValueError, match='Unsupported'):
        repo.load()


def test_saved_arm_speeds_override_yaml_and_ros_but_not_explicit_values():
    BasicSettingsRepository().save({
        'arm_normal_linear_speed_mps': .031,
        'arm_normal_angular_speed_rad_s': .21,
        'arm_precision_linear_speed_mps': .012,
        'arm_precision_angular_speed_rad_s': .09,
        'ready_linear_speed_mps': .04,
    })
    config = ArmMotionParameters(FakeNode({'arm.motion.linear_speed_mps': .9}))
    assert config.get('motion.linear_speed_mps') == .031
    assert config.get('motion.angular_speed_rad_s') == .21
    assert config.get('motion.precision_linear_speed_mps') == .012
    assert config.get('motion.precision_angular_speed_rad_s') == .09
    assert config.get('ready_linear_speed_mps') == .04
    assert config.get('motion.linear_speed_mps', .02) == .02


def test_base_profiles_apply_to_both_tag_and_relative_moves():
    BasicSettingsRepository().save({
        'base_mode': 'precision', 'base_normal_linear_speed_mps': .12,
        'base_precision_linear_speed_mps': .03, 'base_precision_angular_speed_rad_s': .07,
    })
    profiles = WalkingProfiles.from_node(FakeNode())
    assert profiles.for_move().relative_speed_mps == .03
    assert profiles.for_move(tag_relative=True).tag_speed_mps == .03
    assert profiles.precision.angular_speed_rad_s == .07
    assert profiles.normal.relative_speed_mps == profiles.normal.tag_speed_mps == .12


def test_force_multiplier_scales_policy_without_changing_hard_ceiling():
    default = SpeedAwareForceContactPolicy()
    expected = [default.threshold_for(speed) for speed in (0., .005, .08)]
    BasicSettingsRepository().save({'force_threshold_percent': 250.})
    _startup_settings.cache_clear()
    scaled = SpeedAwareForceContactPolicy()
    assert [scaled.threshold_for(speed) for speed in (0., .005, .08)] == pytest.approx([v * 2.5 for v in expected])
    assert ArmMotionParameters().get('contact.hard_force_delta_limit_n') == 20.


def test_collision_startup_waits_for_confirmation_and_applies_only_once():
    BasicSettingsRepository().save({'environment_collision_checking': True})
    control, _ = make_control(confirmed=False)
    assert not control.state().enabled
    active = SensorAttachmentState(status=SensorAttachmentState.STATUS_ACTIVE)
    control._receive_attachment(active)
    assert control.state().enabled
    control.set_enabled(False)
    control._receive_attachment(active)
    assert not control.state().enabled


def test_manual_off_cancels_pending_startup_collision_preference():
    BasicSettingsRepository().save({'environment_collision_checking': True})
    control, _ = make_control(confirmed=False)
    control.set_enabled(False)
    control._receive_attachment(SensorAttachmentState(status=SensorAttachmentState.STATUS_ACTIVE))
    assert not control.state().enabled


def test_mount_startup_approval_validates_remembered_sensor(tmp_path):
    _, repository, store, commands = build_controller(tmp_path)
    definition = sensor_definition()
    repository.create(definition)
    store.save(PersistedSensorAttachmentSelection(definition.sensor_id, 3))
    BasicSettingsRepository().save({'mount_approval_at_startup': True})
    _startup_settings.cache_clear()
    controller = SensorAttachmentController(repository, store, commands)
    assert controller.snapshot().status is SensorAttachmentStatus.ACTIVE
    assert controller.snapshot().active_sensor_id == definition.sensor_id


def test_missing_remembered_mount_is_not_auto_approved_as_bare_hand(tmp_path):
    _, repository, store, commands = build_controller(tmp_path)
    store.save(PersistedSensorAttachmentSelection('missing_sensor', 3))
    BasicSettingsRepository().save({'mount_approval_at_startup': True})
    _startup_settings.cache_clear()
    controller = SensorAttachmentController(repository, store, commands)
    assert controller.snapshot().status is SensorAttachmentStatus.NO_SENSOR
    restarted = SensorAttachmentController(repository, store, commands)
    assert restarted.snapshot().status is SensorAttachmentStatus.NO_SENSOR
    assert store.load().sensor_id == "missing_sensor"


def test_page_unit_conversion_save_reload_reset_and_restart_notice():
    app = QApplication.instance() or QApplication([])
    page = BasicSettingsPage()
    page.fields['arm_normal_linear_speed_mps'].setValue(35.)
    page.fields['arm_precision_angular_speed_rad_s'].setValue(45.)
    page.fields['arm_mode'].setCurrentIndex(1)
    page.fields['mount_approval_at_startup'].setChecked(True)
    page.save()
    saved = page.repository.load()
    assert saved['arm_normal_linear_speed_mps'] == .035
    assert saved['arm_precision_angular_speed_rad_s'] == pytest.approx(.7853981634)
    assert saved['arm_mode'] == 'precision'
    assert saved['mount_approval_at_startup'] is True
    assert 'Restart' in page.status.text()
    page._show_values(page.defaults)
    assert page.repository.load() == saved  # Reset is not saved until requested.
    page.reload()
    assert page.fields['arm_normal_linear_speed_mps'].value() == 35.
    page.close()


def test_mode_defaults_select_existing_command_controls():
    from fault_detector_spot.ui.manipulation.controls import ManipulationControls
    from fault_detector_spot.ui.navigation.base_movement_controls import BaseMovementControls
    app = QApplication.instance() or QApplication([])
    ui = SimpleNamespace(basic_settings={'arm_mode': 'precision', 'base_mode': 'precision'})
    arm = object.__new__(ManipulationControls)
    arm.ui = ui
    arm._make_precision_mode_row()
    base = object.__new__(BaseMovementControls)
    base.ui = ui
    base._make_walking_profile_row()
    assert arm.precision_mode_combo.currentData() is True
    assert base.walking_profile_dropdown.currentData() == 'precision'


@pytest.mark.parametrize('relative', [True, False])
def test_precision_speed_reaches_existing_execution_path(monkeypatch, relative):
    from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementExecutor
    from fault_detector_spot.manipulation.arm_motion_speed import ArmMotionSpeed, ArmMotionSpeedPolicy
    executor = object.__new__(ArmMovementExecutor)
    executor.speed_policy = ArmMotionSpeedPolicy(default_speed=ArmMotionSpeed(.08, .5))
    executor.precision_speed = ArmMotionSpeed(.02, .1)
    captured = []
    executor._start_precision_movement = lambda command, resolve, speed, threshold: captured.append(speed)
    command = SimpleNamespace(precision_mode=True, arm_speed_scale=.5)
    method = executor.relative if relative else executor.tag_probe
    method(command)
    assert captured[-1] == ArmMotionSpeed(.01, .05)
    method(command, speed=ArmMotionSpeed(.04, .2))
    assert captured[-1] == ArmMotionSpeed(.02, .1)


def test_page_preserves_unedited_numeric_precision():
    app = QApplication.instance() or QApplication([])
    repo = BasicSettingsRepository()
    repo.save({'arm_normal_angular_speed_rad_s': .5})
    page = BasicSettingsPage(repository=repo)
    page.save()
    assert repo.load()['arm_normal_angular_speed_rad_s'] == .5
    page.close()
