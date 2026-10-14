"""Default settings preserve the pre-settings command behavior and overrides."""

import math

import pytest
import yaml
from PyQt5.QtWidgets import QApplication

from fault_detector_spot.shared.persistence.basic_settings import (
    BasicSettingsRepository, factory_settings, factory_advanced_settings,
    startup_settings, _startup_settings,
)
from fault_detector_spot.ui.settings.basic_settings_page import BasicSettingsPage
from fault_detector_spot.ui.settings.advanced_settings_page import AdvancedSettingsPage
from fault_detector_spot.manipulation.arm_motion_parameters import ArmMotionParameters
from fault_detector_spot.manipulation.arm_motion_speed import ArmMotionSpeed, ArmMotionSpeedPolicy
from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementExecutor
from fault_detector_spot.navigation.walking_profile import WalkingProfiles
from test_arm_motion_parameters import FakeNode


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros'))
    _startup_settings.cache_clear()
    yield
    _startup_settings.cache_clear()


@pytest.fixture
def app():
    return QApplication.instance() or QApplication([])


@pytest.mark.parametrize('page_type', [BasicSettingsPage, AdvancedSettingsPage])
def test_saving_untouched_page_keeps_existing_speed_overrides(app, page_type):
    page = page_type()
    page.save()
    assert page.repository.load() == {}
    _startup_settings.cache_clear()
    node = FakeNode({'arm.motion.linear_speed_mps': .03, 'arm.motion.angular_speed_rad_s': .12})
    executor = ArmMovementExecutor(object(), action_client=object(), config=ArmMotionParameters(node))
    assert executor.speed_policy.default_speed == executor.precision_speed == ArmMotionSpeed(.03, .12)
    profiles = WalkingProfiles.from_node(FakeNode())
    assert profiles.normal.relative_speed_mps == .1
    assert profiles.normal.tag_speed_mps == .15
    page.close()


def test_precision_inherits_injected_speed_policy():
    speed = ArmMotionSpeed(.027, .13)
    executor = ArmMovementExecutor(object(), action_client=object(),
                                   speed_policy=ArmMotionSpeedPolicy(default_speed=speed))
    assert executor.precision_speed == speed


def test_precision_components_can_be_overridden_independently():
    BasicSettingsRepository().save({'arm_precision_angular_speed_rad_s': .4})
    config = ArmMotionParameters(FakeNode({'arm.motion.linear_speed_mps': .03}))
    executor = ArmMovementExecutor(object(), action_client=object(), config=config)
    assert executor.precision_speed == ArmMotionSpeed(.03, .4)


def test_editing_one_field_does_not_freeze_other_defaults(app):
    page = BasicSettingsPage()
    page.fields['force_threshold_percent'].setValue(125.)
    page.save()
    assert page.repository.load() == {'force_threshold_percent': 125.}
    page.close()


def test_precision_preview_follows_normal_until_explicitly_changed(app):
    page = BasicSettingsPage()
    page.fields['arm_normal_linear_speed_mps'].setValue(30.)
    page.save()
    assert page.repository.load() == {'arm_normal_linear_speed_mps': .03}
    assert page.fields['arm_precision_linear_speed_mps'].value() == 30.
    page.fields['arm_precision_linear_speed_mps'].setValue(20.)
    page.save()
    assert page.repository.load()['arm_precision_linear_speed_mps'] == .02
    page.close()


@pytest.mark.parametrize('page_type', [BasicSettingsPage, AdvancedSettingsPage])
def test_reset_removes_only_this_tabs_overrides_after_save(app, page_type):
    repo = BasicSettingsRepository()
    repo.save({'arm_mode': 'precision', 'arm_normal_linear_speed_mps': .03,
               'arm.contact.minimum_force_delta_threshold_n': 2.})
    before = repo.load()
    page = page_type(repository=repo)
    page.reset_defaults()
    assert repo.load() == before
    page.save()
    assert repo.load() == {key: value for key, value in before.items() if key not in page.fields}
    page.close()


def test_reset_all_settings_restores_inheritance_and_original_modes(app):
    repo = BasicSettingsRepository()
    values = {**factory_settings(), **factory_advanced_settings()}
    values.update(mount_approval_at_startup=True, environment_collision_checking=True,
                  arm_mode='precision', base_mode='precision')
    repo.save(values)
    for page_type in (BasicSettingsPage, AdvancedSettingsPage):
        page = page_type(repository=repo)
        page.reset_defaults()
        page.save()
        page.close()
    assert repo.load() == {}
    _startup_settings.cache_clear()
    assert startup_settings() == {}
    profiles = WalkingProfiles.from_node(FakeNode())
    assert profiles.normal.tag_speed_mps == .15
    assert profiles.relative_profile == profiles.tag_profile == 'normal'


def test_old_full_form_defaults_are_inherited_but_user_choices_survive():
    repo = BasicSettingsRepository()
    old_values = {**factory_settings(), **factory_advanced_settings()}
    old_values.pop('arm.motion.probe_setup_orientation_tolerance_rad')
    old_values.update(arm_mode='precision', mount_approval_at_startup=True,
                      base_precision_linear_speed_mps=.025)
    repo.path.parent.mkdir(parents=True)
    repo.path.write_text(yaml.safe_dump({'version': 1, 'settings': old_values}))
    before = repo.path.read_bytes()
    assert repo.load() == {'arm_mode': 'precision', 'mount_approval_at_startup': True,
                           'base_precision_linear_speed_mps': .025}
    assert repo.path.read_bytes() == before
    profiles = WalkingProfiles.from_node(FakeNode())
    assert profiles.normal.tag_speed_mps == .15
    assert profiles.precision.tag_speed_mps == .025


def test_new_explicit_override_equal_to_factory_value_stays_explicit(app):
    repo = BasicSettingsRepository()
    repo.save({'arm_normal_linear_speed_mps': .04})
    page = BasicSettingsPage(repository=repo)
    page.fields['arm_normal_linear_speed_mps'].setValue(80.)
    page.save()
    assert repo.load() == {'arm_normal_linear_speed_mps': .08}
    assert yaml.safe_load(repo.path.read_text())['version'] == 2
    assert ArmMotionParameters(FakeNode({'arm.motion.linear_speed_mps': .03})).get('motion.linear_speed_mps') == .08
    page.close()


def test_unchanged_save_preserves_other_writers_changes(app):
    repo = BasicSettingsRepository()
    page = BasicSettingsPage(repository=repo)
    repo.save({'arm_mode': 'precision'})
    page.save()
    assert repo.load() == {'arm_mode': 'precision'}
    page.close()


def test_original_manual_and_probe_setup_orientation_defaults_are_distinct(app):
    from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
    from test_probe_point_guided_workflow import FakeUI
    config = ArmMotionParameters()
    assert config.get('motion.orientation_tolerance_rad') == math.radians(2.)
    controls = FinalizingInspectionControls(FakeUI())
    assert float(controls.probe_orientation_tolerance_field.text()) == .087
    assert float(controls.probe_position_tolerance_field.text()) == .01
    controls.refinement_dialog.close()
