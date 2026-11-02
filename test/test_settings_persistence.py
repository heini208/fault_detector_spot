"""Settings persistence and startup snapshots run without ROS or Qt."""

from pathlib import Path

import pytest
import yaml

from fault_detector_spot.shared.persistence.basic_settings import (
    BasicSettingsRepository, factory_settings, startup_settings, _startup_settings,
)
from fault_detector_spot.shared.persistence.settings_parameter_files import settings_parameter_file

ROOT = Path(__file__).resolve().parents[1]


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
    repo.path.write_text('version: 99\nsettings: {}\n')
    with pytest.raises(ValueError, match='Unsupported'):
        repo.load()


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
