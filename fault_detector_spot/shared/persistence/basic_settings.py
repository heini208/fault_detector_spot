"""Validated, atomic user overrides shared by local UI and application processes.

Startup snapshots never change when the settings page saves. Restart all application
processes to apply edits. UI and backend must share ROS_HOME and the same filesystem.
"""

from functools import lru_cache
import math
from pathlib import Path
from types import MappingProxyType

import yaml

from . import advanced_settings
from .file_storage import atomic_write_text
from .runtime_paths import fault_detector_runtime_root


ARM_KEYS = {
    'motion.linear_speed_mps': 'arm_normal_linear_speed_mps',
    'motion.angular_speed_rad_s': 'arm_normal_angular_speed_rad_s',
    'motion.precision_linear_speed_mps': 'arm_precision_linear_speed_mps',
    'motion.precision_angular_speed_rad_s': 'arm_precision_angular_speed_rad_s',
    'ready_linear_speed_mps': 'ready_linear_speed_mps',
}
BASE_KEYS = {
    f'{mode}.{kind}_speed_mps': f'base_{mode}_linear_speed_mps'
    for mode in ('normal', 'precision') for kind in ('relative', 'tag')
}
BASE_KEYS.update({
    f'{mode}.angular_speed_rad_s': f'base_{mode}_angular_speed_rad_s'
    for mode in ('normal', 'precision')
})
BASE_KEYS.update(relative_profile='base_mode', tag_profile='base_mode')
BOOL_KEYS = {'mount_approval_at_startup', 'environment_collision_checking'}
MODE_KEYS = {'arm_mode', 'base_mode'}
NUMBER_KEYS = set(ARM_KEYS.values()) | (set(BASE_KEYS.values()) - MODE_KEYS) | {'force_threshold_percent'}


def validate_settings(values):
    if not isinstance(values, dict):
        raise ValueError('Basic settings must be a mapping')
    unknown = values.keys() - BOOL_KEYS - MODE_KEYS - NUMBER_KEYS - advanced_settings.KEYS
    if unknown:
        raise ValueError(f'Unknown basic settings: {sorted(unknown)}')
    advanced = {key: value for key, value in values.items() if key in advanced_settings.KEYS}
    advanced_settings.validate(advanced)
    if advanced:
        advanced_settings.validate_relationships({**factory_advanced_settings(), **advanced})
    for key, value in values.items():
        if key in advanced_settings.KEYS:
            continue
        if key in BOOL_KEYS:
            valid = type(value) is bool
        elif key in MODE_KEYS:
            valid = value in ('normal', 'precision')
        else:
            valid = type(value) in (int, float) and math.isfinite(value) and value > 0
        if not valid:
            raise ValueError(f'Invalid basic setting {key}: {value!r}')
    return dict(values)


def config_document(filename):
    source = Path(__file__).resolve().parents[3] / 'config'
    if not source.is_dir():
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory('fault_detector_spot')) / 'config'
    return yaml.safe_load((source / filename).read_text())


def config_parameters(filename):
    return config_document(filename)['/**']['ros__parameters']


def factory_advanced_settings():
    return advanced_settings.defaults(config_parameters, config_document)


def factory_settings():
    arm = config_parameters('arm_motion.yaml')
    base = config_parameters('base_motion.yaml')
    values = {
        'mount_approval_at_startup': False,
        'environment_collision_checking': False,
        'arm_mode': 'normal',
        'base_mode': base['base.walking.relative_profile'],
        'force_threshold_percent': 100.0,
    }
    values.update({setting: arm[f'arm.{key}'] for key, setting in ARM_KEYS.items()})
    for mode in ('normal', 'precision'):
        values[f'base_{mode}_linear_speed_mps'] = base[f'base.walking.{mode}.relative_speed_mps']
        values[f'base_{mode}_angular_speed_rad_s'] = base[f'base.walking.{mode}.angular_speed_rad_s']
    return validate_settings(values)


class BasicSettingsRepository:
    def __init__(self, path=None):
        self.path = Path(path) if path is not None else fault_detector_runtime_root() / 'basic_settings.yaml'

    def load(self):
        try:
            document = yaml.safe_load(self.path.read_text(encoding='utf-8'))
        except FileNotFoundError:
            return {}
        except yaml.YAMLError as error:
            raise ValueError(f'Invalid settings YAML: {self.path}') from error
        if not isinstance(document, dict) or type(document.get('version')) is not int or document['version'] != 1:
            raise ValueError(f'Unsupported basic settings document: {self.path}')
        return validate_settings(document.get('settings'))

    def save(self, values):
        values = validate_settings(values)
        atomic_write_text(self.path, yaml.safe_dump({'version': 1, 'settings': values}, sort_keys=True))


@lru_cache(maxsize=None)
def _startup_settings(path):
    return MappingProxyType(BasicSettingsRepository(path).load())


def startup_settings():
    """Return immutable overrides, captured once per process and runtime path."""
    return _startup_settings(BasicSettingsRepository().path)


def parameter_override(name, default):
    """Read a saved override without changing the current process snapshot."""
    return startup_settings().get(name, default)
