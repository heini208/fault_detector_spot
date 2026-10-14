"""Build immutable parameter files for processes with startup-only ROS settings."""

from hashlib import sha256
from pathlib import Path

import yaml

from .advanced_settings import MOVEIT_PATHS, NAVIGATION_PATHS
from .basic_settings import startup_settings
from .file_storage import atomic_write_text
from .runtime_paths import fault_detector_runtime_root


def settings_parameter_file(source, kind, settings=None):
    """Copy only the supported parameters, retaining the original file unchanged.

    A content-addressed path keeps already-running processes' configuration stable.
    Missing parameter paths fail clearly instead of pretending an override applied.
    """
    settings = startup_settings() if settings is None else settings
    paths, root = ((MOVEIT_PATHS, '/move_group') if kind == 'moveit' else
                   (NAVIGATION_PATHS, 'controller_server') if kind == 'navigation' else
                   (None, None))
    if paths is None:
        raise ValueError(f'Unknown settings parameter file kind: {kind}')
    selected = {key: settings[key] for key in paths if key in settings}
    if not selected:
        return str(source)
    document = yaml.safe_load(Path(source).read_text(encoding='utf-8'))
    try:
        parameters = document[root]['ros__parameters']
        for key, value in selected.items():
            target = parameters
            for part in paths[key][:-1]:
                target = target[part]
            leaf = paths[key][-1]
            if leaf not in target:
                raise KeyError(leaf)
            target[leaf] = -value if key == 'navigation.reverse_speed_mps' else value
        if kind == 'navigation':
            # Both the planner and its downstream smoother enforce speed limits.
            smoother_paths = {
                'navigation.forward_speed_mps': (('max_velocity', 0, 1.),),
                'navigation.reverse_speed_mps': (('min_velocity', 0, -1.),),
                'navigation.sideways_speed_mps': (('max_velocity', 1, 1.), ('min_velocity', 1, -1.)),
                'navigation.turning_speed_rad_s': (('max_velocity', 2, 1.), ('min_velocity', 2, -1.)),
            }
            for key in selected.keys() & smoother_paths.keys():
                smoother = document['velocity_smoother']['ros__parameters']
                for name, index, sign in smoother_paths[key]:
                    smoother[name][index] = sign * selected[key]
    except (KeyError, TypeError, IndexError) as error:
        raise ValueError(f'{source} does not support the saved {kind} settings: {error}') from error
    content = yaml.safe_dump(document, sort_keys=False)
    digest = sha256(content.encode('utf-8')).hexdigest()[:16]
    path = fault_detector_runtime_root() / 'generated_settings' / f'{kind}-{digest}.yaml'
    if not path.exists():
        atomic_write_text(path, content)
    return str(path)
