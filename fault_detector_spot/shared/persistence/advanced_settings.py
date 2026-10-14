"""Advanced setting schema and their factory YAML sources."""

import math

# Saved keys match ROS parameter names where there is a real parameter owner.
ARM_KEYS = (
    'arm.motion.position_tolerance_m', 'arm.motion.orientation_tolerance_rad',
    'arm.correction.maximum_attempts',
    'arm.contact.minimum_force_delta_threshold_n',
    'arm.contact.reference_force_delta_threshold_n',
    'arm.contact.maximum_force_delta_threshold_n',
    'arm.contact.retreat_distance_m', 'arm.contact.retreat_speed_mps',
    'arm.contact.telemetry.raw_logging_enabled',
    'arm.motion.moveit_velocity_scaling', 'arm.motion.moveit_acceleration_scaling',
)
BASE_KEYS = (
    'base.goal_verification.position_tolerance_m', 'base.goal_verification.yaw_tolerance_rad',
    'base.correction.maximum_attempts',
    'base.tag_correction.stability.required_samples',
    'base.tag_correction.stability.maximum_position_span_m',
    'base.tag_correction.stability.maximum_yaw_span_rad',
    'base.tag_correction.stability.maximum_sample_span_sec',
)
SURFACE_KEYS = (
    'close_surface.approach_far_speed_mps', 'close_surface.approach_near_speed_mps',
    'close_surface.approach_slowdown_distance_m', 'close_surface.contact_search_force_threshold_n',
)
NAVIGATION_PATHS = {
    'navigation.forward_speed_mps': ('FollowPath', 'vx_max'),
    'navigation.reverse_speed_mps': ('FollowPath', 'vx_min'),
    'navigation.sideways_speed_mps': ('FollowPath', 'vy_max'),
    'navigation.turning_speed_rad_s': ('FollowPath', 'wz_max'),
    'navigation.position_tolerance_m': ('general_goal_checker', 'xy_goal_tolerance'),
    'navigation.heading_tolerance_rad': ('general_goal_checker', 'yaw_goal_tolerance'),
}
MOVEIT_PATHS = {
    'moveit.octomap_resolution_m': ('octomap_resolution',),
    'moveit.lidar_max_range_m': ('lidar', 'max_range'),
}
KEYS = set(ARM_KEYS + BASE_KEYS + SURFACE_KEYS) | NAVIGATION_PATHS.keys() | MOVEIT_PATHS.keys()
BOOL_KEYS = {'arm.contact.telemetry.raw_logging_enabled'}
INTEGER_MINIMUMS = {
    'arm.correction.maximum_attempts': 0,
    'base.correction.maximum_attempts': 0,
    'base.tag_correction.stability.required_samples': 2,
}
SCALING_KEYS = {'arm.motion.moveit_velocity_scaling', 'arm.motion.moveit_acceleration_scaling'}


def validate(values):
    for key, value in values.items():
        if key in BOOL_KEYS:
            valid = type(value) is bool
        elif key in INTEGER_MINIMUMS:
            valid = type(value) is int and value >= INTEGER_MINIMUMS[key]
        else:
            valid = type(value) in (int, float) and math.isfinite(value) and value > 0
            if valid and key in SCALING_KEYS:
                valid = value <= 1
            if valid and key == 'arm.motion.position_tolerance_m':
                valid = .001 <= value <= .1  # Existing command editors' supported range.
        if not valid:
            raise ValueError(f'Invalid advanced setting {key}: {value!r}')


def validate_relationships(values):
    minimum, reference, maximum = (values[f'arm.contact.{name}_force_delta_threshold_n']
                                    for name in ('minimum', 'reference', 'maximum'))
    if not minimum <= reference <= maximum:
        raise ValueError('Force thresholds must satisfy minimum ≤ reference ≤ maximum.')
    if values['close_surface.approach_near_speed_mps'] > values['close_surface.approach_far_speed_mps']:
        raise ValueError('Near-surface speed must not exceed far-surface speed.')


def defaults(parameters, document):
    values = {}
    for filename, keys in [('arm_motion.yaml', ARM_KEYS), ('base_motion.yaml', BASE_KEYS),
                           ('close_surface.yaml', SURFACE_KEYS)]:
        source = parameters(filename)
        values.update({key: source[key] for key in keys})
    for filename, root, paths in [
        ('nav2_lidar_params.yaml', 'controller_server', NAVIGATION_PATHS),
        ('moveit_sensors.yaml', '/move_group', MOVEIT_PATHS),
    ]:
        source = document(filename)[root]['ros__parameters']
        for key, path in paths.items():
            value = source
            for part in path:
                value = value[part]
            values[key] = -value if key == 'navigation.reverse_speed_mps' else value
    validate(values)
    validate_relationships(values)
    return values
