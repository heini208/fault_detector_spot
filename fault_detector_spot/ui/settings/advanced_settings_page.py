"""Advanced tuning controls with user-facing units and scoped explanations."""

import math

from PyQt5.QtWidgets import QLabel

from .basic_settings_page import BasicSettingsPage
from fault_detector_spot.shared.persistence.basic_settings import factory_advanced_settings
from fault_detector_spot.shared.persistence.advanced_settings import INTEGER_MINIMUMS, SCALING_KEYS


DEGREES = 180. / math.pi
# group, explanatory text, (key, label, displayed units per stored unit, suffix)
SECTIONS = (
    ('Arm accuracy', 'Defaults for new commands and points. Saved point tolerances stay unchanged. '
     'Turning accuracy is checked in Precision mode. Zero correction attempts disables tag/Precision retries.', (
        ('arm.motion.position_tolerance_m', 'Allowed position error', 1000., ' mm'),
        ('arm.motion.orientation_tolerance_rad', 'Allowed turning error (Precision moves)', DEGREES, ' °'),
        ('arm.motion.probe_setup_orientation_tolerance_rad', 'Allowed turning error (probe setup)', DEGREES, ' °'),
        ('arm.correction.maximum_attempts', 'Maximum correction attempts', 1., ''),
    )),
    ('Walking accuracy', 'Allowed error after an offset or tag move. Zero correction attempts disables retries.', (
        ('base.goal_verification.position_tolerance_m', 'Allowed position error', 1000., ' mm'),
        ('base.goal_verification.yaw_tolerance_rad', 'Allowed heading error', DEGREES, ' °'),
        ('base.correction.maximum_attempts', 'Maximum correction attempts', 1., ''),
    )),
    ('Force and contact', 'Force is measured as a change from the stationary baseline. '
     'The Basic force percentage scales these three thresholds. The separate hard safety limit stays unchanged.', (
        ('arm.contact.minimum_force_delta_threshold_n', 'Minimum force threshold', 1., ' N'),
        ('arm.contact.reference_force_delta_threshold_n', 'Reference force threshold', 1., ' N'),
        ('arm.contact.maximum_force_delta_threshold_n', 'Maximum force threshold', 1., ' N'),
        ('arm.contact.retreat_distance_m', 'Distance to back away after contact', 1000., ' mm'),
        ('arm.contact.retreat_speed_mps', 'Speed when backing away', 1000., ' mm/s'),
    )),
    ('Approaching a surface', 'Far and near speeds are blended using the distance to the surface at the start '
     'of each movement. Speed is not continuously reduced during a move. Contact search uses its own force threshold.', (
        ('close_surface.approach_far_speed_mps', 'Speed farther from the surface', 1000., ' mm/s'),
        ('close_surface.approach_near_speed_mps', 'Speed near the surface', 1000., ' mm/s'),
        ('close_surface.approach_slowdown_distance_m', 'Distance used to select a slower speed', 1000., ' mm'),
        ('close_surface.contact_search_force_threshold_n', 'Force threshold for zero-distance contact', 1., ' N'),
    )),
    ('Navigation', 'Speed limits and arrival accuracy for autonomous navigation. '
     'Restart the robot application and navigation to apply.', (
        ('navigation.forward_speed_mps', 'Maximum forward speed', 1., ' m/s'),
        ('navigation.reverse_speed_mps', 'Maximum backward speed', 1., ' m/s'),
        ('navigation.sideways_speed_mps', 'Maximum sideways speed', 1., ' m/s'),
        ('navigation.turning_speed_rad_s', 'Maximum turning speed', DEGREES, ' °/s'),
        ('navigation.position_tolerance_m', 'Allowed position error on arrival', 1000., ' mm'),
        ('navigation.heading_tolerance_rad', 'Allowed heading error on arrival', DEGREES, ' °'),
    )),
    ('MoveIt arm planning', 'Percentages of the configured joint speed and acceleration limits. '
     'Applies to normal planning, not the separate straight-line surface approach.', (
        ('arm.motion.moveit_velocity_scaling', 'Joint speed limit', 100., ' %'),
        ('arm.motion.moveit_acceleration_scaling', 'Joint acceleration limit', 100., ' %'),
    )),
    ('MoveIt obstacle map', 'Smaller map cells capture more detail but use more resources. '
     'Restart MoveIt with the robot application to apply.', (
        ('moveit.octomap_resolution_m', 'Obstacle map cell size', 1000., ' mm'),
        ('moveit.lidar_max_range_m', 'Maximum lidar obstacle distance', 1., ' m'),
    )),
    ('Tag observation stability', 'Before a walking correction, tag observations must agree within these limits. '
     'Tighter limits can increase waiting or cause a timeout.', (
        ('base.tag_correction.stability.required_samples', 'Required observations', 1., ''),
        ('base.tag_correction.stability.maximum_position_span_m', 'Allowed position variation', 1000., ' mm'),
        ('base.tag_correction.stability.maximum_yaw_span_rad', 'Allowed heading variation', DEGREES, ' °'),
        ('base.tag_correction.stability.maximum_sample_span_sec', 'Time window for observations', 1., ' s'),
    )),
)


class AdvancedSettingsPage(BasicSettingsPage):
    title = 'Advanced settings'
    factory_defaults = staticmethod(factory_advanced_settings)

    def _build_fields(self):
        for title, explanation, entries in SECTIONS:
            form = self._group(title)
            hint = QLabel(explanation)
            hint.setWordWrap(True)
            form.addRow(hint)
            for key, label, factor, suffix in entries:
                self._number(form, key, label, factor, suffix)
                field = self.fields[key]
                field.setToolTip(explanation)
                if key in INTEGER_MINIMUMS:
                    field.setDecimals(0)
                    field.setMinimum(INTEGER_MINIMUMS[key])
                elif key in SCALING_KEYS:
                    field.setMaximum(100.)
                elif key == 'arm.motion.position_tolerance_m':
                    field.setRange(1., 100.)
            if title == 'Force and contact':
                self.fields['arm.contact.minimum_force_delta_threshold_n'].setToolTip(
                    'The lowest force threshold, used when planned movement speed approaches zero.'
                )
                self.fields['arm.contact.reference_force_delta_threshold_n'].setToolTip(
                    'The threshold at the configured reference movement speed. '
                    'The threshold increases with planned speed between the minimum and maximum.'
                )
                self.fields['arm.contact.maximum_force_delta_threshold_n'].setToolTip(
                    'The upper end of the speed-dependent threshold, before the Basic percentage is applied. '
                    'This is separate from the fixed hard safety limit.'
                )
                self._boolean(form, 'arm.contact.telemetry.raw_logging_enabled',
                              'Save detailed force readings',
                              'Write raw contact telemetry for diagnosing and tuning force detection.')
