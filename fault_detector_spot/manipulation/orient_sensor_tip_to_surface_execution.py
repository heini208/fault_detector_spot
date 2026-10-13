"""Measure the intended tip target, return, then align to its frozen normal."""

from copy import deepcopy
import math
import time

from geometry_msgs.msg import PoseStamped

from fault_detector_spot.inspection.geometry.alignment_orientation import (
    surface_aligned_probe_orientation,
)
from fault_detector_spot.inspection.model.sensor_models import sensor_probe_frame
from fault_detector_spot.manipulation.arm_motion_speed import ArmMotionSpeed
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome, ArmMovementUpdate,
)
from fault_detector_spot.manipulation.measure_sensor_tip_distance_execution import (
    MeasureSensorTipDistanceExecution, MeasureSensorTipDistanceOutcome,
)
from fault_detector_spot.shared.geometry.models import Vector3Data
from fault_detector_spot.shared.geometry.rotation import (
    rotate_vector, rotation_distance_rad,
)
from fault_detector_spot.shared.geometry.transforms import (
    pose_data_to_pose, pose_to_pose_data,
)


class OrientSensorTipToSurfaceExecution:
    """Compose observation/return with one guarded orientation and verification.

    The final probe position is preserved in odom. MoveIt interpolates the hand
    path, so this is not a promise of an exact stationary-tip pivot throughout
    the movement. No alignment retry or recovery movement is initiated here.
    """

    def __init__(self, executor, surface_source, config, monotonic_clock=time.monotonic, logger=None):
        config.validate()
        self.executor = executor
        self.surface_source = surface_source
        self.config = config
        self._clock = monotonic_clock
        self._measurement = MeasureSensorTipDistanceExecution(
            executor, surface_source, config, monotonic_clock=monotonic_clock,
            logger=logger,
        )
        self._active = False
        self._owns_motion = False
        self.failure_outcome = None
        self.feedback_message = ""

    @property
    def active(self):
        return self._active

    def start(self, command):
        if self.active:
            raise RuntimeError("Sensor tip orientation is already active")
        self.failure_outcome = None
        self._owns_motion = False
        if self.executor.active:
            return self._fail("Arm executor is busy; sensor tip orientation was not started", ArmMovementOutcome.BUSY)
        self._active = True
        self._phase = "measuring"
        try:
            self._attachment = self.surface_source.active_attachment()
            if self._attachment[0] != command.motion_sensor_id:
                raise ValueError("Sensor attachment does not match the requested tip")
            if self.executor.moveit_arm_planner is None:
                raise ValueError("Sensor tip orientation requires collision-checked MoveIt planning")
            return self._measurement_update(self._measurement.start(command))
        except Exception as error:
            return self._fail(str(error))

    def cancel(self):
        if self._measurement.active:
            self._measurement.cancel()
        elif self._owns_motion and self.executor.active:
            self.executor.cancel()
        self._owns_motion = False
        self._active = False

    def poll(self):
        if not self.active:
            raise RuntimeError("No sensor tip orientation is active")
        try:
            self._check_attachment()
            if self._phase == "measuring":
                return self._measurement_update(self._measurement.poll())
            self._check_reference(pose_to_pose_data(
                self.executor.probe_motion_planner.current_pose("odom", "body")))
            if self._phase == "aligning":
                return self._motion_update(self.executor.poll())
            if self._phase == "verifying":
                return self._verify_alignment()
            raise RuntimeError(f"Unknown sensor tip orientation phase: {self._phase}")
        except Exception as error:
            return self._fail(str(error))

    def _measurement_update(self, outcome):
        if outcome is MeasureSensorTipDistanceOutcome.RUNNING:
            return self._running(self._measurement.feedback_message)
        if outcome is not MeasureSensorTipDistanceOutcome.SUCCESS:
            return self._fail(
                self._measurement.feedback_message,
                self._measurement.failure_outcome or ArmMovementOutcome.EXECUTION_ERROR,
            )
        self._measured = self._measurement.result
        if self._measured is None:
            raise RuntimeError("Sensor tip measurement returned no surface plane")
        self._check_attachment()
        self._measured.surface_plane_odom.validate()
        if self._measured.surface_plane_odom.frame_id != "odom":
            raise ValueError("Sensor tip surface plane must be expressed in odom")
        # Reuse the verified actual returned tip, never the nominal setup pose.
        self._origin = deepcopy(self._measured.returned_probe_pose)
        self._check_reference(pose_to_pose_data(
            self.executor.probe_motion_planner.current_pose("odom", "body")))
        current = pose_to_pose_data(self.executor.probe_motion_planner.current_pose(
            "odom", sensor_probe_frame(self._attachment[0])))
        if (self._position_error(current, self._origin) > self.config.tip_alignment_start_position_tolerance_m
                or rotation_distance_rad(current.orientation, self._origin.orientation)
                > self.config.return_orientation_tolerance_rad):
            raise ValueError("Sensor tip moved after distance measurement; measure again before aligning")
        if self._axis_error(self._origin) <= 1e-6:
            return self._success(self._origin)
        planner = self.executor.probe_motion_planner
        hand_to_probe = pose_to_pose_data(planner.hand_to_probe_pose(self._attachment[0]))
        target = deepcopy(self._origin)
        target.orientation = surface_aligned_probe_orientation(
            self._measured.surface_plane_odom.normal, hand_to_probe.orientation,
            Vector3Data(0.0, 0.0, 1.0),
        )
        stamped = PoseStamped()
        stamped.header.frame_id = "odom"
        stamped.pose = pose_data_to_pose(target)
        if self.executor.active:
            return self._fail("Arm executor is busy before sensor tip alignment", ArmMovementOutcome.BUSY)
        if self.executor.moveit_arm_planner is None:
            raise RuntimeError("Sensor tip orientation requires collision-checked MoveIt planning")
        self._phase = "aligning"
        self._owns_motion = True
        self.feedback_message = "Aligning sensor tip to the measured target surface"
        update = self.executor.guarded_probe(
            stamped, self._attachment[0],
            speed=ArmMotionSpeed(self.config.observation_speed_mps,
                                self.executor.speed_policy.default_speed.angular_speed_rad_s),
            force_threshold_n=self.config.force_contact_threshold_n,
            cartesian_path=True, ignore_environment_collisions=False,
        )
        if update.outcome is ArmMovementOutcome.BUSY:
            self._owns_motion = False
        return self._motion_update(update)

    def _motion_update(self, update):
        if update.outcome is ArmMovementOutcome.RUNNING:
            return self._running(self.feedback_message)
        if update.outcome is not ArmMovementOutcome.SUCCESS:
            return self._fail(f"Sensor tip alignment failed: {update.detail}", update.outcome)
        self._owns_motion = False
        self._phase = "verifying"
        self._receipt_after = self._clock() + self.config.settle_sec
        return self._running("Verifying sensor tip position and surface alignment")

    def _verify_alignment(self):
        if self._clock() < self._receipt_after:
            return self._running(self.feedback_message)
        try:
            _, probe, body = self.surface_source.sensor_tip_pose_snapshot(
                self._attachment[0], receipt_not_before=self._receipt_after)
            self._check_attachment()
            self._check_reference(body)
            position_error = self._position_error(probe, self._origin)
            axis_error = self._axis_error(probe)
            if (position_error > self.config.tip_alignment_position_tolerance_m
                    or axis_error > self.config.maximum_axis_error_rad):
                raise ValueError(
                    f"Sensor tip alignment not verified: position error {position_error*1000:.2f} mm, "
                    f"limit {self.config.tip_alignment_position_tolerance_m*1000:.2f} mm; "
                    f"axis error {math.degrees(axis_error):.2f} deg, "
                    f"limit {math.degrees(self.config.maximum_axis_error_rad):.2f} deg")
            return self._success(probe)
        except Exception as error:
            if self._clock() - self._receipt_after < self.config.sample_timeout_sec:
                return self._running(f"Verifying sensor tip alignment: {error}")
            return self._fail(str(error), ArmMovementOutcome.CHECKPOINT_TOLERANCE_FAILED)

    def _check_attachment(self):
        if self.surface_source.active_attachment() != self._attachment:
            raise RuntimeError("Sensor attachment changed; sensor tip orientation invalidated")

    def _check_reference(self, body):
        reference = self._measured.reference_body_pose
        if (self._position_error(body, reference) > self.config.reference_position_tolerance_m
                or rotation_distance_rad(body.orientation, reference.orientation)
                > self.config.reference_orientation_tolerance_rad):
            raise RuntimeError("Body/odom reference changed; sensor tip orientation invalidated")

    def _axis_error(self, probe):
        axis = rotate_vector(probe.orientation, Vector3Data(1.0, 0.0, 0.0))
        normal = self._measured.surface_plane_odom.normal
        cosine = -(axis.x*normal.x + axis.y*normal.y + axis.z*normal.z)
        return math.acos(max(-1.0, min(1.0, cosine)))

    @staticmethod
    def _position_error(first, second):
        first.validate()
        second.validate()
        return math.sqrt(sum((getattr(first.position, axis) - getattr(second.position, axis))**2
                             for axis in ("x", "y", "z")))

    def _running(self, detail):
        self.feedback_message = detail
        return ArmMovementUpdate(ArmMovementOutcome.RUNNING, detail)

    def _success(self, probe):
        self._active = False
        distance = self._measured.surface_plane_odom.signed_distance(probe.position)
        self.feedback_message = (
            f"Sensor tip aligned to surface: axis error {math.degrees(self._axis_error(probe)):.2f} deg; "
            f"tip position error {self._position_error(probe, self._origin)*1000:.2f} mm; "
            f"distance {distance*1000:.2f} mm perpendicular")
        return ArmMovementUpdate(ArmMovementOutcome.SUCCESS, self.feedback_message)

    def _fail(self, detail, outcome=ArmMovementOutcome.EXECUTION_ERROR):
        self.cancel()
        self.failure_outcome = outcome
        self.feedback_message = str(detail)
        return ArmMovementUpdate(outcome, self.feedback_message)
