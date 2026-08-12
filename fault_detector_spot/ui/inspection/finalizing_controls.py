"""Add server-owned probe finalization to inspection controls."""

import math

from fault_detector_msgs.msg import ProbeSetupState, SensorDefinitionArray
from fault_detector_msgs.srv import AddSensor, RetireSensor

from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)

from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS

from .controls import InspectionControls


class FinalizingInspectionControls(InspectionControls):
    """Route physical setup workflows through server-owned APIs."""

    def init_ros_communication(self):
        """Keep only presentation-side sensor definition transport."""
        if self.node is None:
            return
        self.sensor_add_client = self.node.create_client(
            AddSensor,
            "fault_detector/add_sensor",
        )
        self.sensor_retire_client = self.node.create_client(
            RetireSensor,
            "fault_detector/retire_sensor",
        )
        self.sensor_list_subscription = self.node.create_subscription(
            SensorDefinitionArray,
            "fault_detector/sensors",
            self._process_sensor_definitions,
            LATCHED_QOS,
        )

    def handle_capture_reference_view(self):
        """Submit camera selections to the server-owned capture action."""
        state = self._probe_setup_state
        if state is None or not state.selected_routine_id:
            self.show_warning(
                "Capture Reference View",
                "Select a saved object and routine first.",
            )
            return False
        camera_ids = tuple(self._selected_reference_camera_ids())
        try:
            selected = tuple(value for value in camera_ids if value)
            if not selected:
                raise ValueError("Select at least one reference camera")
            if len(selected) != len(set(selected)):
                raise ValueError(
                    "Each reference camera can only be selected once"
                )
        except ValueError as exception:
            self.show_warning(
                "Capture Reference View",
                str(exception),
            )
            return False
        request_id = self.ui.execute_probe_reference_capture(
            camera_ids,
            replace_existing=(
                self.replace_reference_view_checkbox.isChecked()
            ),
        )
        if request_id is None:
            return False
        self.reference_view_status_label.setText(
            "Reference capture running"
        )
        return True

    def _update_save_probe_point_state(self, _value=None):
        state = self._probe_setup_state
        point_id = self.probe_point_id_field.text().strip()
        display_name = self.probe_point_display_name_field.text().strip()
        numeric_ready = all(
            self._is_positive_number(field.text())
            for field in (
                self.probe_position_tolerance_field,
                self.probe_orientation_tolerance_field,
                self.probe_measurement_duration_field,
            )
        )
        duplicate = (
            state is not None
            and point_id in state.probe_point_ids
            and point_id != self._editing_probe_point_id
        )
        ready = False
        if state is None or not state.selected_routine_id:
            status = "Select a saved object and routine."
        elif not state.has_reference_pixel:
            status = "Select a point in a captured reference view."
        elif not point_id or not display_name:
            status = "Enter a probe point ID and display name."
        elif duplicate:
            status = f"Probe point '{point_id}' already exists."
        elif not numeric_ready:
            status = "Enter positive tolerances and measurement duration."
        elif (
            state.surface_verification_state
            != ProbeSetupState.SURFACE_VERIFICATION_CONVERGED
        ):
            status = "Verify the final surface distance before saving."
        elif state.motion_pending:
            status = "Wait for the active probe movement to finish."
        else:
            ready = True
            status = "Ready to save and retract."
        self.approve_and_retract_button.setEnabled(ready)
        self.save_probe_point_status_label.setText(status)

    def _refresh_refinement_dialog(self):
        super()._refresh_refinement_dialog()
        presentation = self._refinement_presentation
        if presentation is None:
            return
        pending = presentation.pending_motion is not None
        probe_page = presentation.active_stage is RefinementStage.PROBE
        alignment_reached = (
            presentation.motion_states[RefinementStage.ALIGNMENT]
            is RefinementMotionState.REACHED
        )
        self.retract_without_saving_button.setEnabled(
            probe_page
            and alignment_reached
            and not pending
        )
        self._update_save_probe_point_state()

    def handle_approve_and_retract(self):
        """Request server-owned save followed by mandatory retraction."""
        point_id = self._required_text(
            self.probe_point_id_field,
            "a probe point ID",
        )
        display_name = self._required_text(
            self.probe_point_display_name_field,
            "a probe point display name",
        )
        if point_id is None or display_name is None:
            return False
        try:
            position_tolerance = self._positive_value(
                self.probe_position_tolerance_field,
                "Position tolerance",
            )
            orientation_tolerance = self._positive_value(
                self.probe_orientation_tolerance_field,
                "Orientation tolerance",
            )
            measurement_duration = self._positive_value(
                self.probe_measurement_duration_field,
                "Measurement duration",
            )
        except ValueError as exception:
            self.show_warning("Finalize Probe Point", str(exception))
            return False
        request_id = self.ui.execute_probe_refinement_finalization(
            save_requested=True,
            probe_point_id=point_id,
            probe_point_display_name=display_name,
            position_tolerance_m=position_tolerance,
            orientation_tolerance_rad=orientation_tolerance,
            measurement_duration_sec=measurement_duration,
        )
        if request_id is None:
            return False
        self.approve_and_retract_button.setEnabled(False)
        self.retract_without_saving_button.setEnabled(False)
        self.save_probe_point_status_label.setText(
            "Saving probe point and retracting"
        )
        return True

    def handle_retract_without_saving(self):
        """Request server-owned retraction without persistence."""
        request_id = self.ui.execute_probe_refinement_finalization(
            save_requested=False,
        )
        if request_id is None:
            return False
        self.approve_and_retract_button.setEnabled(False)
        self.retract_without_saving_button.setEnabled(False)
        self.refinement_recovery_status_label.setText(
            "Retracting without saving"
        )
        return True

    @staticmethod
    def _positive_value(field, label):
        text = field.text().strip()
        try:
            value = float(text)
        except ValueError as exception:
            raise ValueError(f"{label} must be a number") from exception
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{label} must be positive")
        return value


__all__ = ["FinalizingInspectionControls"]
