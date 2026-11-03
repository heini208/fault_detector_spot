"""Tests for the compact inspection setup workspace."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState, TagElement

from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)
from fault_detector_spot.ui.navigation.base_movement_controls import (
    BaseMovementControls,
)


class FakePublisher:
    def publish(self, message):
        pass


class FakeUI:
    def __init__(self, object_root):
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = object_root
        self.visible_tags = {}
        self.available_frames = []
        self.posture_state_source = None

    def update_tags_dropdown(self, dropdown):
        previous = dropdown.currentText()
        tag_ids = sorted(self.visible_tags)
        dropdown.clear()
        if tag_ids:
            dropdown.addItems([str(tag_id) for tag_id in tag_ids])
            index = dropdown.findText(previous)
            dropdown.setCurrentIndex(max(0, index))
        else:
            dropdown.addItem("no tags available")
        dropdown.setEnabled(bool(tag_ids))

    def update_frames_dropdown(self, dropdown):
        previous = dropdown.currentText()
        frames = list(self.available_frames)
        dropdown.clear()
        if not frames:
            frames = ["no frames available"]
        dropdown.addItems(frames)
        index = dropdown.findText(previous)
        dropdown.setCurrentIndex(max(0, index))


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_base_position_dialog_uses_routine_setup_defaults(
    application,
    tmp_path,
):
    ui = FakeUI(tmp_path)
    ui.visible_tags = {7: object()}
    ui.available_frames = ["body", "Tag_7"]
    controls = FinalizingInspectionControls(ui)
    state = ProbeSetupState()
    state.selected_object_id = "motor"
    state.selected_routine_id = "scan"
    state.selected_reference_tag_id = 7
    state.base_body_height_m = -0.12
    controls._probe_setup_state = state

    assert controls.show_base_position_dialog()
    popup = controls.base_position_movement_controls

    assert popup.walking_profile_dropdown.currentData() == "precision"
    assert popup.frames_dropdown.currentText() == "Tag_7"
    assert popup.offset_fields["X"].text() == "-1.00"
    assert popup.body_height_slider.value() == 0
    assert popup.body_height_value_label.text() == "0.00 m"


@pytest.mark.parametrize("height_cm", [-20, 0, 15])
def test_base_position_save_includes_selected_height_without_moving(
    application, tmp_path, height_cm,
):
    ui = FakeUI(tmp_path)
    requests = []
    operations = []

    def submit(intent):
        requests.append(intent)
        return "save-position"

    ui.execute_probe_setup = submit
    ui.execute_operation = operations.append
    controls = FinalizingInspectionControls(ui)
    requests.clear()
    state = ProbeSetupState()
    state.selected_object_id = "motor"
    state.selected_routine_id = "scan"
    state.base_body_height_m = 0.10
    controls._probe_setup_state = state
    assert controls.show_base_position_dialog()
    controls.base_position_movement_controls.body_height_slider.setValue(height_cm)

    assert requests == []
    assert controls.handle_save_base_position()

    assert len(requests) == 1
    assert requests[0].operation == ProbeSetupIntent.OPERATION_SAVE_BASE_POSITION
    assert requests[0].body_height_m == pytest.approx(height_cm / 100.0)
    assert operations == []


@pytest.fixture
def base_position_session(tmp_path):
    ui = FakeUI(tmp_path)
    tag = TagElement()
    tag.id = 7
    tag.pose.pose.orientation.w = 1.0
    ui.visible_tags = {7: tag}
    ui.available_frames = ["body", "Tag_7"]
    requests = []
    operations = []

    def save(intent):
        requests.append(intent)
        return "save-position"

    def move(intent):
        operations.append(intent)
        return "move-base"

    ui.execute_probe_setup = save
    ui.execute_operation = move
    controls = FinalizingInspectionControls(ui)
    requests.clear()
    state = ProbeSetupState()
    state.selected_object_id = "motor"
    state.selected_routine_id = "scan"
    state.selected_reference_tag_id = 7
    state.has_base_position = True
    state.base_body_height_m = 0.12
    controls._probe_setup_state = state
    assert controls.show_base_position_dialog()
    popup = controls.base_position_movement_controls
    popup.ask_question = lambda *_args: True
    return SimpleNamespace(
        ui=ui, controls=controls, popup=popup, state=state,
        requests=requests, operations=operations,
    )


def test_base_position_reopen_does_not_reuse_saved_height(base_position_session):
    session = base_position_session
    assert session.controls.handle_save_base_position()
    assert session.requests[-1].body_height_m == 0.0

    session.popup.body_height_slider.setValue(12)
    assert session.controls.handle_save_base_position()
    assert session.requests[-1].body_height_m == pytest.approx(0.12)
    session.controls.base_position_dialog.hide()
    assert session.controls.show_base_position_dialog()
    assert session.controls.handle_save_base_position()

    assert session.popup.body_height_slider.value() == 0
    assert session.requests[-1].body_height_m == 0.0
    assert session.state.base_body_height_m == 0.12


@pytest.mark.parametrize("handler", ["handle_move_to_tag", "handle_move_base_relative"])
def test_setup_base_movement_clears_previously_selected_height(
    base_position_session, handler,
):
    session = base_position_session
    session.popup.body_height_slider.setValue(12)
    assert session.controls.handle_save_base_position()
    assert session.requests[-1].body_height_m == pytest.approx(0.12)

    getattr(session.popup, handler)()

    assert len(session.operations) == 1
    assert session.popup.body_height_slider.value() == 0
    assert session.popup.body_height_value_label.text() == "0.00 m"
    assert session.controls.handle_save_base_position()
    assert session.requests[-1].body_height_m == 0.0

    session.popup.body_height_slider.setValue(-8)
    assert session.controls.handle_save_base_position()
    assert session.requests[-1].body_height_m == pytest.approx(-0.08)


@pytest.mark.parametrize("handler", ["handle_move_to_tag", "handle_move_base_relative"])
@pytest.mark.parametrize("declined", [False, True])
def test_unsubmitted_setup_base_movement_preserves_selected_height(
    base_position_session, handler, declined,
):
    session = base_position_session
    session.popup.body_height_slider.setValue(12)
    if declined:
        session.popup.ask_question = lambda *_args: False
    else:
        session.ui.execute_operation = session.operations.append

    getattr(session.popup, handler)()

    assert len(session.operations) == (0 if declined else 1)
    assert session.popup.body_height_slider.value() == 12
    assert session.controls.handle_save_base_position()
    assert session.requests[-1].body_height_m == pytest.approx(0.12)


def test_setup_move_with_missing_tag_preserves_selected_height(base_position_session):
    session = base_position_session
    session.popup.body_height_slider.setValue(12)
    session.ui.visible_tags.clear()
    session.popup.show_warning = lambda *_args: None

    session.popup.handle_move_to_tag()

    assert session.operations == []
    assert session.popup.body_height_slider.value() == 12


@pytest.mark.parametrize("handler", ["handle_move_to_tag", "handle_move_base_relative"])
def test_regular_base_movement_preserves_height_slider(base_position_session, handler):
    session = base_position_session
    controls = BaseMovementControls(session.ui)
    controls.ask_question = lambda *_args: True
    controls.body_height_slider.setValue(12)

    getattr(controls, handler)()

    assert len(session.operations) == 1
    assert controls.body_height_slider.value() == 12


def test_base_position_height_draft_does_not_leak_between_routines(
    application, tmp_path,
):
    controls = FinalizingInspectionControls(FakeUI(tmp_path))
    state = ProbeSetupState()
    state.selected_object_id = "motor"
    state.selected_routine_id = "scan"
    state.has_base_position = True
    state.base_body_height_m = -0.12
    controls._probe_setup_state = state
    controls.show_base_position_dialog()
    controls.base_position_movement_controls.body_height_slider.setValue(18)
    controls._apply_base_position_state(state)
    assert controls.base_position_movement_controls.body_height_slider.value() == 18
    assert "body height -0.12 m" in controls.base_position_status_label.text()

    other = ProbeSetupState()
    other.selected_object_id = "motor"
    other.selected_routine_id = "other"
    controls._probe_setup_state = other
    controls._apply_base_position_state(other)

    assert not controls.save_base_position_button.isEnabled()
    assert controls.base_position_dialog.isHidden()
    controls.show_base_position_dialog()
    assert controls.base_position_movement_controls.body_height_slider.value() == 0

    controls.base_position_movement_controls.body_height_slider.setValue(10)
    controls.base_position_dialog.hide()
    controls.show_base_position_dialog()
    assert controls.base_position_movement_controls.body_height_slider.value() == 0


def test_base_position_dialog_follows_live_tag_and_frame_updates(
    application,
    tmp_path,
):
    ui = FakeUI(tmp_path)
    controls = FinalizingInspectionControls(ui)
    state = ProbeSetupState()
    state.selected_object_id = "motor"
    state.selected_routine_id = "scan"
    state.selected_reference_tag_id = 7
    controls._probe_setup_state = state

    assert controls.show_base_position_dialog()
    popup = controls.base_position_movement_controls
    assert popup.tag_dropdown.currentText() == "no tags available"

    ui.visible_tags = {5: object(), 7: object()}
    ui.available_frames = ["body", "odom"]
    controls.update_base_position_tags_dropdown()
    controls.update_base_position_frames_dropdown()

    assert popup.tag_dropdown.currentText() == "7"
    assert popup.tag_dropdown.isEnabled()
    assert {
        popup.frames_dropdown.itemText(index)
        for index in range(popup.frames_dropdown.count())
    } == {"body", "odom"}


def test_root_ui_forwards_live_lists_to_base_position_dialog(base_position_session):
    from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI

    session = base_position_session
    ui = session.ui
    ui.inspection_controls = session.controls
    ui.manipulation_controls = SimpleNamespace(
        update_tags_dropdown=lambda: None,
        update_frames_dropdown=lambda: None,
    )
    ui.base_movement_controls = ui.manipulation_controls
    ui.navigation_controls = SimpleNamespace(update_tags_dropdown=lambda: None)

    Fault_Detector_UI._process_visible_tags(
        ui, SimpleNamespace(elements=[SimpleNamespace(id=7), SimpleNamespace(id=9)])
    )
    Fault_Detector_UI._process_available_frames(
        ui, SimpleNamespace(names=["odom", "map"])
    )

    assert session.popup.tag_dropdown.currentText() == "7"
    assert [session.popup.tag_dropdown.itemText(index)
            for index in range(session.popup.tag_dropdown.count())] == ["7", "9"]
    assert [session.popup.frames_dropdown.itemText(index)
            for index in range(session.popup.frames_dropdown.count())] == ["odom", "map"]


def test_transient_approval_statuses_update_workflow_controls(
    application,
    tmp_path,
):
    controls = FinalizingInspectionControls(FakeUI(tmp_path))
    controls._probe_setup = SimpleNamespace(
        safe_approach_approved=True,
        surface_alignment_approved=True,
        probe_pose_approved=False,
    )

    controls._update_probe_setup_status_widgets()

    assert controls.approach_step_status_label.text() == "Approved"
    assert controls.alignment_step_status_label.text() == "Approved"
    assert controls.probe_step_status_label.text() == (
        "Ready for refinement"
    )
    assert controls.save_approach_status_label.text() == "Approved"
    assert controls.save_alignment_status_label.text() == "Approved"
    assert controls.save_probe_status_label.text() == "Not approved"


def test_refinement_dialog_uses_stage_safe_controls(application, tmp_path):
    controls = FinalizingInspectionControls(FakeUI(tmp_path))

    assert not controls.refinement_dialog.isModal()
    assert controls.refinement_dialog.stage_stack.count() == 6
    assert controls.move_aligned_pose_button.text() == "Move to Final Candidate"
    assert controls.use_current_alignment_button.text() == (
        "Save Final Aligned Pre-approach Pose"
    )
    assert controls.refinement_buttons["approach"] == {}
    assert "front" in controls.refinement_buttons["alignment"]
    assert "back" in controls.refinement_buttons["alignment"]
    assert controls.refinement_buttons["probe"] == {}
    assert controls.test_surface_distance_button.text() == (
        "Move Close to Surface"
    )
    assert not controls.test_surface_distance_button.isEnabled()
