"""Tests for visible-tag selection in movement and landmark controls."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication, QComboBox  # noqa: E402, I100
import pytest  # noqa: E402, I100

from fault_detector_spot.ui.fault_detector_ui import (  # noqa: E402, I100
    Fault_Detector_UI,
)
from fault_detector_spot.ui.manipulation.controls import (  # noqa: E402, I100
    ManipulationControls,
)
from fault_detector_spot.ui.navigation.base_movement_controls import (  # noqa: E402, I100
    BaseMovementControls,
)
from fault_detector_spot.ui.navigation.controls import NavigationControls  # noqa: E402, I100
from fault_detector_msgs.msg import NavigationSetupIntent  # noqa: E402, I100


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


class _Parent:

    def __init__(self, tag_ids):
        self.node = None
        self.visible_tags = {tag_id: object() for tag_id in tag_ids}
        self.submitted = []

    def update_tags_dropdown(self, dropdown):
        Fault_Detector_UI.update_tags_dropdown(self, dropdown)

    def execute_navigation_setup(self, intent):
        self.submitted.append(intent)
        return "request-id"


def test_movement_tabs_use_dropdowns_with_current_visible_tags():
    parent = _Parent([9, 2])

    manipulation = object.__new__(ManipulationControls)
    manipulation.ui = parent
    manipulation._make_tag_input_row()

    base_movement = object.__new__(BaseMovementControls)
    base_movement.ui = parent
    base_movement._make_tag_input_row()

    for controls in (manipulation, base_movement):
        assert isinstance(controls.tag_dropdown, QComboBox)
        assert [
            controls.tag_dropdown.itemText(index)
            for index in range(controls.tag_dropdown.count())
        ] == ["2", "9"]


def test_tag_dropdown_preserves_a_still_visible_selection():
    parent = _Parent([2, 9])
    dropdown = QComboBox()
    parent.update_tags_dropdown(dropdown)
    dropdown.setCurrentText("9")

    parent.visible_tags = {9: object(), 12: object()}
    parent.update_tags_dropdown(dropdown)

    assert dropdown.currentText() == "9"
    assert [dropdown.itemText(index) for index in range(dropdown.count())] == [
        "9",
        "12",
    ]


def test_tag_dropdown_disables_itself_when_no_tags_are_visible():
    parent = _Parent([])
    dropdown = QComboBox()

    parent.update_tags_dropdown(dropdown)

    assert dropdown.currentText() == "no tags available"
    assert not dropdown.isEnabled()


def test_visible_tag_message_refreshes_movement_and_landmark_dropdowns():
    calls = []
    ui = Fault_Detector_UI.__new__(Fault_Detector_UI)
    ui.manipulation_controls = SimpleNamespace(
        update_tags_dropdown=lambda: calls.append("manipulation")
    )
    ui.base_movement_controls = SimpleNamespace(
        update_tags_dropdown=lambda: calls.append("base")
    )
    ui.navigation_controls = SimpleNamespace(
        update_tags_dropdown=lambda: calls.append("navigation")
    )
    message = SimpleNamespace(
        elements=[SimpleNamespace(id=9), SimpleNamespace(id=2)]
    )

    ui._process_visible_tags(message)

    assert sorted(ui.visible_tags) == [2, 9]
    assert calls == ["manipulation", "base", "navigation"]


@pytest.mark.parametrize("waypoint_name,selected_tag", [("", 9), ("Loading bay", 9), ("2", 9), ("17", 0)])
def test_landmark_button_uses_selected_visible_tag_independently_of_waypoint_name(waypoint_name, selected_tag):
    parent = _Parent([0, 2, 9])
    controls = NavigationControls(parent)
    controls.current_map = "plant"
    controls.poi_name_field.setText(waypoint_name)
    controls.landmark_tag_dropdown.setCurrentText(str(selected_tag))

    controls.add_landmark_button.click()

    intent, = parent.submitted
    assert intent.operation == NavigationSetupIntent.OPERATION_ADD_VISIBLE_TAG_LANDMARK
    assert intent.map_name == "plant"
    assert intent.tag_id == selected_tag
    assert intent.waypoint_name == ""
    assert controls.poi_name_field.text() == waypoint_name
    if waypoint_name:
        controls.add_waypoint_button.click()
        assert parent.submitted[-1].operation == NavigationSetupIntent.OPERATION_ADD_CURRENT_WAYPOINT
        assert parent.submitted[-1].waypoint_name == waypoint_name


def test_landmark_selection_updates_and_rejects_a_tag_that_just_disappeared():
    parent = _Parent([2, 9])
    controls = NavigationControls(parent)
    controls.current_map = "plant"
    warnings = []
    controls.show_warning = lambda *args: warnings.append(args)
    controls.landmark_tag_dropdown.setCurrentText("9")
    parent.visible_tags[12] = object()
    controls.update_tags_dropdown()
    assert controls.landmark_tag_dropdown.currentText() == "9"

    del parent.visible_tags[9]
    controls.add_landmark_button.click()
    assert parent.submitted == []
    assert warnings[-1][0] == "No Visible Tag Selected"
    controls.update_tags_dropdown()
    assert controls.landmark_tag_dropdown.findText("9") == -1

    parent.visible_tags.clear()
    controls.update_tags_dropdown()
    assert not controls.landmark_tag_dropdown.isEnabled()
    assert not controls.add_landmark_button.isEnabled()
    parent.visible_tags[0] = object()
    controls.update_tags_dropdown()
    assert controls.landmark_tag_dropdown.isEnabled()
    assert controls.add_landmark_button.isEnabled()
    assert controls.landmark_tag_dropdown.currentText() == "0"


def test_move_to_tag_ui_passes_selected_accuracy_tolerance(monkeypatch):
    from fault_detector_msgs.msg import OperationalIntent
    controls = object.__new__(ManipulationControls)
    controls.ui = _Parent([2])
    controls._make_tag_input_row()
    controls._make_precision_mode_row()
    controls.tag_tolerance_input.setValue(.025)
    monkeypatch.setattr(controls, "add_tag_element_to_intent", lambda intent: intent)
    monkeypatch.setattr(controls, "add_offset_to_intent", lambda intent: intent)
    assert controls.add_tag_info_to_intent(OperationalIntent()).tag_position_tolerance_m == .025
