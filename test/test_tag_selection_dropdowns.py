"""Tests for visible-tag selection in the movement controls."""

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


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


class _Parent:

    def __init__(self, tag_ids):
        self.visible_tags = {tag_id: object() for tag_id in tag_ids}

    def update_tags_dropdown(self, dropdown):
        Fault_Detector_UI.update_tags_dropdown(self, dropdown)


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


def test_visible_tag_message_refreshes_both_movement_dropdowns():
    calls = []
    ui = Fault_Detector_UI.__new__(Fault_Detector_UI)
    ui.manipulation_controls = SimpleNamespace(
        update_tags_dropdown=lambda: calls.append("manipulation")
    )
    ui.base_movement_controls = SimpleNamespace(
        update_tags_dropdown=lambda: calls.append("base")
    )
    message = SimpleNamespace(
        elements=[SimpleNamespace(id=9), SimpleNamespace(id=2)]
    )

    ui._process_visible_tags(message)

    assert sorted(ui.visible_tags) == [2, 9]
    assert calls == ["manipulation", "base"]
