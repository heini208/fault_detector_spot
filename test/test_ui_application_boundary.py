"""Guard the remote UI application boundary."""

from pathlib import Path


UI_ROOT = Path(__file__).parents[1] / "fault_detector_spot" / "ui"
OPERATIONAL_UI_FILES = (
    UI_ROOT / "fault_detector_ui.py",
    UI_ROOT / "manipulation" / "controls.py",
    UI_ROOT / "navigation" / "base_movement_controls.py",
    UI_ROOT / "navigation" / "controls.py",
    UI_ROOT / "inspection" / "controls.py",
)


def test_operational_ui_does_not_use_internal_command_messages():
    forbidden = (
        "BasicCommand",
        "ComplexCommand",
        "CommandStatus",
        "build_basic_command",
        "complex_command_publisher",
    )

    for path in OPERATIONAL_UI_FILES:
        source = path.read_text(encoding="utf-8")
        for value in forbidden:
            assert value not in source, f"{value} remains in {path.name}"


def test_operational_ui_does_not_reference_internal_command_topics():
    forbidden_topics = (
        "fault_detector/commands/basic_command",
        "fault_detector/commands/complex_command",
        "fault_detector/_internal/commands",
        "fault_detector/_internal/command_status",
        "fault_detector/command_status",
        "fault_detector/command_tree_status",
    )

    for path in OPERATIONAL_UI_FILES:
        source = path.read_text(encoding="utf-8")
        for topic in forbidden_topics:
            assert topic not in source, f"{topic} remains in {path.name}"
