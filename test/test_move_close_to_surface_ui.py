"""UI wiring regressions for surface movement controls."""

from pathlib import Path


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def _controls_source():
    return (ROOT / "ui/manipulation/controls.py").read_text(encoding="utf-8")


def test_manipulation_ui_submits_distance_and_tolerance():
    source = _controls_source()
    start = source.index("    def handle_move_close_to_surface(self):")
    end = source.index("\n    def handle_move_to_tag_surface", start)
    handler = source[start:end]

    assert "INTENT_MOVE_CLOSE_TO_SURFACE" in handler
    assert "surface_distance_input.value()" in handler
    assert "surface_tolerance_input.value()" in handler
    assert "target_surface_distance_m" in handler
    assert "surface_tolerance_m" in handler
    assert "execute_operation(intent)" in handler
    assert "show_setup_unavailable" not in handler


def test_tag_surface_test_control_is_next_to_move_close_to_surface():
    source = _controls_source()
    start = source.index("    def _make_surface_action_row(self)")
    end = source.index("\n    def handle_orient_to_surface", start)
    row = source[start:end]

    close_index = row.index("Move Close to Surface")
    tag_surface_index = row.index("Move to Tag Surface")

    assert close_index < tag_surface_index
    assert 'QLabel("ⓘ")' in row
    assert "TAG_SURFACE_TEST_INFO" in row
    assert "handle_move_to_tag_surface" in row


def test_tag_surface_test_payload_uses_only_yz_in_tag_frame():
    source = _controls_source()
    start = source.index("    def handle_move_to_tag_surface(self):")
    end = source.index("\n    def _reset_all_zero", start)
    handler = source[start:end]

    assert '"INTENT_MOVE_TO_TAG_SURFACE"' in handler
    assert "add_tag_element_to_intent(intent)" in handler
    assert 'intent.offset.header.frame_id = "tag"' in handler
    assert "intent.offset.pose.position.x = 0.0" in handler
    assert 'self._get_offset("Y")' in handler
    assert 'self._get_offset("Z")' in handler
    assert 'self._get_offset("X")' not in handler
    assert "frames_dropdown" not in handler
    assert "orientation_combo" not in handler
    assert "surface_distance_input.value()" in handler
    assert "surface_tolerance_input.value()" in handler
    assert "execute_operation(intent)" in handler


def test_tag_surface_info_explains_temporary_input_semantics():
    source = _controls_source()

    assert "Arm offset X (backward/forward)" in source
    assert "Y/Z are always" in source
    assert "interpreted in the tag frame" in source
    assert "reference-frame point instead" in source
