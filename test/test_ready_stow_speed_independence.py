from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_ready_arm_has_dedicated_linear_speed():
    config = (ROOT / "config" / "arm_motion.yaml").read_text(encoding="utf-8")
    executor = (
        ROOT
        / "fault_detector_spot"
        / "manipulation"
        / "arm_movement_executor.py"
    ).read_text(encoding="utf-8")

    assert "arm.ready_linear_speed_mps: 0.08" in config
    assert 'config.get("ready_linear_speed_mps")' in executor
    assert "speed if speed is not None else self.ready_speed" in executor
    assert "self._operation_speed = self.ready_speed" in executor


def test_stow_uses_native_spot_command_without_general_motion_speed():
    executor = (
        ROOT
        / "fault_detector_spot"
        / "manipulation"
        / "arm_movement_executor.py"
    ).read_text(encoding="utf-8")

    start = executor.index("    def _build_stow_goal")
    end = executor.index("    def _build_moveit_joint_goal", start)
    stow_builder = executor[start:end]

    assert "RobotCommandBuilder.arm_stow_command()" in stow_builder
    assert "motion.linear_speed_mps" not in stow_builder
    assert "ready_speed" not in stow_builder
