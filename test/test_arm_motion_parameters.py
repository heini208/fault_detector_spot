"""YAML is the sole arm default source, including standalone constructors."""

from types import SimpleNamespace

import pytest
import yaml

from fault_detector_spot.manipulation import (
    arm_motion_parameters as parameters_module,
)
from fault_detector_spot.manipulation.arm_motion_parameters import (
    ArmMotionParameters,
)
from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeed, ArmMotionSpeedPolicy,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)
from fault_detector_spot.manipulation.arm_force_baseline import (
    ForceBaselineSampler,
)
from fault_detector_spot.manipulation.hand_settling_detector import (
    HandSettlingDetector,
)
from fault_detector_spot.manipulation.force_contact_policy import (
    SpeedAwareForceContactPolicy,
)
from fault_detector_spot.manipulation.arm_contact_evidence import (
    ArmContactEvidenceAnalyzer,
)
from fault_detector_spot.manipulation.arm_contact_telemetry import (
    ArmContactTelemetry,
)


class FakeNode:
    def __init__(self, overrides=None):
        self.overrides = overrides or {}
        self.parameters = {}

    def has_parameter(self, name):
        return name in self.parameters

    def declare_parameter(self, name, default):
        assert name not in self.parameters
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])


def write_config(tmp_path, values):
    path = tmp_path / "arm_motion.yaml"
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": values}}))
    return path


def test_new_yaml_key_is_declared_without_a_python_registry(tmp_path):
    path = write_config(tmp_path, {"arm.new_setting": 0.37})
    node = FakeNode()
    config = ArmMotionParameters(node, path=path)
    assert config.get("new_setting") == 0.37
    assert node.parameters == {"arm.new_setting": 0.37}
    # Sharing a node never redeclares an existing parameter.
    ArmMotionParameters(node, path=path)


def test_explicit_then_ros_then_yaml_precedence(tmp_path):
    path = write_config(tmp_path, {"arm.new_setting": 0.37})
    config = ArmMotionParameters(
        FakeNode({"arm.new_setting": 0.42}), path=path
    )
    assert config.get("new_setting") == 0.42
    assert config.get("new_setting", 0.0) == 0.0
    assert ArmMotionParameters(path=path).get("new_setting") == 0.37


def test_missing_key_has_no_python_fallback(tmp_path):
    config = ArmMotionParameters(
        path=write_config(tmp_path, {"arm.present": 1})
    )
    with pytest.raises(ValueError, match="Missing parameter 'arm.missing'"):
        config.get("missing")


@pytest.mark.parametrize(
    "value", [True, "fast", None, float("nan"), float("inf")]
)
def test_invalid_ros_values_are_rejected(tmp_path, value):
    path = write_config(tmp_path, {"arm.speed": 0.1})
    config = ArmMotionParameters(FakeNode({"arm.speed": value}), path=path)
    with pytest.raises(ValueError, match="arm.speed"):
        config.get("speed")


def test_yaml_boolean_false_is_not_replaced(tmp_path):
    path = write_config(tmp_path, {"arm.enabled": True})
    config = ArmMotionParameters(FakeNode({"arm.enabled": False}), path=path)
    assert config.get("enabled") is False


def test_malformed_yaml_structure_fails_clearly(tmp_path):
    path = tmp_path / "arm_motion.yaml"
    path.write_text("wrong: structure\n")
    with pytest.raises(ValueError, match="ros__parameters"):
        ArmMotionParameters(path=path)


def test_installed_package_uses_share_config(monkeypatch, tmp_path):
    import ament_index_python.packages

    monkeypatch.setattr(
        parameters_module, "__file__", str(tmp_path / "lib/pkg/module.py")
    )
    monkeypatch.setattr(
        ament_index_python.packages, "get_package_share_directory",
        lambda name: str(tmp_path / "share" / name),
    )
    assert parameters_module.arm_motion_config_path() == (
        tmp_path / "share/fault_detector_spot/config/arm_motion.yaml"
    )


def test_missing_checkout_yaml_does_not_use_stale_installed_config(
    monkeypatch, tmp_path,
):
    (tmp_path / "config").mkdir()
    monkeypatch.setattr(
        parameters_module, "__file__", str(tmp_path / "pkg/arm/module.py")
    )
    with pytest.raises(FileNotFoundError, match="arm_motion.yaml"):
        ArmMotionParameters()


def test_standalone_consumers_use_changed_yaml(monkeypatch, tmp_path):
    values = yaml.safe_load(
        parameters_module.arm_motion_config_path().read_text()
    )["/**"]["ros__parameters"]
    values.update({
        "arm.ready_forward_distance_m": 0.23,
        "arm.ready_lift_distance_m": 0.31,
        "arm.motion.linear_speed_mps": 0.17,
        "arm.motion.angular_speed_rad_s": 0.61,
        "arm.motion.minimum_duration_sec": 0.71,
        "arm.force_baseline.minimum_samples": 13,
        "arm.settling.stable_duration_sec": 0.63,
        "arm.contact.consecutive_samples": 4,
        "arm.contact.shadow.off_axis_speed_threshold_mps": 0.07,
        "arm.contact.telemetry.raw_logging_enabled": False,
    })
    path = write_config(tmp_path, values)
    monkeypatch.setattr(
        parameters_module, "arm_motion_config_path", lambda: path
    )
    executor = ArmMovementExecutor(object(), action_client=object())
    assert executor.ready_forward_distance_m == 0.23
    assert executor.ready_lift_distance_m == 0.31
    assert executor.speed_policy.default_speed.linear_speed_mps == 0.17
    assert ArmMotionSpeed().angular_speed_rad_s == 0.61
    assert ArmMotionSpeedPolicy().minimum_duration_sec == 0.71
    assert ForceBaselineSampler(object()).minimum_samples == 13
    assert HandSettlingDetector(object(), object()).stable_duration_sec == 0.63
    assert SpeedAwareForceContactPolicy().consecutive_samples == 4
    assert ArmContactEvidenceAnalyzer().off_axis_speed_threshold_mps == 0.07
    telemetry = ArmContactTelemetry(object(), object(), raw_log_root=tmp_path)
    assert telemetry.raw_logging_enabled is False


def test_ros_overrides_reach_consumers():
    node = FakeNode({
        "arm.motion.linear_speed_mps": 0.19,
        "arm.settling.stable_duration_sec": 0.67,
        "arm.force_baseline.minimum_samples": 17,
        "arm.contact.consecutive_samples": 6,
        "arm.contact.shadow.off_axis_speed_threshold_mps": 0.09,
        "arm.ready_forward_distance_m": 0.0,
    })
    config = ArmMotionParameters(node)
    executor = ArmMovementExecutor(
        object(), action_client=object(), config=config
    )
    assert executor.ready_forward_distance_m == 0.0
    assert executor.speed_policy.default_speed.linear_speed_mps == 0.19
    settling = HandSettlingDetector.from_node(node, object(), object())
    assert settling.stable_duration_sec == 0.67
    assert ForceBaselineSampler.from_node(node, object()).minimum_samples == 17
    contact = SpeedAwareForceContactPolicy.from_node(node)
    assert contact.consecutive_samples == 6
    evidence = ArmContactEvidenceAnalyzer.from_node(node)
    assert evidence.off_axis_speed_threshold_mps == 0.09
