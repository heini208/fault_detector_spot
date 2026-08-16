"""Guard application ownership of registry and attachment authority."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_application_owns_registry_and_attachment_with_one_repository():
    source = (
        ROOT
        / "fault_detector_spot/application/api/application_api_node.py"
    ).read_text(encoding="utf-8")

    assert "sensor_repository = SensorRepository" in source
    assert "SensorAttachmentController(" in source
    assert "SensorRegistryController(" in source
    assert "SensorRegistryApi(" in source


def test_registry_transport_has_no_attachment_state_subscription():
    source = (
        ROOT / "fault_detector_spot/application/api/sensor_registry_api.py"
    ).read_text(encoding="utf-8")

    assert "SensorAttachmentState" not in source
    assert "create_subscription" not in source
    assert "_attachment_state" not in source


def test_ui_registry_and_attachment_transports_are_independent():
    registry = (
        ROOT
        / "fault_detector_spot/ui/ros/sensor_registry_client.py"
    ).read_text(encoding="utf-8")
    attachment = (
        ROOT
        / "fault_detector_spot/ui/ros/sensor_attachment_client.py"
    ).read_text(encoding="utf-8")
    models = (
        ROOT
        / "fault_detector_spot/ui/sensor/models.py"
    ).read_text(encoding="utf-8")

    assert "class SensorRegistryClient" in registry
    assert "SensorDefinitionArray" in registry
    assert "AddSensor" in registry
    assert "SelectSensorAttachment" not in registry
    assert "SensorAttachmentState" not in registry

    assert "class SensorAttachmentClient" in attachment
    assert "SensorAttachmentState" in attachment
    assert "SelectSensorAttachment" in attachment
    assert "SensorDefinitionArray" not in attachment
    assert "AddSensor" not in attachment
    assert "DeleteSensor" not in attachment

    assert "fault_detector_msgs" not in models
    assert "PyQt5" not in models


def test_standalone_registry_process_is_removed():
    launch = (ROOT / "launch/fault_detector_launch.py").read_text(
        encoding="utf-8"
    )
    setup = (ROOT / "setup.py").read_text(encoding="utf-8")

    assert 'executable="sensor_registry"' not in launch
    assert "'sensor_registry =" not in setup
