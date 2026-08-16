"""Tests for authoritative physical sensor attachment state."""

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
)
from fault_detector_spot.application.controllers.sensor_attachment_controller import (
    SensorAttachmentController,
    SensorAttachmentStatus,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    SensorDefinition,
)
from fault_detector_spot.inspection.repository.sensor_attachment_state_store import (
    SensorAttachmentStateStore,
)
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)


def sensor_definition(sensor_id="bmm150_01"):
    """Create one registered hand-mounted sensor."""
    return SensorDefinition(
        sensor_id=sensor_id,
        display_name="BMM150 Hall sensor",
        hand_to_probe=PoseData(
            position=Vector3Data(x=0.2, y=0.01, z=-0.02),
            orientation=QuaternionData.identity(),
        ),
    )


def build_controller(tmp_path):
    """Create isolated repository and attachment state."""
    repository = SensorRepository(
        tmp_path / "sensors",
        tmp_path / "retired_sensors",
    )
    state_store = SensorAttachmentStateStore(
        tmp_path / "attachment.yaml"
    )
    command_controller = CommandController()
    controller = SensorAttachmentController(
        repository,
        state_store,
        command_controller,
    )
    return controller, repository, state_store, command_controller


def test_first_startup_has_no_confirmed_sensor(tmp_path):
    controller, _, _, _ = build_controller(tmp_path)

    state = controller.snapshot()

    assert state.status is SensorAttachmentStatus.NO_SENSOR
    assert state.active_sensor_id == ""
    assert state.pending_sensor_id == ""
    assert state.attachment_revision == 0
    assert state.sensor_dependent_motion_allowed is False

    with pytest.raises(RuntimeError):
        controller.require_confirmed_sensor("bmm150_01")


def test_selecting_registered_sensor_requires_confirmation(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())

    state = controller.select_sensor("bmm150_01")

    assert state.status is SensorAttachmentStatus.CONFIRMATION_PENDING
    assert state.pending_sensor_id == "bmm150_01"
    assert state.attachment_revision == 1
    assert state.sensor_dependent_motion_allowed is False

    with pytest.raises(RuntimeError):
        controller.require_confirmed_sensor("bmm150_01")


def test_exact_confirmation_activates_sensor(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())
    pending = controller.select_sensor("bmm150_01")

    state = controller.confirm_sensor(
        "bmm150_01",
        pending.attachment_revision,
    )
    attachment = controller.require_confirmed_sensor("bmm150_01")

    assert state.status is SensorAttachmentStatus.ACTIVE
    assert state.active_sensor_id == "bmm150_01"
    assert state.pending_sensor_id == ""
    assert attachment.sensor_id == "bmm150_01"
    assert attachment.probe_frame == "bmm150_01_probe"
    assert attachment.hand_to_probe_position == (0.2, 0.01, -0.02)


def test_confirmation_rejects_stale_revision(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())
    pending = controller.select_sensor("bmm150_01")

    with pytest.raises(RuntimeError, match="stale"):
        controller.confirm_sensor(
            "bmm150_01",
            pending.attachment_revision - 1,
        )


def test_confirmation_rejects_wrong_sensor(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())
    repository.create(sensor_definition("thermal_01"))
    pending = controller.select_sensor("bmm150_01")

    with pytest.raises(RuntimeError, match="pending sensor"):
        controller.confirm_sensor(
            "thermal_01",
            pending.attachment_revision,
        )


def test_routine_sensor_must_match_confirmed_attachment(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())
    repository.create(sensor_definition("thermal_01"))
    pending = controller.select_sensor("bmm150_01")
    controller.confirm_sensor(
        "bmm150_01",
        pending.attachment_revision,
    )

    with pytest.raises(RuntimeError, match="requires sensor"):
        controller.require_confirmed_sensor("thermal_01")


def test_restart_restores_selection_as_unconfirmed(tmp_path):
    controller, repository, state_store, command_controller = (
        build_controller(tmp_path)
    )
    repository.create(sensor_definition())
    pending = controller.select_sensor("bmm150_01")
    controller.confirm_sensor(
        "bmm150_01",
        pending.attachment_revision,
    )

    restarted = SensorAttachmentController(
        repository,
        state_store,
        command_controller,
    )
    state = restarted.snapshot()

    assert state.status is SensorAttachmentStatus.CONFIRMATION_PENDING
    assert state.active_sensor_id == ""
    assert state.pending_sensor_id == "bmm150_01"
    assert state.attachment_revision == pending.attachment_revision + 1

    with pytest.raises(RuntimeError):
        restarted.require_confirmed_sensor("bmm150_01")


def test_clear_sensor_disables_sensor_dependent_motion(tmp_path):
    controller, repository, state_store, _ = build_controller(tmp_path)
    repository.create(sensor_definition())
    pending = controller.select_sensor("bmm150_01")
    controller.confirm_sensor(
        "bmm150_01",
        pending.attachment_revision,
    )

    state = controller.clear_sensor()

    assert state.status is SensorAttachmentStatus.NO_SENSOR
    assert state.active_sensor_id == ""
    assert state.pending_sensor_id == ""
    assert state.sensor_dependent_motion_allowed is False
    assert state_store.load().sensor_id == ""


def test_unknown_sensor_selection_does_not_change_state(tmp_path):
    controller, _, _, _ = build_controller(tmp_path)
    before = controller.snapshot()

    with pytest.raises(FileNotFoundError):
        controller.select_sensor("missing_sensor")

    assert controller.snapshot() == before


def test_sensor_change_is_rejected_while_command_lane_has_work(tmp_path):
    controller, repository, _, command_controller = build_controller(
        tmp_path
    )
    repository.create(sensor_definition())
    request = CommandRequest.create(
        command=SemanticCommand(command_id=CommandID.STAND_UP),
        client_id="test",
        origin=CommandOrigin.SYSTEM,
    )
    command_controller.submit(request)

    with pytest.raises(RuntimeError, match="active or queued"):
        controller.select_sensor("bmm150_01")


def test_confirmed_geometry_snapshot_is_independent(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())
    pending = controller.select_sensor("bmm150_01")
    controller.confirm_sensor(
        "bmm150_01",
        pending.attachment_revision,
    )

    attachment = controller.require_confirmed_sensor("bmm150_01")
    pose = attachment.hand_to_probe()
    pose.position.x = 99.0
    refreshed = controller.require_confirmed_sensor("bmm150_01")

    assert refreshed.hand_to_probe_position[0] == pytest.approx(0.2)
