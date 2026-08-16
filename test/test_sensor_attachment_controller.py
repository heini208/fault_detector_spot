"""Tests for authoritative physical sensor attachment state."""

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
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
    BARE_HAND_MOTION_ID,
    SensorDefinition,
)
from fault_detector_spot.inspection.repository.sensor_attachment_state_store import (
    PersistedSensorAttachmentSelection,
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


def test_first_startup_requires_bare_hand_confirmation(tmp_path):
    controller, repository, state_store, _ = build_controller(tmp_path)

    state = controller.snapshot()

    assert state.status is SensorAttachmentStatus.NO_SENSOR
    assert state.active_sensor_id == ""
    assert state.pending_sensor_id == ""
    assert state.attachment_revision == 0
    assert state.sensor_dependent_motion_allowed is False
    assert repository.list_sensor_ids() == []
    assert state_store.load() is None

    with pytest.raises(RuntimeError, match="confirmation is pending"):
        controller.require_motion_attachment()


def test_no_sensor_confirmation_activates_bare_hand_geometry(tmp_path):
    controller, _, state_store, _ = build_controller(tmp_path)
    initial = controller.snapshot()

    state = controller.confirm_sensor(
        "",
        initial.attachment_revision,
    )
    attachment = controller.require_motion_attachment()

    assert state.status is SensorAttachmentStatus.ACTIVE
    assert state.active_sensor_id == ""
    assert state.pending_sensor_id == ""
    assert state.sensor_dependent_motion_allowed is True
    assert state_store.load() is None
    assert attachment.has_sensor is False
    assert attachment.sensor_id == ""
    assert attachment.motion_sensor_id == BARE_HAND_MOTION_ID
    assert attachment.probe_frame == "hand"
    assert attachment.hand_to_probe_position == (0.0, 0.0, 0.0)
    assert attachment.hand_to_probe_orientation == (0.0, 0.0, 0.0, 1.0)


def test_no_sensor_confirmation_rejects_sensor_id(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())

    with pytest.raises(RuntimeError, match="pending no-sensor state"):
        controller.confirm_sensor(
            "bmm150_01",
            controller.snapshot().attachment_revision,
        )


def test_selecting_registered_sensor_requires_confirmation(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())

    state = controller.select_sensor("bmm150_01")

    assert state.status is SensorAttachmentStatus.CONFIRMATION_PENDING
    assert state.pending_sensor_id == "bmm150_01"
    assert state.attachment_revision == 1
    assert state.sensor_dependent_motion_allowed is False

    with pytest.raises(RuntimeError, match="confirmation is pending"):
        controller.require_motion_attachment()


def test_exact_confirmation_activates_sensor(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())
    pending = controller.select_sensor("bmm150_01")

    state = controller.confirm_sensor(
        "bmm150_01",
        pending.attachment_revision,
    )
    attachment = controller.require_motion_attachment()

    assert state.status is SensorAttachmentStatus.ACTIVE
    assert state.active_sensor_id == "bmm150_01"
    assert state.pending_sensor_id == ""
    assert attachment.has_sensor is True
    assert attachment.sensor_id == "bmm150_01"
    assert attachment.motion_sensor_id == "bmm150_01"
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


def test_explicit_expected_sensor_must_match_confirmed_attachment(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())
    repository.create(sensor_definition("thermal_01"))
    pending = controller.select_sensor("bmm150_01")
    controller.confirm_sensor(
        "bmm150_01",
        pending.attachment_revision,
    )

    with pytest.raises(RuntimeError, match="Expected sensor"):
        controller.require_confirmed_sensor("thermal_01")


def test_restart_restores_real_sensor_as_unconfirmed(tmp_path):
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

    with pytest.raises(RuntimeError, match="confirmation is pending"):
        restarted.require_motion_attachment()


def test_remove_sensor_requires_bare_hand_confirmation(tmp_path):
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
    assert state_store.load() is None

    with pytest.raises(RuntimeError, match="confirmation is pending"):
        controller.require_motion_attachment()

    confirmed = controller.confirm_sensor(
        "",
        state.attachment_revision,
    )
    attachment = controller.require_motion_attachment()

    assert confirmed.status is SensorAttachmentStatus.ACTIVE
    assert attachment.motion_sensor_id == BARE_HAND_MOTION_ID
    assert attachment.probe_frame == "hand"


def test_restart_requires_no_sensor_reconfirmation(tmp_path):
    controller, repository, state_store, command_controller = (
        build_controller(tmp_path)
    )
    controller.confirm_sensor("", controller.snapshot().attachment_revision)

    restarted = SensorAttachmentController(
        repository,
        state_store,
        command_controller,
    )
    state = restarted.snapshot()

    assert state.status is SensorAttachmentStatus.NO_SENSOR
    assert state.attachment_revision == 0
    assert state_store.load() is None
    with pytest.raises(RuntimeError, match="confirmation is pending"):
        restarted.require_motion_attachment()


def test_legacy_builtin_hand_selection_is_normalized_to_no_sensor(tmp_path):
    repository = SensorRepository(tmp_path / "sensors")
    state_store = SensorAttachmentStateStore(tmp_path / "attachment.yaml")
    state_store.save(
        PersistedSensorAttachmentSelection(
            sensor_id=BARE_HAND_MOTION_ID,
            attachment_revision=7,
        )
    )

    controller = SensorAttachmentController(
        repository,
        state_store,
        CommandController(),
    )

    assert controller.snapshot().status is SensorAttachmentStatus.NO_SENSOR
    assert controller.snapshot().attachment_revision == 7
    assert state_store.load() is None
    with pytest.raises(RuntimeError, match="confirmation is pending"):
        controller.require_motion_attachment()


def test_restart_drops_selection_for_deleted_sensor(tmp_path):
    repository = SensorRepository(tmp_path / "sensors")
    state_store = SensorAttachmentStateStore(tmp_path / "attachment.yaml")
    state_store.save(
        PersistedSensorAttachmentSelection(
            sensor_id="deleted_sensor",
            attachment_revision=5,
        )
    )

    controller = SensorAttachmentController(
        repository,
        state_store,
        CommandController(),
    )

    state = controller.snapshot()
    assert state.status is SensorAttachmentStatus.NO_SENSOR
    assert state.active_sensor_id == ""
    assert state.pending_sensor_id == ""
    assert state.attachment_revision == 5
    assert state_store.load() is None
    with pytest.raises(RuntimeError, match="confirmation is pending"):
        controller.require_motion_attachment()


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
        recording_policy=RecordingPolicy.EXCLUDE,
    )
    command_controller.submit(request)

    with pytest.raises(RuntimeError, match="active or queued"):
        controller.select_sensor("bmm150_01")
    with pytest.raises(RuntimeError, match="active or queued"):
        controller.clear_sensor()


def test_confirmed_geometry_snapshot_is_independent(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())
    pending = controller.select_sensor("bmm150_01")
    controller.confirm_sensor(
        "bmm150_01",
        pending.attachment_revision,
    )

    attachment = controller.require_motion_attachment()
    pose = attachment.hand_to_probe()
    pose.position.x = 99.0
    refreshed = controller.require_motion_attachment()

    assert refreshed.hand_to_probe_position[0] == pytest.approx(0.2)


def test_move_to_tag_request_is_bound_to_confirmed_sensor(tmp_path):
    controller, repository, _, command_controller = build_controller(
        tmp_path
    )
    repository.create(sensor_definition())
    pending = controller.select_sensor("bmm150_01")
    controller.confirm_sensor(
        "bmm150_01",
        pending.attachment_revision,
    )
    command_controller.add_request_preparer(controller.prepare_request)
    accepted = []
    command_controller.add_accepted_listener(accepted.append)
    request = CommandRequest.create(
        command=SemanticCommand(command_id=CommandID.MOVE_ARM_TO_TAG),
        client_id="test",
        origin=CommandOrigin.SYSTEM,
        recording_policy=RecordingPolicy.EXCLUDE,
    )

    command_controller.submit(request)

    assert accepted[0].command.motion_sensor_id == "bmm150_01"


def test_move_to_tag_requires_then_uses_confirmed_bare_hand(tmp_path):
    controller, _, _, command_controller = build_controller(tmp_path)
    command_controller.add_request_preparer(controller.prepare_request)
    accepted = []
    command_controller.add_accepted_listener(accepted.append)
    request = CommandRequest.create(
        command=SemanticCommand(command_id=CommandID.MOVE_ARM_TO_TAG),
        client_id="test",
        origin=CommandOrigin.SYSTEM,
        recording_policy=RecordingPolicy.EXCLUDE,
    )

    with pytest.raises(RuntimeError, match="confirmation is pending"):
        command_controller.submit(request)

    controller.confirm_sensor("", controller.snapshot().attachment_revision)
    command_controller.submit(request)

    assert accepted[0].command.motion_sensor_id == BARE_HAND_MOTION_ID


def test_explicit_stale_motion_attachment_is_rejected(tmp_path):
    controller, repository, _, command_controller = build_controller(
        tmp_path
    )
    repository.create(sensor_definition())
    repository.create(sensor_definition("thermal_01"))
    pending = controller.select_sensor("bmm150_01")
    controller.confirm_sensor(
        "bmm150_01",
        pending.attachment_revision,
    )
    command_controller.add_request_preparer(controller.prepare_request)
    request = CommandRequest.create(
        command=SemanticCommand(
            command_id=CommandID.MOVE_ARM_RELATIVE,
            motion_sensor_id="thermal_01",
        ),
        client_id="test",
        origin=CommandOrigin.SYSTEM,
        recording_policy=RecordingPolicy.EXCLUDE,
    )

    with pytest.raises(RuntimeError, match="prepared for attachment"):
        command_controller.submit(request)

    assert command_controller.active_request_id == ""
    assert command_controller.queued_request_ids == ()


def test_sensor_change_is_rejected_during_attachment_reservation(tmp_path):
    controller, repository, _, _ = build_controller(tmp_path)
    repository.create(sensor_definition())
    pending = controller.select_sensor("bmm150_01")
    controller.confirm_sensor(
        "bmm150_01",
        pending.attachment_revision,
    )

    with controller.reserve_motion_attachment() as attachment:
        assert attachment.sensor_id == "bmm150_01"
        with pytest.raises(RuntimeError, match="workflow is active"):
            controller.clear_sensor()

    state = controller.clear_sensor()
    assert state.status is SensorAttachmentStatus.NO_SENSOR
