"""Tests for shared setup operation bookkeeping."""

from uuid import uuid4

from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
)
from fault_detector_spot.application.commanding.request_identity import (
    new_request_id,
)
from fault_detector_spot.application.setup.setup_context import (
    SetupContextSnapshot,
)
from fault_detector_spot.application.setup.setup_operation_registry import (
    SetupOperationRegistry,
)


def context(
    origin=CommandOrigin.PROBE_SETUP,
    client_id="setup-ui",
    revision=0,
):
    return SetupContextSnapshot(
        context_id=str(uuid4()),
        client_id=client_id,
        origin=origin,
        revision=revision,
    )


def test_operation_registry_tracks_context_and_payload():
    registry = SetupOperationRegistry()
    owner = context()
    request_id = new_request_id()

    tracked = registry.register(
        request_id,
        owner,
        {"kind": "motion"},
    )

    assert registry.get(request_id) == tracked
    assert registry.owned(request_id, owner) == tracked
    assert tracked.payload == {"kind": "motion"}


def test_operation_registry_discards_by_context_identity():
    registry = SetupOperationRegistry()
    owner = context()
    request_id = new_request_id()
    registry.register(request_id, owner, "motion")

    newer = SetupContextSnapshot(
        context_id=owner.context_id,
        client_id=owner.client_id,
        origin=owner.origin,
        revision=owner.revision + 1,
    )

    assert registry.owned(request_id, newer) is None
    assert registry.request_ids_for(newer) == (request_id,)

    registry.discard_context(newer)

    assert registry.get(request_id) is None


def test_operation_registry_owns_listener_deduplication():
    registry = SetupOperationRegistry()
    received = []

    registry.add_listener(received.append)
    registry.add_listener(received.append)
    registry.emit("status")
    registry.remove_listener(received.append)
    registry.emit("ignored")

    assert received == ["status"]
