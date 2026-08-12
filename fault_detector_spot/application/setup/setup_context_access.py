"""Shared setup context lookup and ownership checks."""

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.application.setup.setup_context import (
    SetupContextSnapshot,
    setup_origin,
    validate_context_id,
)


class SetupContextAccess:
    def __init__(self, setup_coordinator):
        self.setup_coordinator = setup_coordinator

    def contexts_for(self, origin) -> tuple:
        origin = setup_origin(origin)
        return tuple(
            context
            for context in self.setup_coordinator.contexts
            if context.origin is origin
        )

    def resolve(
        self,
        context_id: str,
        client_id: str,
        origin=None,
    ) -> SetupContextSnapshot:
        context_id = validate_context_id(context_id)
        client_id = required_client_id(client_id)
        expected_origin = (
            setup_origin(origin)
            if origin is not None
            else None
        )
        context = next(
            (
                candidate
                for candidate in self.setup_coordinator.contexts
                if candidate.context_id == context_id
            ),
            None,
        )
        if (
            context is None
            or (
                expected_origin is not None
                and context.origin is not expected_origin
            )
        ):
            raise LookupError(
                f"Unknown setup context: {context_id}"
            )
        self.setup_coordinator.require_current(context)
        if context.client_id != client_id:
            raise ValueError(
                "Client ID does not own the setup context"
            )
        return context


__all__ = ["SetupContextAccess"]
