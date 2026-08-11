"""Request identities for correlated command dispatch and completion."""

from uuid import UUID, uuid4


def new_request_id() -> str:
    """Return one canonical random request identity."""
    return str(uuid4())


def validate_request_id(request_id: str) -> str:
    """Validate and normalize one non-empty canonical UUID string."""
    if not isinstance(request_id, str):
        raise TypeError("Request ID must be a string")
    value = request_id.strip()
    if not value:
        raise ValueError("Request ID must not be empty")
    try:
        parsed = UUID(value)
    except (AttributeError, TypeError, ValueError) as exception:
        raise ValueError("Request ID must be a UUID") from exception
    canonical = str(parsed)
    if value != canonical:
        raise ValueError("Request ID must use canonical UUID form")
    return canonical


def request_id_or_new(request_id: str) -> str:
    """Normalize a supplied identity or create one for legacy publishers."""
    value = request_id.strip() if isinstance(request_id, str) else ""
    return validate_request_id(value) if value else new_request_id()
