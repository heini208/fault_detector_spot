"""Client identities for application-owned requests."""


def required_client_id(value: str) -> str:
    """Return one normalized, non-empty client identifier."""
    if not isinstance(value, str):
        raise TypeError("Client ID must be a string")
    normalized = value.strip()
    if not normalized:
        raise ValueError("Client ID must not be empty")
    return normalized


__all__ = ["required_client_id"]
