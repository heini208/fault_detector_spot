"""Shared utilities for persistent inspection repositories."""

import os
import tempfile
from pathlib import Path


def validate_storage_name(name: str, field_name: str) -> str:
    """Validate a name used as a file or directory component."""
    if not isinstance(name, str):
        raise TypeError(f"{field_name} must be a string")

    if not name:
        raise ValueError(f"{field_name} must not be empty")

    if name != name.strip():
        raise ValueError(
            f"{field_name} must not contain surrounding whitespace"
        )

    if name in {".", ".."}:
        raise ValueError(f"Invalid {field_name}: {name}")

    if "/" in name or "\\" in name or "\x00" in name:
        raise ValueError(f"Invalid {field_name}: {name}")

    return name


def atomic_write_text(path: Path, content: str) -> None:
    """Atomically replace a UTF-8 text file."""
    path.parent.mkdir(parents=True, exist_ok=True)

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=str(path.parent),
        text=True,
    )

    try:
        with os.fdopen(
            descriptor,
            "w",
            encoding="utf-8",
        ) as temporary_file:
            temporary_file.write(content)
            temporary_file.flush()
            os.fsync(temporary_file.fileno())

        os.replace(temporary_name, path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise