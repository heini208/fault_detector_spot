import types
from dataclasses import fields, is_dataclass
from enum import Enum
from typing import Any, List, Literal, Union, get_args, get_origin, get_type_hints

from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)


_TRANSIENT_FIELDS = {
    SemanticCommand: frozenset({
        "motion_sensor_id",
    }),
}


def serialize_recorded_command(command: SemanticCommand) -> dict:
    if not isinstance(command, SemanticCommand):
        raise TypeError("Recorded command must be a SemanticCommand")
    return _serialize_dataclass(command)


def deserialize_recorded_command(data: dict) -> SemanticCommand:
    return _deserialize_dataclass(
        SemanticCommand,
        _object(data, "Recorded command"),
    )


def deserialize_recording(document) -> List[SemanticCommand]:
    if not isinstance(document, dict):
        raise ValueError("Recording must be an object")
    entries = document.get("commands")
    if not isinstance(entries, list):
        raise ValueError("Recording commands must be a list")
    return [deserialize_recorded_command(entry) for entry in entries]


def _serialize_dataclass(value) -> dict:
    return {
        field.name: _serialize_value(getattr(value, field.name))
        for field in _recordable_fields(type(value))
    }


def _serialize_value(value):
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return _serialize_dataclass(value)
    if isinstance(value, list):
        return [_serialize_value(item) for item in value]
    if isinstance(value, tuple):
        return [_serialize_value(item) for item in value]
    if isinstance(value, dict):
        if not all(isinstance(key, str) for key in value):
            raise TypeError("Recorded dictionaries require string keys")
        return {
            key: _serialize_value(item)
            for key, item in value.items()
        }
    if isinstance(value, (set, frozenset)):
        return [_serialize_value(item) for item in value]
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    raise TypeError(
        "Unsupported recorded value type: "
        f"{type(value).__name__}"
    )


def _deserialize_dataclass(cls, data: dict):
    recordable_fields = _recordable_fields(cls)
    expected = {field.name for field in recordable_fields}
    actual = set(data)

    missing = expected - actual
    if missing:
        raise ValueError(
            "Recorded command is missing field(s): "
            + ", ".join(sorted(missing))
        )

    unexpected = actual - expected
    if unexpected:
        raise ValueError(
            "Recorded command contains unexpected field(s): "
            + ", ".join(sorted(unexpected))
        )

    type_hints = get_type_hints(cls)
    values = {
        field.name: _deserialize_value(
            type_hints[field.name],
            data[field.name],
            f"{cls.__name__}.{field.name}",
        )
        for field in recordable_fields
    }
    return cls(**values)


def _deserialize_value(annotation, value, label):
    if annotation is Any:
        return value

    origin = get_origin(annotation)
    arguments = get_args(annotation)

    if origin in (Union, types.UnionType):
        if value is None and type(None) in arguments:
            return None
        for candidate in arguments:
            if candidate is type(None):
                continue
            try:
                return _deserialize_value(candidate, value, label)
            except (TypeError, ValueError):
                continue
        raise TypeError(
            f"{label} does not match any supported union type"
        )

    if is_dataclass(annotation):
        return _deserialize_dataclass(
            annotation,
            _object(value, label),
        )

    if isinstance(annotation, type) and issubclass(annotation, Enum):
        return annotation(value)

    if origin is list:
        if not isinstance(value, list):
            raise TypeError(f"{label} must be a list")
        item_type = arguments[0] if arguments else Any
        return [
            _deserialize_value(
                item_type,
                item,
                f"{label}[{index}]",
            )
            for index, item in enumerate(value)
        ]

    if origin is tuple:
        if not isinstance(value, list):
            raise TypeError(f"{label} must be a list")
        if len(arguments) == 2 and arguments[1] is Ellipsis:
            return tuple(
                _deserialize_value(
                    arguments[0],
                    item,
                    f"{label}[{index}]",
                )
                for index, item in enumerate(value)
            )
        if arguments and len(arguments) != len(value):
            raise ValueError(
                f"{label} has the wrong number of tuple items"
            )
        item_types = arguments or (Any,) * len(value)
        return tuple(
            _deserialize_value(
                item_type,
                item,
                f"{label}[{index}]",
            )
            for index, (item_type, item) in enumerate(
                zip(item_types, value)
            )
        )

    if origin is dict:
        if not isinstance(value, dict):
            raise TypeError(f"{label} must be an object")
        key_type, value_type = (
            arguments if len(arguments) == 2 else (Any, Any)
        )
        return {
            _deserialize_value(
                key_type,
                key,
                f"{label}.key",
            ): _deserialize_value(
                value_type,
                item,
                f"{label}[{key!r}]",
            )
            for key, item in value.items()
        }

    if origin in (set, frozenset):
        if not isinstance(value, list):
            raise TypeError(f"{label} must be a list")
        item_type = arguments[0] if arguments else Any
        items = (
            _deserialize_value(
                item_type,
                item,
                f"{label}[{index}]",
            )
            for index, item in enumerate(value)
        )
        return origin(items)

    if origin is Literal:
        if value not in arguments:
            raise ValueError(
                f"{label} is not one of the supported literal values"
            )
        return value

    if annotation is str:
        if not isinstance(value, str):
            raise TypeError(f"{label} must be a string")
        return value

    if annotation is bool:
        if not isinstance(value, bool):
            raise TypeError(f"{label} must be a boolean")
        return value

    if annotation is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{label} must be an integer")
        return value

    if annotation is float:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
        ):
            raise TypeError(f"{label} must be a number")
        return float(value)

    if annotation is type(None):
        if value is not None:
            raise TypeError(f"{label} must be null")
        return None

    raise TypeError(
        f"Unsupported recorded field type for {label}: "
        f"{annotation!r}"
    )


def _recordable_fields(cls):
    transient = _TRANSIENT_FIELDS.get(cls, frozenset())
    return tuple(
        field
        for field in fields(cls)
        if field.name not in transient
    )


def _object(value, label):
    if not isinstance(value, dict):
        raise TypeError(f"{label} must be an object")
    return value
