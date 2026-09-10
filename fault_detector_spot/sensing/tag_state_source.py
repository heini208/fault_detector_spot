"""Fresh snapshots of tag state published by the tag observation runtime."""

from copy import deepcopy
from threading import RLock
import math
import time

from fault_detector_msgs.msg import TagElementArray

from fault_detector_spot.shared.ros.qos_profiles import TAG_STATE_QOS


DEFAULT_TAG_STATE_STALE_AFTER_SEC = 1.5


class TagStateSource:
    """Cache authoritative base, visible, and reachable tag snapshots."""

    def __init__(
        self,
        node,
        stale_after_sec: float = DEFAULT_TAG_STATE_STALE_AFTER_SEC,
        monotonic_clock=time.monotonic,
    ):
        if node is None:
            raise RuntimeError("TagStateSource requires a ROS node")
        stale_after_sec = float(stale_after_sec)
        if not math.isfinite(stale_after_sec) or stale_after_sec <= 0.0:
            raise ValueError("Tag state stale timeout must be positive")
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")

        self.node = node
        self.stale_after_sec = stale_after_sec
        self._monotonic_clock = monotonic_clock
        self._lock = RLock()

        self._base_tags = {}
        self._visible_tags = {}
        self._reachable_tags = {}
        self._base_received_at = None
        self._visible_received_at = None
        self._reachable_received_at = None

        self._subscriptions = [
            node.create_subscription(
                TagElementArray,
                "fault_detector/state/base_tags",
                self._receive_base_tags,
                TAG_STATE_QOS,
            ),
            node.create_subscription(
                TagElementArray,
                "fault_detector/state/visible_tags",
                self._receive_visible_tags,
                TAG_STATE_QOS,
            ),
            node.create_subscription(
                TagElementArray,
                "fault_detector/state/reachable_tags",
                self._receive_reachable_tags,
                TAG_STATE_QOS,
            ),
        ]

    def base_snapshot(self, now: float = None):
        return self._snapshot(
            "_base_tags",
            "_base_received_at",
            now,
        )

    def visible_snapshot(self, now: float = None):
        return self._snapshot(
            "_visible_tags",
            "_visible_received_at",
            now,
        )

    def reachable_snapshot(self, now: float = None):
        return self._snapshot(
            "_reachable_tags",
            "_reachable_received_at",
            now,
        )

    def reachable_tag(self, tag_id: int, now: float = None):
        return self.reachable_snapshot(now).get(int(tag_id))

    def destroy(self) -> None:
        subscriptions = tuple(self._subscriptions)
        self._subscriptions.clear()
        for subscription in subscriptions:
            self.node.destroy_subscription(subscription)

    def _snapshot(self, values_attr, receipt_attr, now):
        current = (
            self._monotonic_clock()
            if now is None
            else float(now)
        )
        with self._lock:
            received_at = getattr(self, receipt_attr)
            values = getattr(self, values_attr)
            if (
                received_at is None
                or current - received_at > self.stale_after_sec
            ):
                return {}
            return deepcopy(values)

    def _receive_base_tags(self, message: TagElementArray) -> None:
        self._store("_base_tags", "_base_received_at", message)

    def _receive_visible_tags(self, message: TagElementArray) -> None:
        self._store("_visible_tags", "_visible_received_at", message)

    def _receive_reachable_tags(self, message: TagElementArray) -> None:
        self._store(
            "_reachable_tags",
            "_reachable_received_at",
            message,
        )

    def _store(self, values_attr, receipt_attr, message) -> None:
        snapshot = {
            int(tag.id): deepcopy(tag)
            for tag in message.elements
        }
        with self._lock:
            setattr(self, values_attr, snapshot)
            setattr(
                self,
                receipt_attr,
                self._monotonic_clock(),
            )


__all__ = [
    "DEFAULT_TAG_STATE_STALE_AFTER_SEC",
    "TagStateSource",
]
