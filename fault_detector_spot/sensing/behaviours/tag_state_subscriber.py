"""Copy externally produced tag-state snapshots onto the blackboard."""

from copy import deepcopy
from threading import RLock
import time

import py_trees
from fault_detector_msgs.msg import TagElementArray

from fault_detector_spot.shared.ros.qos_profiles import TAG_STATE_QOS


class TagStateSubscriber(py_trees.behaviour.Behaviour):
    """Temporarily expose authoritative tag topics to legacy BT consumers."""

    def __init__(
        self,
        name: str = "TagStateSubscriber",
        state_timeout_sec: float = 1.5,
    ):
        super().__init__(name)
        if state_timeout_sec <= 0.0:
            raise ValueError("state_timeout_sec must be positive")
        self.state_timeout_sec = float(state_timeout_sec)
        self.node = None
        self._lock = RLock()
        self._base_tags = {}
        self._visible_tags = {}
        self._reachable_tags = {}
        self._base_receipt_time = None
        self._visible_receipt_time = None
        self._reachable_receipt_time = None
        self._base_subscription = None
        self._visible_subscription = None
        self._reachable_subscription = None
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError(f"{self.name}: no ROS node provided")
        self.blackboard.register_key(
            "base_tag_observations",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            "visible_tags",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            "reachable_tags",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.base_tag_observations = {}
        self.blackboard.visible_tags = {}
        self.blackboard.reachable_tags = {}
        self._base_subscription = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/base_tags",
            self._receive_base_tags,
            TAG_STATE_QOS,
        )
        self._visible_subscription = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/visible_tags",
            self._receive_visible_tags,
            TAG_STATE_QOS,
        )
        self._reachable_subscription = self.node.create_subscription(
            TagElementArray,
            "fault_detector/state/reachable_tags",
            self._receive_reachable_tags,
            TAG_STATE_QOS,
        )
        return True

    def update(self):
        now = time.monotonic()
        with self._lock:
            base_tags = self._fresh_snapshot(
                self._base_tags,
                self._base_receipt_time,
                now,
            )
            visible_tags = self._fresh_snapshot(
                self._visible_tags,
                self._visible_receipt_time,
                now,
            )
            reachable_tags = self._fresh_snapshot(
                self._reachable_tags,
                self._reachable_receipt_time,
                now,
            )
        self.blackboard.base_tag_observations = base_tags
        self.blackboard.visible_tags = visible_tags
        self.blackboard.reachable_tags = reachable_tags
        self.feedback_message = (
            f"Base tags: {sorted(base_tags)}; "
            f"visible tags: {sorted(visible_tags)}; "
            f"reachable tags: {sorted(reachable_tags)}"
        )
        return py_trees.common.Status.SUCCESS

    def shutdown(self):
        if self.node is None:
            return
        subscriptions = (
            "_base_subscription",
            "_visible_subscription",
            "_reachable_subscription",
        )
        for attribute in subscriptions:
            subscription = getattr(self, attribute)
            if subscription is not None:
                self.node.destroy_subscription(subscription)
                setattr(self, attribute, None)

    def _receive_base_tags(self, message: TagElementArray) -> None:
        with self._lock:
            self._base_tags = self._message_snapshot(message)
            self._base_receipt_time = time.monotonic()

    def _receive_visible_tags(self, message: TagElementArray) -> None:
        with self._lock:
            self._visible_tags = self._message_snapshot(message)
            self._visible_receipt_time = time.monotonic()

    def _receive_reachable_tags(self, message: TagElementArray) -> None:
        with self._lock:
            self._reachable_tags = self._message_snapshot(message)
            self._reachable_receipt_time = time.monotonic()

    def _fresh_snapshot(self, values, receipt_time, now):
        if (
            receipt_time is None
            or now - receipt_time > self.state_timeout_sec
        ):
            return {}
        return deepcopy(values)

    @staticmethod
    def _message_snapshot(message: TagElementArray):
        return {
            int(tag.id): deepcopy(tag)
            for tag in message.elements
        }
