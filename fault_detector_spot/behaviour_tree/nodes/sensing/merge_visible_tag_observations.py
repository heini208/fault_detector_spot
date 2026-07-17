"""Select visible tag observations with strict base-camera priority."""

from copy import deepcopy
from typing import Dict, Optional

import py_trees
from fault_detector_msgs.msg import TagElement
from rclpy.node import Node


class MergeVisibleTagObservations(
    py_trees.behaviour.Behaviour
):
    """Select base observations first and hand observations as fallback."""

    def __init__(
        self,
        name: str = "MergeVisibleTagObservations",
    ):
        """Create a strict base-first observation selector."""
        super().__init__(name)
        self.node: Optional[Node] = None
        self.blackboard = self.attach_blackboard_client()
        self._previous_sources: Dict[int, str] = {}

    def setup(self, **kwargs):
        """Register blackboard inputs and outputs."""
        self.node = kwargs.get("node")

        self.blackboard.register_key(
            key="base_tag_observations",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key="hand_tag_observations",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key="visible_tags",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="visible_tag_sources",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="visible_tag_source_changed_ids",
            access=py_trees.common.Access.WRITE,
        )

        self.blackboard.visible_tags = {}
        self.blackboard.visible_tag_sources = {}
        self.blackboard.visible_tag_source_changed_ids = set()
        self._previous_sources = {}

    def update(self) -> py_trees.common.Status:
        """Select one authoritative observation for every tag ID."""
        base_tags: Dict[int, TagElement] = getattr(
            self.blackboard,
            "base_tag_observations",
            {},
        ) or {}
        hand_tags: Dict[int, TagElement] = getattr(
            self.blackboard,
            "hand_tag_observations",
            {},
        ) or {}

        selected = {}
        sources = {}

        for tag_id in set(base_tags) | set(hand_tags):
            if tag_id in base_tags:
                selected[tag_id] = deepcopy(
                    base_tags[tag_id]
                )
                sources[tag_id] = "base"
            else:
                selected[tag_id] = deepcopy(
                    hand_tags[tag_id]
                )
                sources[tag_id] = "hand"

        changed_ids = {
            tag_id
            for tag_id in (
                set(self._previous_sources) | set(sources)
            )
            if self._previous_sources.get(tag_id)
            != sources.get(tag_id)
        }

        self._log_source_changes(changed_ids, sources)
        self._previous_sources = deepcopy(sources)

        self.blackboard.visible_tags = selected
        self.blackboard.visible_tag_sources = sources
        self.blackboard.visible_tag_source_changed_ids = (
            changed_ids
        )
        self.feedback_message = (
            f"Selected tags: {sources}; "
            f"source changes: {sorted(changed_ids)}"
        )

        return py_trees.common.Status.SUCCESS

    def _log_source_changes(
        self,
        changed_ids,
        sources: Dict[int, str],
    ) -> None:
        """Log source transitions when a ROS node is available."""
        if self.node is None:
            return

        for tag_id in sorted(changed_ids):
            previous = self._previous_sources.get(
                tag_id,
                "none",
            )
            current = sources.get(tag_id, "none")
            self.node.get_logger().info(
                f"Tag {tag_id} source changed: "
                f"{previous} -> {current}"
            )