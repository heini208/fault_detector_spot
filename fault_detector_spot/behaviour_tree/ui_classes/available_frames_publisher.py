from enum import Enum

import rclpy
from fault_detector_msgs.msg import StringArray
from fault_detector_msgs.msg import TagElementArray
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from rclpy.node import Node


class FrameNames(str, Enum):
    GRAV_BODY_FRAME = "flat_body"
    MAP_FRAME = "map"
    ODOMETRY_FRAME = "odom"
    VISION_FRAME = "vision"
    BODY_FRAME = "body"


class AvailableFramesPublisher(Node):
    def __init__(self):
        super().__init__('available_frames_publisher')
        self.static_frames = [f.value for f in FrameNames]
        self.visible_tag_ids = set()
        self.last_published_frames = None

        self.tag_sub = self.create_subscription(
            TagElementArray,
            'fault_detector/state/visible_tags',
            self.visible_tags_callback,
            10
        )

        self.publisher = self.create_publisher(
            StringArray,
            'fault_detector/state/available_frames',
            LATCHED_QOS
        )

        self.publish_available_frames()

    def visible_tags_callback(self, msg: TagElementArray):
        tag_ids = {tag.id for tag in msg.elements}
        self.visible_tag_ids = tag_ids
        self.publish_available_frames()

    def publish_available_frames(self):
        tag_frames = [f'Tag_{i}' for i in sorted(self.visible_tag_ids)]
        frames = list(self.static_frames) + tag_frames

        # Only publish if contents changed
        if self.last_published_frames != frames:
            msg = StringArray()
            msg.names = frames
            self.publisher.publish(msg)
            self.last_published_frames = frames[:]


def main(args=None):
    rclpy.init(args=args)
    node = AvailableFramesPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()