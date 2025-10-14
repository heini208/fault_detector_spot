from typing import Optional

import py_trees
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# QoS Profiles
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

VOLATILE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

class LastLocalizationPose(py_trees.behaviour.Behaviour):
    """
    Listens to /pose and the localization topic (PoseWithCovarianceStamped) and updates
    the blackboard with the latest pose estimation.
    Prioritizes the localization topic if it is recent (slightly older is still preferred).
    """

    def __init__(self, name: str = "LastLocalizationPose", localization_leeway_sec: float = 0.1,
                 localization_topic: str = "/localization_pose"):
        super().__init__(name)
        self.node: Optional[Node] = None
        self.blackboard = self.attach_blackboard_client()

        self.localization_topic = localization_topic
        self.localization_leeway_sec = localization_leeway_sec
        self._last_localization_pose: Optional[PoseWithCovarianceStamped] = None
        self._last_slam_pose: Optional[PoseWithCovarianceStamped] = None

        self._sub_amcl: Optional[rclpy.subscription.Subscription] = None
        self._sub_slam: Optional[rclpy.subscription.Subscription] = None

    def setup(self, **kwargs):
        """
        Initialize ROS subscriptions with proper QoS.
        """
        self.node = kwargs['node']

        # AMCL is typically TRANSIENT_LOCAL, RELIABLE
        self._sub_amcl = self.node.create_subscription(
            PoseWithCovarianceStamped,
            self.localization_topic,
            self._amcl_callback,
            LATCHED_QOS
        )

        # SLAM pose is usually VOLATILE / BEST_EFFORT
        self._sub_slam = self.node.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self._slam_callback,
            VOLATILE_QOS
        )

        self.logger.info("LastLocalizationPose subscriptions.")
        self.blackboard.register_key("last_pose_estimation", access=py_trees.common.Access.WRITE)

        # Initialize blackboard variable with zero pose
        zero_pose = PoseWithCovarianceStamped()
        zero_pose.header.stamp = rclpy.time.Time().to_msg()
        zero_pose.header.frame_id = "map"
        zero_pose.pose.pose.position.x = 0.0
        zero_pose.pose.pose.position.y = 0.0
        zero_pose.pose.pose.position.z = 0.0
        zero_pose.pose.pose.orientation.x = 0.0
        zero_pose.pose.pose.orientation.y = 0.0
        zero_pose.pose.pose.orientation.z = 0.0
        zero_pose.pose.pose.orientation.w = 1.0  # identity quaternion
        self.blackboard.last_pose_estimation = zero_pose

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        self._last_localization_pose = msg

    def _slam_callback(self, msg: PoseWithCovarianceStamped):
        self._last_slam_pose = msg

    def update(self) -> py_trees.common.Status:
        """
        Determine which pose to use and write it to the blackboard.
        """
        if self._last_localization_pose and self._last_slam_pose:
            # Compare timestamps
            localization_time = self._last_localization_pose.header.stamp.sec + self._last_localization_pose.header.stamp.nanosec * 1e-9
            slam_time = self._last_slam_pose.header.stamp.sec + self._last_slam_pose.header.stamp.nanosec * 1e-9

            # Prioritize localization if it's newer or slightly older within leeway
            if localization_time >= slam_time - self.localization_leeway_sec:
                selected_pose = self._last_localization_pose
                source = self.localization_topic
            else:
                selected_pose = self._last_slam_pose
                source = "/pose"
        elif self._last_localization_pose:
            selected_pose = self._last_localization_pose
            source = self.localization_topic
        elif self._last_slam_pose:
            selected_pose = self._last_slam_pose
            source = "/pose"
        else:
            self.feedback_message = "No pose received yet."
            return py_trees.common.Status.SUCCESS
        # Write to blackboard
        self.blackboard.last_pose_estimation = selected_pose
        self.feedback_message = f"Using pose from {source}"
        return py_trees.common.Status.SUCCESS