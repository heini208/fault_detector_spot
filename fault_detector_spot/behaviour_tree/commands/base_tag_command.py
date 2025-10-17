#!/usr/bin/env python3
import numpy as np

import tf2_geometry_msgs
import tf_transformations as tf
from builtin_interfaces.msg import Time
from fault_detector_spot.behaviour_tree.commands.simple_command import SimpleCommand
from geometry_msgs.msg import PoseStamped, Quaternion
from synchros2.tf_listener_wrapper import TFListenerWrapper


class BaseTagCommand(SimpleCommand):
    """
    Move the robot base relative to a detected tag, with an optional offset in another frame.
    Offset axes are rotated into the target frame axes.
    Orientation is transformed from the offset frame into the target frame (yaw-only).
    """

    def __init__(
            self,
            command_id: str,
            stamp: Time,
            goal_pose: PoseStamped,  # tag pose in body frame
            tag_id: int,
            offset: PoseStamped = None,  # offset in some frame
            target_frame: str = "odom",
    ):
        self.command_id = command_id
        self.stamp = stamp
        self.goal_pose = goal_pose
        self.tag_id = tag_id
        self.target_frame = target_frame
        self.offset = offset or PoseStamped()
        if not self.offset.header.frame_id:
            self.offset.header.frame_id = self.target_frame
        self.offset.pose.orientation = self.offset.pose.orientation or Quaternion(w=1.0)

    def get_offset_pose(self, transformer: TFListenerWrapper) -> PoseStamped:
        """
        Compute the final goal pose in target_frame by:
        1. Transforming tag position into target_frame
        2. Rotating offset position into target_frame axes
        3. Adding rotated offset to tag position
        4. Rotating offset orientation into target frame and extracting yaw
        """
        # Step 1: transform tag into target frame
        tf_to_target_frame = transformer.lookup_a_tform_b(
            self.target_frame, self.goal_pose.header.frame_id, timeout_sec=2
        )
        tag_in_target = tf2_geometry_msgs.do_transform_pose_stamped(
            transform=tf_to_target_frame, pose=self.goal_pose
        )

        # Step 2: rotate offset position into target frame
        offset_in_target = self._rotate_offset_into_target_frame(transformer)

        # Step 3: add offset to tag position
        goal_position = np.array([
            tag_in_target.pose.position.x + offset_in_target[0],
            tag_in_target.pose.position.y + offset_in_target[1],
            0.0  # ignore height for base navigation
        ])

        # Step 4: rotate offset orientation into target frame (ignore tag position)
        q_offset = [
            self.offset.pose.orientation.x,
            self.offset.pose.orientation.y,
            self.offset.pose.orientation.z,
            self.offset.pose.orientation.w
        ]

        if transformer and self.offset.header.frame_id != self.target_frame:
            tf_msg = transformer.lookup_a_tform_b(
                self.target_frame, self.offset.header.frame_id, timeout_sec=2
            )
            q_rot = [
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w
            ]
            q_offset_in_target = tf.quaternion_multiply(q_rot, q_offset)
        else:
            q_offset_in_target = q_offset

        # Step 5: extract yaw-only
        _, _, yaw = tf.euler_from_quaternion(q_offset_in_target)
        q_yaw_only = tf.quaternion_from_euler(0.0, 0.0, yaw)

        # Step 6: construct PoseStamped
        result = PoseStamped()
        result.header.frame_id = self.target_frame
        result.header.stamp = self.stamp
        result.pose.position.x = goal_position[0]
        result.pose.position.y = goal_position[1]
        result.pose.position.z = goal_position[2]
        result.pose.orientation = Quaternion(
            x=q_yaw_only[0],
            y=q_yaw_only[1],
            z=q_yaw_only[2],
            w=q_yaw_only[3]
        )

        return result

    def _rotate_offset_into_target_frame(self, transformer: TFListenerWrapper) -> np.ndarray:
        """
        Rotate the offset vector from its frame into the target frame axes.
        Returns a numpy 3-vector of the rotated offset.
        """
        offset_vec = np.array([
            self.offset.pose.position.x,
            self.offset.pose.position.y,
            self.offset.pose.position.z
        ])

        if transformer and self.offset.header.frame_id != self.target_frame:
            # lookup rotation from offset frame -> target frame
            tf_msg = transformer.lookup_a_tform_b(
                self.target_frame, self.offset.header.frame_id, timeout_sec=2
            )
            q_rot = [
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w
            ]
            R = tf.quaternion_matrix(q_rot)[:3, :3]
        else:
            R = np.eye(3)

        offset_rotated = R.dot(offset_vec)
        return offset_rotated