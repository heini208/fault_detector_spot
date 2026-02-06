#!/usr/bin/env python3
import numpy as np

import tf_transformations as tf
from builtin_interfaces.msg import Time
from fault_detector_spot.behaviour_tree.commands.simple_command import SimpleCommand
from geometry_msgs.msg import PoseStamped, Quaternion
from synchros2.tf_listener_wrapper import TFListenerWrapper


class MoveCommand(SimpleCommand):
    """
    Abstract base class for all move commands.
    Provides storage for goal/offset and helper methods for TF operations.
    """

    def __init__(
            self,
            command_id: str,
            stamp: Time,
            offset: PoseStamped = None,
            target_frame: str = "body",
    ):
        super().__init__(command_id, stamp)
        self.target_frame = target_frame
        self.offset = offset or PoseStamped()

        # Ensure offset has a frame
        if not self.offset.header.frame_id:
            self.offset.header.frame_id = self.target_frame

        # Ensure offset has valid orientation
        if self.offset.pose.orientation.w == 0 and \
                self.offset.pose.orientation.x == 0 and \
                self.offset.pose.orientation.y == 0 and \
                self.offset.pose.orientation.z == 0:
            self.offset.pose.orientation = Quaternion(w=1.0)

    def compute_goal_pose(self, transformer: TFListenerWrapper) -> PoseStamped:
        """
        Abstract method to compute the final goal pose in the target frame.
        """
        raise NotImplementedError("Subclasses must implement compute_goal_pose")

    def _rotate_vector_into_frame(self, vector: np.ndarray, source_frame: str, target_frame: str,
                                  transformer: TFListenerWrapper) -> np.ndarray:
        """
        Helper to rotate a 3D vector from source_frame to target_frame using the transformer.
        """
        if transformer and source_frame != target_frame:
            tf_msg = transformer.lookup_a_tform_b(
                target_frame, source_frame, timeout_sec=2
            )
            q_rot = [
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w
            ]
            R = tf.quaternion_matrix(q_rot)[:3, :3]
            return R.dot(vector)
        return vector

    def _rotate_quaternion_into_frame(self, quat_xyzw: list, source_frame: str, target_frame: str,
                                      transformer: TFListenerWrapper) -> list:
        """
        Helper to rotate a quaternion from source_frame to target_frame.
        """
        if transformer and source_frame != target_frame:
            tf_msg = transformer.lookup_a_tform_b(
                target_frame, source_frame, timeout_sec=2
            )
            q_rot = [
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w
            ]
            return list(tf.quaternion_multiply(q_rot, quat_xyzw))
        return quat_xyzw

    def _rotate_only_yaw_into_frame(self, quat_xyzw: list, source_frame: str, target_frame: str,
                                    transformer: TFListenerWrapper) -> list:
        """
        Rotates a quaternion from source to target and calculates the 2D Yaw
        by projecting the X-axis vector. Robust against high-pitch Gimbal Lock.
        """
        # 1. Calculate the full 3D rotation quaternion
        if not transformer or source_frame == target_frame:
            return list(quat_xyzw)

            # 1) Get yaw-only from the transform by using an identity input
        tf_msg = transformer.lookup_a_tform_b(target_frame, source_frame, timeout_sec=2)
        q_rot = [
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w,
        ]
        R = tf.quaternion_matrix(q_rot)[:3, :3]
        v_forward_source = np.array([0.0, 0.0, -1.0])
        v_forward_target = np.dot(R, v_forward_source)
        yaw_new = np.arctan2(v_forward_target[1], v_forward_target[0])
        rot_quat = tf.quaternion_from_euler(0.0, 0.0, yaw_new)

        final_orientation = tf.quaternion_multiply(rot_quat, quat_xyzw)
        return list(final_orientation)