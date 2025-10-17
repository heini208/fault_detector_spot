#!/usr/bin/env python3
import numpy as np

import tf_transformations as tf
from builtin_interfaces.msg import Time
from fault_detector_spot.behaviour_tree.commands.simple_command import SimpleCommand
from geometry_msgs.msg import PoseStamped, Quaternion
from synchros2.tf_listener_wrapper import TFListenerWrapper


class MoveBaseRelativeCommand(SimpleCommand):
    """
    Compute a PoseStamped goal relative to the robot’s current body frame.
    Offset can be defined in any frame; it is rotated into the body axes.
    Yaw from offset is used as final heading.
    """

    def __init__(
            self,
            command_id: str,
            stamp: Time,
            offset: PoseStamped = None,  # offset in some frame
            target_frame: str = "body",
    ):
        self.command_id = command_id
        self.stamp = stamp
        self.offset = offset or PoseStamped()
        self.target_frame = target_frame
        if not self.offset.header.frame_id:
            self.offset.header.frame_id = self.target_frame
        self.offset.pose.orientation = self.offset.pose.orientation or Quaternion(w=1.0)

        self.goal_pose: PoseStamped = PoseStamped()

    def compute_goal_pose(self, transformer: TFListenerWrapper) -> PoseStamped:
        """
        Build goal_pose relative to the robot body frame:
        1. Rotate offset position into body axes.
        2. Use offset yaw as heading.
        """
        offset_vec = np.array([
            self.offset.pose.position.x,
            self.offset.pose.position.y,
            0.0
        ])

        # Step 1: rotate offset into target/body frame
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
            R = tf.quaternion_matrix(q_rot)[:3, :3]
        else:
            R = np.eye(3)

        offset_in_body = R.dot(offset_vec)

        # Step 2: compute yaw-only heading
        q_offset = [
            self.offset.pose.orientation.x,
            self.offset.pose.orientation.y,
            self.offset.pose.orientation.z,
            self.offset.pose.orientation.w
        ]
        _, _, yaw = tf.euler_from_quaternion(q_offset)
        q_yaw_only = tf.quaternion_from_euler(0.0, 0.0, yaw)

        # Step 3: build PoseStamped
        self.goal_pose.header.frame_id = self.target_frame
        self.goal_pose.header.stamp = self.stamp
        self.goal_pose.pose.position.x = offset_in_body[0]
        self.goal_pose.pose.position.y = offset_in_body[1]
        self.goal_pose.pose.position.z = offset_in_body[2]
        self.goal_pose.pose.orientation = Quaternion(
            x=q_yaw_only[0],
            y=q_yaw_only[1],
            z=q_yaw_only[2],
            w=q_yaw_only[3]
        )

        return self.goal_pose