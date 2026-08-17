#!/usr/bin/env python3
import math

import numpy as np
from builtin_interfaces.msg import Time
from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)
from fault_detector_spot.inspection.geometry.rotation import (
    multiply_quaternions,
    quaternion_from_euler,
    quaternion_from_rotation,
    rotation_from_quaternion,
)
from fault_detector_spot.inspection.model.models import QuaternionData
from geometry_msgs.msg import PoseStamped, Quaternion
from synchros2.tf_listener_wrapper import TFListenerWrapper


class MoveCommand(ExecutionCommand):
    """Base class for movement commands with shared TF rotation helpers."""

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

        if not self.offset.header.frame_id:
            self.offset.header.frame_id = self.target_frame

        orientation = self.offset.pose.orientation
        if (
            orientation.w == 0
            and orientation.x == 0
            and orientation.y == 0
            and orientation.z == 0
        ):
            self.offset.pose.orientation = Quaternion(w=1.0)

    def compute_goal_pose(
        self,
        transformer: TFListenerWrapper,
    ) -> PoseStamped:
        raise NotImplementedError(
            "Subclasses must implement compute_goal_pose"
        )

    def _rotate_vector_into_frame(
        self,
        vector: np.ndarray,
        source_frame: str,
        target_frame: str,
        transformer: TFListenerWrapper,
    ) -> np.ndarray:
        values = np.asarray(vector, dtype=float)
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise ValueError("Movement vector must contain three finite values")
        if source_frame == target_frame:
            return values.copy()
        transform_rotation = self._frame_rotation(
            source_frame,
            target_frame,
            transformer,
        )
        return transform_rotation.apply(values)

    def _rotate_quaternion_into_frame(
        self,
        quat_xyzw: list,
        source_frame: str,
        target_frame: str,
        transformer: TFListenerWrapper,
    ) -> list:
        quaternion = self._quaternion_data(quat_xyzw)
        if source_frame == target_frame:
            result = quaternion_from_rotation(
                rotation_from_quaternion(quaternion)
            )
        else:
            transform = quaternion_from_rotation(
                self._frame_rotation(
                    source_frame,
                    target_frame,
                    transformer,
                )
            )
            result = multiply_quaternions(transform, quaternion)
        return [result.x, result.y, result.z, result.w]

    def _rotate_only_yaw_into_frame(
        self,
        quat_xyzw: list,
        source_frame: str,
        target_frame: str,
        transformer: TFListenerWrapper,
    ) -> list:
        quaternion = self._quaternion_data(quat_xyzw)
        if source_frame == target_frame:
            result = quaternion_from_rotation(
                rotation_from_quaternion(quaternion)
            )
            return [result.x, result.y, result.z, result.w]

        transform_rotation = self._frame_rotation(
            source_frame,
            target_frame,
            transformer,
        )
        tag_normal_target = transform_rotation.apply(
            np.array([0.0, 0.0, -1.0], dtype=float)
        )
        planar_norm = math.hypot(
            float(tag_normal_target[0]),
            float(tag_normal_target[1]),
        )
        if planar_norm <= 1e-12:
            raise ValueError(
                "Tag normal has no horizontal heading in the target frame"
            )
        yaw = math.atan2(
            float(tag_normal_target[1]),
            float(tag_normal_target[0]),
        )
        yaw_rotation = quaternion_from_euler("z", yaw)
        result = multiply_quaternions(yaw_rotation, quaternion)
        return [result.x, result.y, result.z, result.w]

    def _rotate_vector_into_frame_yaw_only(
        self,
        vector: np.ndarray,
        source_frame: str,
        target_frame: str,
        transformer: TFListenerWrapper,
    ) -> np.ndarray:
        values = np.asarray(vector, dtype=float)
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise ValueError("Movement vector must contain three finite values")
        if source_frame == target_frame:
            return values.copy()

        tag_yaw = self._rotate_only_yaw_into_frame(
            [0.0, 0.0, 0.0, 1.0],
            source_frame,
            target_frame,
            transformer,
        )
        yaw_rotation = rotation_from_quaternion(
            self._quaternion_data(tag_yaw)
        )
        planar = yaw_rotation.apply(
            np.array([values[0], values[1], 0.0], dtype=float)
        )
        return np.array(
            [planar[0], planar[1], values[2]],
            dtype=float,
        )

    @staticmethod
    def _quaternion_data(values) -> QuaternionData:
        array = np.asarray(values, dtype=float)
        if array.shape != (4,) or not np.all(np.isfinite(array)):
            raise ValueError(
                "Movement quaternion must contain four finite values"
            )
        quaternion = QuaternionData(
            x=float(array[0]),
            y=float(array[1]),
            z=float(array[2]),
            w=float(array[3]),
        )
        quaternion.validate()
        return quaternion

    @staticmethod
    def _ros_quaternion_data(quaternion) -> QuaternionData:
        return MoveCommand._quaternion_data(
            [
                quaternion.x,
                quaternion.y,
                quaternion.z,
                quaternion.w,
            ]
        )

    @staticmethod
    def _frame_rotation(
        source_frame: str,
        target_frame: str,
        transformer: TFListenerWrapper,
    ):
        if not source_frame.strip() or not target_frame.strip():
            raise ValueError("Movement frame names must not be empty")
        if transformer is None:
            raise RuntimeError(
                "Movement between different frames requires TF"
            )
        transform = transformer.lookup_a_tform_b(
            target_frame,
            source_frame,
            timeout_sec=0.0,
        )
        quaternion = MoveCommand._ros_quaternion_data(
            transform.transform.rotation
        )
        return rotation_from_quaternion(quaternion)
