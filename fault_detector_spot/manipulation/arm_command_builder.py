"""Build Spot arm commands and validate/retime planned joint trajectories.

These helpers do not submit commands or own execution lifecycle state.
"""

import math
from bosdyn.api import arm_command_pb2, robot_command_pb2, synchronized_command_pb2
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_spot_api_msgs.conversions import convert
from geometry_msgs.msg import PoseStamped
from spot_msgs.action import RobotCommand
from spot_msgs.srv import RobotCommand as RobotCommandService
from synchros2.utilities import namespace_with
from fault_detector_spot.manipulation.moveit_arm_planner import ARM_JOINT_NAMES


def _command_goal(command):
    goal = RobotCommand.Goal()
    convert(command, goal.command)
    return goal


def build_arm_stop_request() -> RobotCommandService.Request:
    arm_stop = arm_command_pb2.ArmStopCommand.Request()
    arm_command = arm_command_pb2.ArmCommand.Request(
        arm_stop_command=arm_stop
    )
    synchronized = synchronized_command_pb2.SynchronizedCommand.Request(
        arm_command=arm_command
    )
    command = robot_command_pb2.RobotCommand(
        synchronized_command=synchronized
    )
    request = RobotCommandService.Request()
    convert(command, request.command)
    return request


def build_gripper_goal(open_fraction: float) -> RobotCommand.Goal:
    return _command_goal(
        RobotCommandBuilder.claw_gripper_open_fraction_command(open_fraction)
    )


def build_stow_goal() -> RobotCommand.Goal:
    stow_command = RobotCommandBuilder.arm_stow_command()
    return _command_goal(stow_command)


def build_moveit_joint_goal(
    trajectory,
    minimum_duration_sec=None,
) -> tuple[RobotCommand.Goal, float]:
    names = tuple(trajectory.joint_names)
    if len(names) != len(ARM_JOINT_NAMES) or set(names) != set(
        ARM_JOINT_NAMES
    ):
        raise ValueError(
            "MoveIt trajectory must contain exactly the six Spot arm joints"
        )

    points = tuple(trajectory.points)
    if not points:
        raise ValueError("MoveIt trajectory contains no points")

    joint_indices = [
        names.index(name)
        for name in ARM_JOINT_NAMES
    ]
    joint_positions = []
    times = []
    joint_velocities = []
    use_velocities = None

    for index, point in enumerate(points):
        if len(point.positions) != len(names):
            raise ValueError(
                f"MoveIt trajectory point {index} has "
                f"{len(point.positions)} positions for {len(names)} joints"
            )

        joint_positions.append([
            float(point.positions[joint_index])
            for joint_index in joint_indices
        ])
        times.append(
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) * 1e-9
        )

        velocity_count = len(point.velocities)
        point_has_velocities = velocity_count > 0
        if point_has_velocities and velocity_count != len(names):
            raise ValueError(
                f"MoveIt trajectory point {index} has "
                f"{velocity_count} velocities for {len(names)} joints"
            )
        if use_velocities is None:
            use_velocities = point_has_velocities
        elif use_velocities != point_has_velocities:
            raise ValueError(
                "MoveIt trajectory must provide velocities for every "
                "point or for none"
            )
        if point_has_velocities:
            joint_velocities.append([
                float(point.velocities[joint_index])
                for joint_index in joint_indices
            ])

    if any(not math.isfinite(value) or value < 0.0 for value in times):
        raise ValueError(
            "MoveIt trajectory times must be finite and nonnegative"
        )
    if any(right <= left for left, right in zip(times, times[1:])):
        raise ValueError(
            "MoveIt trajectory times must be strictly increasing"
        )

    moveit_duration_sec = times[-1]
    requested_duration_sec = 0.0
    if minimum_duration_sec is not None:
        requested_duration_sec = float(minimum_duration_sec)
        if (
            not math.isfinite(requested_duration_sec)
            or requested_duration_sec <= 0.0
        ):
            raise ValueError(
                "Requested arm movement duration must be positive and finite"
            )

    target_duration_sec = max(
        moveit_duration_sec,
        requested_duration_sec,
    )
    time_scale = 1.0
    if (
        moveit_duration_sec > 1e-9
        and target_duration_sec > moveit_duration_sec
    ):
        time_scale = target_duration_sec / moveit_duration_sec
        times = [
            value * time_scale
            for value in times
        ]
        if use_velocities:
            joint_velocities = [
                [
                    value / time_scale
                    for value in velocities
                ]
                for velocities in joint_velocities
            ]

    start_delay = max(0.0, 0.25 - times[0])
    times = [value + start_delay for value in times]
    execution_duration_sec = times[-1]

    command = RobotCommandBuilder.arm_joint_move_helper(
        joint_positions,
        times,
        joint_velocities=(
            joint_velocities
            if use_velocities
            else None
        ),
    )
    return _command_goal(command), execution_duration_sec


def build_pose_goal(
    target: PoseStamped,
    duration_sec: float,
    robot_name: str,
) -> RobotCommand.Goal:
    duration = float(duration_sec)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError(
            "Arm movement duration must be positive and finite"
        )

    target_frame = target.header.frame_id.strip()
    if not target_frame:
        raise ValueError("Arm pose target frame must not be empty")

    pose = target.pose
    command = RobotCommandBuilder.arm_pose_command(
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        namespace_with(robot_name, target_frame),
        duration,
    )
    return _command_goal(command)

