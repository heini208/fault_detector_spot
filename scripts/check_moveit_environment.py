#!/usr/bin/env python3
"""Request one diagnostic MoveIt plan; never send a trajectory to Spot."""

import argparse
import math
import time


def arguments():
    parser = argparse.ArgumentParser(
        description="Plan one explicit hand target in body; never execute robot motion.",
        epilog=(
            "Requires the MoveIt environmental updater to be running. Use only while "
            "application motion is idle and this is the sole planning client. Checked "
            "requests clear the shared octomap; every request sets the shared occupancy "
            "ACM policy. The last applied policy remains on exit. A timed-out server "
            "request may still finish later; no automatic policy restoration is attempted."
        ),
    )
    parser.add_argument("--target", nargs=3, type=float, required=True,
                        metavar=("X", "Y", "Z"), help="Hand position in body, metres")
    parser.add_argument("--quaternion", nargs=4, type=float, default=(0., 0., 0., 1.),
                        metavar=("QX", "QY", "QZ", "QW"),
                        help="Unit hand orientation in body; default identity")
    parser.add_argument("--cartesian", action="store_true", help="Request a straight Cartesian path")
    parser.add_argument("--lidar", action="store_true",
                        help="Require fresh lidar as well as camera observations; match the launch setting")
    parser.add_argument("--ignore-environment-collisions", action="store_true",
                        help="Ignore sensor occupancy, retaining robot and explicit-object rules")
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in args.target + list(args.quaternion)):
        parser.error("Target position and quaternion must be finite")
    norm = math.hypot(*args.quaternion)
    if abs(norm - 1.0) > 1e-3:
        parser.error("Quaternion must have unit length (within 0.001)")
    args.quaternion = [value / norm for value in args.quaternion]
    return args


def main():
    args = arguments()  # --help and invalid inputs never initialize ROS.
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from fault_detector_spot.manipulation.arm_motion_parameters import ArmMotionParameters
    from fault_detector_spot.manipulation.moveit_arm_planner import (
        MoveItArmPlanner, MoveItPlanOutcome,
    )
    from fault_detector_spot.manipulation.moveit_environment_source import MoveItEnvironmentSource

    rclpy.init(args=[])
    node = rclpy.create_node(
        "check_moveit_environment",
        parameter_overrides=[
            rclpy.Parameter("arm.environment.lidar_enabled", value=args.lidar),
        ],
    )
    planner = None
    try:
        config = ArmMotionParameters(node)
        planner = MoveItArmPlanner(
            node,
            velocity_scaling=config.get("motion.moveit_velocity_scaling"),
            acceleration_scaling=config.get("motion.moveit_acceleration_scaling"),
            min_arm_sh1_rad=config.get("motion.arm_sh1_safe_min_rad"),
            environment_collision_policy_enabled=True,
            environment_source=MoveItEnvironmentSource(node, config),
        )
        target = PoseStamped()
        target.header.frame_id = "body"
        for axis, value in zip("xyz", args.target):
            setattr(target.pose.position, axis, value)
        for axis, value in zip("xyzw", args.quaternion):
            setattr(target.pose.orientation, axis, value)
        print("Planning only: shared octomap/ACM may change; no robot motion will be sent.", flush=True)
        start = planner.start_cartesian if args.cartesian else planner.start
        began = time.monotonic()
        update = None
        # Service-unavailable returns have made no requests; retry discovery briefly.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            update = start(target, ignore_environment_collisions=args.ignore_environment_collisions)
            if (update.outcome is not MoveItPlanOutcome.SERVICE_UNAVAILABLE
                    or time.monotonic() - began >= 5.0):
                break
        last_detail = None
        while rclpy.ok() and update is not None and update.outcome is MoveItPlanOutcome.RUNNING:
            if update.detail != last_detail:
                print(update.detail, flush=True)
                last_detail = update.detail
            if time.monotonic() - began >= 45.0:
                planner.cancel()
                print("TIMEOUT: diagnostic exceeded 45 s; outstanding server work may still finish.")
                return 2
            rclpy.spin_once(node, timeout_sec=0.1)
            update = planner.poll()
        if update is None or update.outcome is MoveItPlanOutcome.RUNNING:
            print("Interrupted before a terminal planning result.")
            return 2
        points = len(update.trajectory.points) if update.trajectory is not None else 0
        print(f"{update.outcome.value}: {update.detail}; trajectory points: {points}")
        return 0 if update.outcome is MoveItPlanOutcome.SUCCESS else 1
    except KeyboardInterrupt:
        print("Interrupted; outstanding server work may still finish.")
        return 2
    finally:
        if planner is not None:
            planner.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
        print("No execution requested. Shared collision policy is left as last applied.")


if __name__ == "__main__":
    raise SystemExit(main())
