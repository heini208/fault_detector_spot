"""Build and run the main robot behavior tree."""

import sys
import time
from threading import Lock
from typing import Callable

import py_trees
import py_trees_ros
import rclpy
from py_trees.behaviours import CheckBlackboardVariableValue
from py_trees.common import ComparisonExpression
from py_trees.decorators import EternalGuard, StatusToBlackboard
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from fault_detector_spot.shared.ros.shutdown_coordinator import (
    ShutdownCoordinator, shutdown_signals,
)
from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore

from fault_detector_spot.shared.persistence.basic_settings import startup_settings

from fault_detector_spot.application.behaviour_tree.behaviours.command_manager import (
    CommandManager,
)
from fault_detector_spot.application.behaviour_tree.behaviours.helper_initializer import (
    HelperInitializer,
)
from fault_detector_spot.application.behaviour_tree.behaviours.new_command_guard import (
    NewCommandGuard,
)
from fault_detector_spot.application.behaviour_tree.behaviours.wait_for_duration import (
    WaitForDuration,
)
from fault_detector_spot.inspection.behaviours.publish_live_inspection_object import (
    PublishLiveInspectionObject,
)
from fault_detector_spot.inspection.behaviours.resolve_live_inspection_object import (
    ResolveLiveInspectionObject,
)
from fault_detector_spot.manipulation.behaviours.close_gripper_action import (
    CloseGripperAction,
)
from fault_detector_spot.manipulation.behaviours.measure_sensor_tip_distance_behaviour import (
    MeasureSensorTipDistanceBehaviour,
)
from fault_detector_spot.manipulation.behaviours.move_close_to_surface_behaviour import (
    MoveCloseToSurfaceBehaviour,
)
from fault_detector_spot.manipulation.behaviours.orient_to_surface_behaviour import (
    OrientToSurfaceBehaviour,
)
from fault_detector_spot.manipulation.behaviours.orient_sensor_tip_to_surface_behaviour import (
    OrientSensorTipToSurfaceBehaviour,
)
from fault_detector_spot.manipulation.behaviours.orient_to_tag_behaviour import (
    OrientToTagBehaviour,
)
from fault_detector_spot.manipulation.behaviours.arm_goal_behaviour import (
    ArmGoalBehaviour,
)
from fault_detector_spot.manipulation.behaviours.ready_arm_behaviour import (
    ReadyArmBehaviour,
)
from fault_detector_spot.manipulation.behaviours.stow_arm_behaviour import (
    StowArmBehaviour,
)
from fault_detector_spot.manipulation.behaviours.toggle_gripper_action import (
    ToggleGripperAction,
)
from fault_detector_spot.mapping.behaviours.enable_localization import (
    EnableLocalization,
)
from fault_detector_spot.mapping.behaviours.enable_slam import EnableSLAM
from fault_detector_spot.mapping.behaviours.stop_mapping import StopMapping
from fault_detector_spot.mapping.behaviours.swap_map import SwapMap
from fault_detector_spot.navigation.behaviours.landmark_relocalizer import (
    LandmarkRelocalizer,
)
from fault_detector_spot.navigation.behaviours.base_goal_behaviour import (
    BaseGoalBehaviour,
)
from fault_detector_spot.navigation.behaviours.change_body_height_behaviour import (
    ChangeBodyHeightBehaviour,
)
from fault_detector_spot.navigation.behaviours.stand_up_behaviour import (
    StandUpBehaviour,
)
from fault_detector_spot.navigation.behaviours.sit_down_behaviour import (
    SitDownBehaviour,
)
from fault_detector_spot.navigation.behaviours.navigate_to_goal_pose import (
    NavigateToGoalPose,
)
from fault_detector_spot.navigation.behaviours.set_waypoint_as_goal import (
    SetWaypointAsGoal,
)
from fault_detector_spot.application.behaviour_tree.behaviours.buffer_and_status_publisher import (
    BufferStatusPublisher,
)
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)
from fault_detector_spot.sensing.behaviours.tag_state_subscriber import (
    TagStateSubscriber,
)
from fault_detector_spot.sensing.behaviours.visible_tag_to_map import (
    VisibleTagToMap,
)
from fault_detector_spot.inspection.behaviours.execute_probe_point import ExecuteProbePoint
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.navigation.behaviours.last_localization_pose import (
    LastLocalizationPose,
)


helper_initializer: HelperInitializer = None


def read_parameter(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


class _TreeDispatch:
    """Fence queued timer callbacks and retain already-running tick ownership."""

    def __init__(self, tick):
        self._tick = tick
        self._lock = Lock()
        self._paused = False
        self._active = 0

    def tick(self):
        with self._lock:
            if self._paused:
                return
            self._active += 1
        try:
            return self._tick()
        finally:
            with self._lock:
                self._active -= 1

    def pause(self):
        with self._lock:
            self._paused = True

    def status(self):
        with self._lock:
            return self._active == 0, f"{self._active} behavior-tree ticks awaiting completion"


def start_tree_ticking(tree, period_ms=50.0):
    tree._shutdown_dispatch = _TreeDispatch(tree.tick)
    tree.timer = tree.node.create_timer(
        period_ms / 1000.0,
        tree._shutdown_dispatch.tick,
        clock=Clock(clock_type=ClockType.STEADY_TIME),
    )
    return tree.timer


def create_root(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    return create_behavior_tree(node)


def create_behavior_tree(node: rclpy.node.Node):
    from fault_detector_spot.application.behaviour_tree.command_status_ledger import CommandStatusLedger

    status_ledger = CommandStatusLedger()
    root = py_trees.composites.Parallel(
        "FaultDetectorSpot",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    root.add_children([
        build_sensing_tree(node, status_ledger=status_ledger),
        build_buffered_command_tree(node, status_ledger=status_ledger),
        build_publisher_tree(node, status_ledger=status_ledger),
    ])
    return root


def build_sensing_tree(node: rclpy.node.Node, *, status_ledger=None) -> py_trees.behaviour.Behaviour:
    tag_state_timeout_sec = float(
        read_parameter(
            node,
            "tag_sensing.state_timeout_sec",
            1.5,
        )
    )
    probe_max_age_sec = float(
        read_parameter(
            node,
            "tag_sensing.probe_max_age_sec",
            1.5,
        )
    )
    active_object_id = read_parameter(
        node,
        "inspection.active_object_id",
        "",
    )
    active_routine_id = read_parameter(
        node,
        "inspection.active_routine_id",
        "",
    )
    inspection_object_root = read_parameter(
        node,
        "inspection.object_root",
        "",
    )
    inspection_execution_frame = read_parameter(
        node,
        "inspection.execution_frame",
        "odom",
    )

    sensing_seq = py_trees.composites.Parallel(
        "Sensing",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    helpers = get_helper_container(node)
    cmd_sub = CommandSubscriber(
        name="UI Command Listener", defer_subscription=True,
        status_ledger=status_ledger,
        robot_command_resources=helpers.robot_command_resources,
    )
    pose_sub = LastLocalizationPose(name="LastLocalizationPose")
    tag_scan_sequence = py_trees.composites.Sequence(
        name="ScanForTags",
        memory=True,
    )
    tag_state_subscriber = TagStateSubscriber(
        name="TagStateSubscriber",
        state_timeout_sec=tag_state_timeout_sec,
    )
    helpers = get_helper_container(node)
    tf_buffer = helpers.robot_command_resources.get_tf_listener(node).buffer
    live_object_resolver = ResolveLiveInspectionObject(
        object_id=active_object_id,
        routine_id=active_routine_id,
        tf_buffer=tf_buffer,
        execution_frame=inspection_execution_frame,
        maximum_age_sec=probe_max_age_sec,
        object_root=inspection_object_root or None,
        name="ResolveLiveInspectionObject",
    )
    live_object_publisher = PublishLiveInspectionObject(
        name="PublishLiveInspectionObject",
    )
    world_frame_transformer = VisibleTagToMap(
        rtabmap_runtime=helpers.rtabmap_runtime,
        tf_buffer=tf_buffer,
        name="VisibleTagToMap",
    )
    tag_scan_sequence.add_children([
        tag_state_subscriber,
        live_object_resolver,
        live_object_publisher,
        world_frame_transformer,
    ])
    sensing_seq.add_children([
        tag_scan_sequence,
        cmd_sub,
        pose_sub,
    ])
    return sensing_seq


def build_buffered_command_tree(
    node: rclpy.node.Node, *, status_ledger=None,
) -> py_trees.behaviour.Behaviour:
    buffered_command_tree = py_trees.composites.Parallel(
        "BufferedCommandTree",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    command_tree = build_repeat_guarded_cancelable_command_tree(node, status_ledger=status_ledger)
    buffer = CommandManager(name="CommandManager")
    buffered_command_tree.add_children([
        buffer,
        command_tree,
    ])
    return buffered_command_tree


def get_helper_container(node: rclpy.node.Node):
    global helper_initializer
    if helper_initializer is None:
        helper_initializer = HelperInitializer(
            name="InitHelpers",
            node=node,
        )
        helper_initializer.setup(timeout=10)
        helper_initializer.tick_once()
    return helper_initializer


def close_helper_container():
    """Close and forget process-wide helper resources."""
    global helper_initializer
    if helper_initializer is None:
        return
    helper_initializer.close()
    helper_initializer = None


def build_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    command_selector = py_trees.composites.Selector(
        name="CommandSelector",
        memory=True,
    )
    rtabmap_runtime = get_helper_container(node).rtabmap_runtime
    helper = get_helper_container(node)
    robot_command_resources = helper.robot_command_resources
    tag_state_source = helper.tag_state_source
    from fault_detector_spot.manipulation.behaviours.recover_arm_behaviour import RecoverArmBehaviour
    specs = [
        (CommandID.RECOVER_ARM, lambda n: RecoverArmBehaviour(
            name="RecoverArm", robot_command_resources=robot_command_resources,
        )),
        (CommandID.EXECUTE_PROBE_POINT, lambda n: ExecuteProbePoint(
            name="ExecuteProbePoint", tag_state_source=tag_state_source,
            robot_command_resources=robot_command_resources,
        )),
        (
            CommandID.STOW_ARM,
            lambda n: StowArmBehaviour(
                name="StowArmBehaviour",
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.READY_ARM,
            lambda n: ReadyArmBehaviour(
                name="ReadyArmBehaviour",
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.READY_SAFE_APPROACH,
            lambda n: ReadyArmBehaviour(
                name="ReadySafeApproachBehaviour",
                robot_command_resources=robot_command_resources,
                safe_approach=True,
            ),
        ),
        (
            CommandID.TOGGLE_GRIPPER,
            lambda n: ToggleGripperAction(
                name="ToggleGripperAction",
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.MOVE_ARM_TO_TAG,
            lambda n: ArmGoalBehaviour(
                name="ArmToTagBehaviour",
                tag_state_source=tag_state_source,
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.MOVE_SAFE_APPROACH,
            lambda n: ArmGoalBehaviour(
                name="MoveSafeApproachBehaviour",
                tag_state_source=tag_state_source,
                robot_command_resources=robot_command_resources,
                safe_approach=True,
            ),
        ),
        (
            CommandID.ADJUST_SAFE_APPROACH,
            lambda n: ArmGoalBehaviour(
                name="AdjustSafeApproachBehaviour",
                tag_state_source=tag_state_source,
                robot_command_resources=robot_command_resources,
                safe_approach=True,
            ),
        ),
        (
            CommandID.MOVE_BASE_TO_TAG,
            lambda n: BaseGoalBehaviour(
                name="BaseToTagBehaviour",
                tag_state_source=tag_state_source,
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.MOVE_BASE_TO_POSE,
            lambda n: BaseGoalBehaviour(
                name="BasePoseBehaviour",
                tag_state_source=tag_state_source,
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.MOVE_BASE_RELATIVE,
            lambda n: BaseGoalBehaviour(
                name="BaseRelativeBehaviour",
                tag_state_source=tag_state_source,
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.MOVE_ARM_TO_POSE,
            lambda n: ArmGoalBehaviour(
                name="ArmPoseBehaviour",
                tag_state_source=tag_state_source,
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.MOVE_ARM_RELATIVE,
            lambda n: ArmGoalBehaviour(
                name="ArmRelativeBehaviour",
                tag_state_source=tag_state_source,
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.MOVE_CLOSE_TO_SURFACE,
            lambda n: MoveCloseToSurfaceBehaviour(
                name="MoveCloseToSurfaceBehaviour",
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.MEASURE_SENSOR_TIP_DISTANCE,
            lambda n: MeasureSensorTipDistanceBehaviour(
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.ORIENT_SENSOR_TIP_TO_SURFACE,
            lambda n: OrientSensorTipToSurfaceBehaviour(
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.ORIENT_TO_SURFACE,
            lambda n: OrientToSurfaceBehaviour(
                name="OrientToSurfaceBehaviour",
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.ORIENT_TO_TAG,
            lambda n: OrientToTagBehaviour(
                name="OrientToTagBehaviour",
                tag_state_source=tag_state_source,
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.CHANGE_BODY_HEIGHT,
            lambda n: ChangeBodyHeightBehaviour(
                name="ChangeBodyHeightBehaviour",
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.STAND_UP,
            lambda n: StandUpBehaviour(
                name="StandUpBehaviour",
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.SIT_DOWN,
            lambda n: SitDownBehaviour(
                name="SitDownBehaviour",
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.WAIT_TIME,
            lambda n: WaitForDuration(name="WaitForDuration"),
        ),
        (
            CommandID.CLOSE_GRIPPER,
            lambda n: CloseGripperAction(
                robot_command_resources=robot_command_resources,
            ),
        ),
        (
            CommandID.START_SLAM,
            lambda n: EnableSLAM(rtabmap_runtime),
        ),
        (
            CommandID.START_LOCALIZATION,
            lambda n: EnableLocalization(rtabmap_runtime),
        ),
        (
            CommandID.SWAP_MAP,
            lambda n: SwapMap(rtabmap_runtime),
        ),
        (
            CommandID.STOP_MAPPING,
            lambda n: StopMapping(rtabmap_runtime),
        ),
        (
            CommandID.MOVE_TO_WAYPOINT,
            build_navigate_to_goal_pose_tree,
        ),
    ]

    for command_id, constructor in specs:
        command_selector.add_child(
            make_simple_command_sequence(
                node,
                command_id,
                constructor,
            )
        )
    return command_selector


def build_cancelable_command_tree(
    node: rclpy.node.Node, *, status_ledger=None,
) -> py_trees.behaviour.Behaviour:
    cancel_check = match_command_checker(CommandID.EMERGENCY_CANCEL)
    stop_target_check = CheckBlackboardVariableValue(
        name="StopTargetsBelongToThisExecutionInstance",
        check=ComparisonExpression(
            variable="last_command",
            value=True,
            operator=lambda command, allowed: (
                getattr(command, "stop_confirmation_allowed", False) is allowed
            ),
        ),
    )
    robot_command_resources = get_helper_container(node).robot_command_resources
    from fault_detector_spot.application.behaviour_tree.behaviours.confirm_robot_stop import ConfirmRobotStop
    confirm_stop = ConfirmRobotStop(robot_command_resources)
    cancel_seq = py_trees.composites.Sequence("LatchedSoftwareStop", memory=True)
    cancel_seq.add_children([cancel_check, stop_target_check, confirm_stop])
    from fault_detector_spot.application.behaviour_tree.behaviours.reset_software_stop import ResetSoftwareStop
    reset_seq = py_trees.composites.Sequence("ResetSoftwareStopSequence", memory=True)
    reset_seq.add_children([
        match_command_checker(CommandID.RESET_SOFTWARE_STOP),
        ResetSoftwareStop(robot_command_resources, status_ledger),
    ])
    normal_tree = build_command_tree(node)

    def not_emergency(blackboard):
        return not blackboard.estop_flag

    emergency_guard = EternalGuard(
        name="EmergencyGuard",
        child=normal_tree,
        condition=not_emergency,
        blackboard_keys={"estop_flag"},
    )
    root = py_trees.composites.Selector(
        "CancelableCommandSelector",
        memory=True,
    )
    root.add_children([
        cancel_seq,
        reset_seq,
        emergency_guard,
    ])
    return StatusToBlackboard(
        name="CommandTree→BB",
        child=root,
        variable_name="command_tree_status",
    )


def build_publisher_tree(
    node: rclpy.node.Node,
    *, status_ledger=None,
) -> py_trees.behaviour.Behaviour:
    helpers = get_helper_container(node)
    runtime = helpers.rtabmap_runtime

    def execution_idle():
        return (helpers.robot_command_resources.execution_idle()
                and not runtime.has_pending_operation()
                and not runtime.nav2_runtime.has_pending_operation())

    def stop_runtime_idle():
        # Mapping shutdown is not physical stop evidence. The owners cancel
        # Nav2 and require velocity-source quiescence and stationary telemetry.
        return helpers.robot_command_resources.stop_sources_quiet()

    cmd_pub = BufferStatusPublisher(
        name="CommandStatusPublisher", status_ledger=status_ledger,
        execution_idle=execution_idle, stop_runtime_idle=stop_runtime_idle,
    )
    init_pose_pub = LandmarkRelocalizer(
        helpers.rtabmap_runtime,
        tf_buffer=helpers.robot_command_resources.get_tf_listener(node).buffer,
        name="InitPosePublisher",
    )
    publisher_tree = py_trees.composites.Parallel(
        "PublisherTree",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    publisher_tree.add_children([
        cmd_pub,
        init_pose_pub,
    ])
    return publisher_tree


def build_repeat_guarded_cancelable_command_tree(
    node: rclpy.node.Node, *, status_ledger=None,
) -> py_trees.behaviour.Behaviour:
    guard = NewCommandGuard(name="NewCommandGuard")
    guarded_sequence = py_trees.composites.Sequence(
        name="GuardedCommands",
        memory=True,
    )
    cancelable_command_tree = build_cancelable_command_tree(node, status_ledger=status_ledger)
    guarded_sequence.add_children([
        guard,
        cancelable_command_tree,
    ])
    return guarded_sequence


def make_simple_command_sequence(
    node: rclpy.node.Node,
    command_id: CommandID,
    behaviour_ctor: Callable[
        [rclpy.node.Node],
        py_trees.behaviour.Behaviour,
    ],
) -> py_trees.composites.Sequence:
    seq_name = f"{command_id.name.title().replace('_', '')}Sequence"
    seq = py_trees.composites.Sequence(
        seq_name,
        memory=True,
    )
    check = CheckBlackboardVariableValue(
        name=f"Check_{command_id.name}",
        check=ComparisonExpression(
            variable="last_command",
            value=command_id,
            operator=lambda command, candidate: (
                command is not None
                and command.command_id == candidate
            ),
        ),
    )
    child = behaviour_ctor(node)
    seq.add_children([
        check,
        child,
    ])
    return seq


def match_command_checker(
    command_id: int,
) -> CheckBlackboardVariableValue:
    return CheckBlackboardVariableValue(
        name=f"Check_{command_id}",
        check=ComparisonExpression(
            variable="last_command",
            value=command_id,
            operator=lambda command, candidate: (
                command is not None
                and command.command_id == candidate
            ),
        ),
    )


def build_navigate_to_goal_pose_tree(
    node: rclpy.node.Node,
) -> py_trees.behaviour.Behaviour:
    sequence = py_trees.composites.Sequence(
        "NavigateToWaypoint",
        memory=True,
    )
    set_goal = SetWaypointAsGoal(
        name="SetWaypointAsGoal"
    )
    navigate = NavigateToGoalPose(
        name="NavigateToGoalPose",
        robot_command_resources=get_helper_container(node).robot_command_resources,
    )
    sequence.add_children([
        set_goal,
        navigate,
    ])
    return sequence


def make_shutdown_coordinator(node, tree, executor):
    """Keep ROS callbacks and execution evidence alive through confirmation."""
    helper = helper_initializer
    resources = helper.robot_command_resources if helper is not None else None
    subscribers = ([item for item in tree.root.iterate()
                    if isinstance(item, CommandSubscriber)] if tree is not None else [])
    admission = {}
    if resources is not None:
        # Freeze submissions and resource lookup before taking teardown
        # snapshots. This operation does not wait for an execution owner.
        resources.close_admission()
        admission["motion submissions"] = resources.close_admission
    for index, subscriber in enumerate(subscribers):
        admission[f"command admission/{index}"] = subscriber.close_admission
    if tree is not None:
        dispatch = getattr(tree, "_shutdown_dispatch", None)
        if dispatch is not None:
            # A timer callback already queued in the executor must also be
            # fenced, even before the timer-cancel operation is serviced.
            dispatch.pause()
        def pause_dispatch():
            timer = getattr(tree, "timer", None)
            if timer is not None:
                timer.cancel()
        admission["tree dispatch"] = pause_dispatch

    requests, confirmations, cleanup = {}, {}, {}
    for index, subscriber in enumerate(subscribers):
        confirmations[f"prior ownership/{index}"] = subscriber.shutdown_ownership_status
    if tree is not None and dispatch is not None:
        confirmations["tree dispatch"] = dispatch.status
    retained = ({name: detail for subscriber in subscribers
                 for name, detail in subscriber.startup_unresolved.items()}
                if subscribers else StopUnconfirmedStore(owner="bt_runner").load())
    for name, detail in retained.items():
        confirmations[f"prior shutdown/{name}"] = lambda detail=detail: (False, detail)
    if resources is not None:
        requests["command sources"] = resources.begin_shutdown
        confirmations["command sources"] = resources.poll_shutdown
        for name, owner in resources.shutdown_owners().items():
            requests[name] = owner.begin_shutdown
            confirmations[name] = owner.poll_shutdown

    def report_unconfirmed(unresolved):
        if resources is not None:
            resources.report_shutdown_unconfirmed()
        errors = []
        try:
            node.get_logger().error(f"Shutdown stop-unconfirmed; reconciliation required: {unresolved}")
        except Exception as error:
            errors.append(str(error))
        if subscribers:
            for subscriber in subscribers:
                try:
                    subscriber.report_shutdown_result(unresolved)
                except Exception as error:
                    errors.append(str(error))
        else:
            StopUnconfirmedStore(owner="bt_runner").record({
                name: detail for name, detail in unresolved.items()
                if not name.startswith("prior shutdown/")
            })
        if errors:
            raise RuntimeError("; ".join(errors))

    def report_confirmed():
        if not coordinator.unresolved:
            if resources is not None:
                resources.report_shutdown_confirmed()
            for subscriber in subscribers:
                subscriber.report_shutdown_result({})

    cleanup["publish shutdown result"] = report_confirmed
    if helper is not None:
        cleanup.update(helper.cleanup_operations())
        cleanup.update(resources.cleanup_operations())
    if tree is not None:
        cleanup["behavior tree"] = lambda: tree.shutdown(destroy_node=False)
    cleanup["callback executor"] = lambda: executor.shutdown(timeout_sec=1.0)
    cleanup["ROS node"] = node.destroy_node
    cleanup["ROS context"] = rclpy.try_shutdown
    def confirm_sessions():
        if coordinator.unresolved:
            return
        if any(name != "confirm shutdown session" for name in coordinator.cleanup_errors):
            raise RuntimeError("Shutdown session retained until all resource cleanup completes")
        for subscriber in subscribers:
            if not subscriber.confirm_shutdown_session():
                raise RuntimeError("Shutdown session still owns unresolved work")
    cleanup["confirm shutdown session"] = confirm_sessions
    coordinator = ShutdownCoordinator(
        close_admission=admission, request_stop=requests,
        confirmations=confirmations, cleanup=cleanup,
        progress=lambda timeout: executor.spin_once(timeout_sec=timeout),
        report_unconfirmed=report_unconfirmed,
        timeout_sec=float(read_parameter(node, "shutdown.confirmation_timeout_sec", 10.0)),
        cleanup_timeout_sec=float(read_parameter(node, "shutdown.cleanup_timeout_sec", 20.0)),
    )
    return coordinator


def main(args=None):
    started = time.monotonic()
    startup_settings()
    with shutdown_signals() as shutdown_requested:
        # ROS's default signal handler invalidates the context before finally,
        # which would make action results and stop-service replies unavailable.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("bt_driver")
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        tree = None
        failed = False
        try:
            root = create_root(node)
            tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
            tree.setup(node=node, timeout=15.0)
            resources = get_helper_container(node).robot_command_resources
            resources.prepare_stop(node)
            if not shutdown_requested.is_set():
                for behaviour in root.iterate():
                    if isinstance(behaviour, CommandSubscriber):
                        behaviour.activate()
                start_tree_ticking(tree)
                node.get_logger().info(
                    f"BT command consumer ready at +{time.monotonic() - started:.3f}s"
                )
            while not shutdown_requested.is_set() and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        except py_trees_ros.exceptions.TimedOutError as exception:
            failed = True
            node.get_logger().error(f"Behavior tree setup failed: {exception}")
        finally:
            coordinator = make_shutdown_coordinator(node, tree, executor)
            report = coordinator.close()
            if report.cleanup_errors:
                # The ROS logger may already be destroyed; retain a diagnostic
                # even after context shutdown, and preserve helpers for retry.
                print(f"Shutdown cleanup incomplete: {report.cleanup_errors}", file=sys.stderr)
            if report.confirmed and not report.cleanup_errors:
                global helper_initializer
                helper_initializer = None
        if failed:
            sys.exit(1)


if __name__ == "__main__":
    main()
