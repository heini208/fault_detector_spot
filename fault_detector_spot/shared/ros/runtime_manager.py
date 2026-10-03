"""Shared ownership and lifecycle of a nested ROS launch process.

Process liveness is deliberately distinct from ROS lifecycle/action readiness.
Subclasses own launch arguments and domain-specific mode/save operations.
"""

import subprocess
from concurrent.futures import ThreadPoolExecutor
from threading import RLock

import py_trees

from fault_detector_spot.shared.ros.process_lifecycle import (
    is_process_group_running, terminate_process_group,
)


class RuntimeManager:
    PROCESS_KEY = None
    RUNTIME_NAME = "ROS"
    INTERRUPT_TIMEOUT_SEC = 3.0

    def __init__(self, node, blackboard):
        if not self.PROCESS_KEY:
            raise TypeError("Runtime managers must define a process key")
        self.node = node
        self.bb = blackboard
        self.bb.register_key(self.PROCESS_KEY, access=py_trees.common.Access.WRITE)
        if not self.bb.exists(self.PROCESS_KEY):
            setattr(self.bb, self.PROCESS_KEY, None)
        self._runtime_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix=f"{self.RUNTIME_NAME}-runtime",
        )
        self._runtime_lock = RLock()
        self._process_lock = RLock()
        self._runtime_future = None
        self._runtime_operation = ""
        self._closing = False
        self._executor_closed = False
        self._closed = False

    @property
    def process(self):
        return getattr(self.bb, self.PROCESS_KEY, None)

    def is_running(self) -> bool:
        """Check the owned process group, not ROS node readiness."""
        return is_process_group_running(self.process)

    def _use_sim_time(self) -> bool:
        try:
            return bool(self.node.get_parameter("use_sim_time").value)
        except Exception:
            return False

    def _use_sim_time_launch_arg(self) -> str:
        return "true" if self._use_sim_time() else "false"

    def _launch(self, launch_file, launch_args=()):
        with self._process_lock:
            if self._closing:
                raise RuntimeError(f"{self.RUNTIME_NAME} runtime is shutting down")
            if self.is_running():
                return self.process
            args = list(launch_args)
            if not any(item.startswith("use_sim_time:=") for item in args):
                args.append(f"use_sim_time:={self._use_sim_time_launch_arg()}")
            process = subprocess.Popen(
                ["ros2", "launch", "fault_detector_spot", launch_file, *args],
                start_new_session=True,
            )
            setattr(self.bb, self.PROCESS_KEY, process)
            self.node.get_logger().info(
                f"Started {self.RUNTIME_NAME} with PID {process.pid}"
            )
            return process

    def _stop_process(self, interrupt_timeout_sec=None,
                      terminate_timeout_sec=2.0, kill_timeout_sec=1.0):
        process = self.process
        if process is None:
            return True
        if not terminate_process_group(
            process,
            interrupt_timeout_sec=(self.INTERRUPT_TIMEOUT_SEC
                                   if interrupt_timeout_sec is None else interrupt_timeout_sec),
            terminate_timeout_sec=terminate_timeout_sec,
            kill_timeout_sec=kill_timeout_sec,
        ):
            self.node.get_logger().error(f"{self.RUNTIME_NAME} process did not terminate")
            return False
        setattr(self.bb, self.PROCESS_KEY, None)
        self.node.get_logger().info(f"Stopped {self.RUNTIME_NAME}")
        return True

    def stop(self) -> bool:
        """Stop the owned group; retain ownership if termination fails."""
        with self._process_lock:
            return self._stop_process()

    def _stop_for_close(self):
        return self.stop()

    def begin_runtime_operation(
        self,
        operation_name: str,
        callback,
        *args,
        **kwargs,
    ) -> bool:
        with self._runtime_lock:
            if self._closing:
                raise RuntimeError(f"{self.RUNTIME_NAME} runtime is shutting down")
            self._reap_completed_runtime_operation()
            if self._runtime_future is not None:
                return False
            self._runtime_operation = operation_name
            self._runtime_future = self._runtime_executor.submit(
                callback,
                *args,
                **kwargs,
            )
            return True

    def poll_runtime_operation(self, operation_name: str):
        with self._runtime_lock:
            future = self._runtime_future
            if future is None:
                raise RuntimeError(
                    f"Runtime operation '{operation_name}' is not active"
                )
            if self._runtime_operation != operation_name:
                raise RuntimeError(
                    f"Another {self.RUNTIME_NAME} runtime operation is active: "
                    f"{self._runtime_operation}"
                )
            if not future.done():
                return None

            self._runtime_future = None
            self._runtime_operation = ""

        return future.result()

    def _reap_completed_runtime_operation(self):
        future = self._runtime_future
        if future is None or not future.done():
            return

        operation_name = self._runtime_operation
        self._runtime_future = None
        self._runtime_operation = ""
        try:
            future.result()
        except Exception as exception:
            self.node.get_logger().warning(
                f"Discarded completed {self.RUNTIME_NAME} runtime operation "
                f"'{operation_name}' after preemption: {exception}"
            )

    def close(self):
        """Stop asynchronous work, then terminate every owned process group."""
        with self._runtime_lock:
            if self._closed:
                return True
            self._closing = True
            executor_closed = self._executor_closed

        if not executor_closed:
            self._runtime_executor.shutdown(
                wait=True,
                cancel_futures=True,
            )
            with self._runtime_lock:
                self._executor_closed = True
                future = self._runtime_future
                operation_name = self._runtime_operation
                self._runtime_future = None
                self._runtime_operation = ""

            if future is not None and future.done():
                try:
                    future.result()
                except Exception as exception:
                    self.node.get_logger().warning(
                        f"{self.RUNTIME_NAME} runtime operation "
                        f"'{operation_name}' failed during shutdown: "
                        f"{exception}"
                    )

        if not self._stop_for_close():
            raise RuntimeError(f"{self.RUNTIME_NAME} process did not terminate")
        with self._runtime_lock:
            self._closed = True
        return True

