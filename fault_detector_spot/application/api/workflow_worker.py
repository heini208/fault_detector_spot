"""Run remaining blocking workflows outside the ROS callback executor."""

from threading import Thread

from rclpy.task import Future


def run_in_worker(admission, key, work):
    """Run admitted work once; its permit bounds capacity and owns cancellation."""
    if not admission.claim(key):
        raise RuntimeError("Workflow has no available admission reservation")
    future = Future()

    def run():
        try:
            result = work(lambda: admission.cancel_requested(key))
        except BaseException as error:
            admission.release(key)
            future.set_exception(error)
        else:
            admission.release(key)
            future.set_result(result)

    try:
        Thread(target=run, name="application-workflow", daemon=True).start()
    except BaseException:
        admission.release(key)
        raise
    return future


def deliver_update(node, publish, message):
    """Observer failures must not erase a saved-data or terminal result."""
    try:
        publish(message)
    except Exception as error:
        try:
            node.get_logger().warning(f"Probe workflow update could not be delivered: {error}")
        except Exception:
            pass  # Completion must also survive logging during teardown.
