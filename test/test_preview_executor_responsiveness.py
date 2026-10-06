"""Bulk preview requests must leave executor capacity for live inputs."""

from threading import Event, Lock, Thread
from types import SimpleNamespace
from uuid import uuid4
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from fault_detector_msgs.srv import GetProbeReferencePreview

from fault_detector_spot.application.api.probe_setup_api import ProbeSetupApi
from fault_detector_spot.inspection.setup.reference_view_depth_projection import ImageRegion


def test_six_previews_do_not_starve_live_input_callbacks():
    # Dedicated ROS domain and namespace: this test never contacts Spot.
    context = Context()
    rclpy.init(context=context, domain_id=217)
    namespace = '/preview_test_' + uuid4().hex
    server = rclpy.create_node('server', namespace=namespace, context=context)
    client_node = rclpy.create_node('client', namespace=namespace, context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    release, started, heartbeat = Event(), Event(), Event()
    guard = Lock()
    active = 0
    peak = 0

    def load(_snapshot, view_id):
        nonlocal active, peak
        with guard:
            active += 1
            peak = max(peak, active)
        started.set()
        try:
            if not release.wait(8):
                raise RuntimeError('Test preview was not released')
            return SimpleNamespace(
                reference_view_id=view_id, camera_id='hand', slot_index=0,
                image=Image(), selectable_region=ImageRegion(0, 0, 1, 1),
            )
        finally:
            with guard:
                active -= 1

    state_listeners = []
    coordinator = SimpleNamespace(
        context=lambda *_: object(), snapshot=lambda _: object(),
        add_state_listener=state_listeners.append,
        remove_state_listener=state_listeners.remove,
    )
    api = ProbeSetupApi(server, coordinator, None, None, SimpleNamespace(load=load))
    timer = server.create_timer(0.02, heartbeat.set)
    client = client_node.create_client(
        GetProbeReferencePreview, 'fault_detector/application/get_probe_reference_preview',
    )
    executor.add_node(server)
    executor.add_node(client_node)
    thread = Thread(target=executor.spin)
    thread.start()
    try:
        assert client.wait_for_service(timeout_sec=5)
        futures = []
        for index in range(6):
            request = GetProbeReferencePreview.Request()
            request.reference_view_id = str(index)
            futures.append(client.call_async(request))
        assert started.wait(3)
        # Allow the service backlog to occupy workers, then require a NEW
        # heartbeat while the preview handler remains deliberately blocked.
        time.sleep(0.5)
        heartbeat.clear()
        assert heartbeat.wait(1), 'Preview backlog starved live input callbacks'
        assert peak == 1
        release.set()
        deadline = time.monotonic() + 5
        while not all(future.done() for future in futures) and time.monotonic() < deadline:
            time.sleep(0.02)
        assert all(f.done() and f.result().success for f in futures)
    finally:
        release.set()
        executor.shutdown()
        thread.join(timeout=5)
        server.destroy_timer(timer)
        api.close()
        server.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown(context=context)
