"""Own the mapping-independent arm occupancy-checking preference."""

from dataclasses import dataclass
from threading import RLock

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.clock import Clock, ClockType
from std_srvs.srv import SetBool

from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS


@dataclass(frozen=True)
class ArmCollisionPolicy:
    enabled: bool
    revision: int


class ArmCollisionControl:
    """Publish planning intent; toggling never moves the arm or changes a scene.

    Availability means the control is online, not that obstacle sensors exist.
    Each application process starts disabled, independently of mapping lifecycle.
    """

    def __init__(self, node):
        self.node = node
        self._lock = RLock()
        self._state = ArmCollisionPolicy(enabled=False, revision=0)
        self._closed = False
        self._publisher = node.create_publisher(
            DiagnosticStatus, "fault_detector/arm_collision_checking", LATCHED_QOS,
        )
        self._service = node.create_service(
            SetBool, "fault_detector/set_arm_collision_checking", self._set_enabled,
        )
        self._timer = node.create_timer(
            0.5, self._publish_state, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._publish_state()

    def state(self):
        with self._lock:
            return self._state

    def set_enabled(self, enabled):
        if type(enabled) is not bool:
            raise TypeError("Arm collision checking must be a boolean")
        with self._lock:
            if self._closed:
                raise RuntimeError("Arm collision control is closed")
            if enabled != self._state.enabled:
                self._state = ArmCollisionPolicy(enabled, self._state.revision + 1)
            self._publish_state()
            return self._state

    def _set_enabled(self, request, response):
        try:
            state = self.set_enabled(request.data)
        except (TypeError, RuntimeError) as exception:
            response.success = False
            response.message = str(exception)
        else:
            response.success = True
            response.message = (
                "Environmental occupancy checking enabled for the current MoveIt scene"
                if state.enabled else "Environmental occupancy checking disabled"
            )
        return response

    def _publish_state(self):
        with self._lock:
            if self._closed:
                return
            self._publisher.publish(DiagnosticStatus(
                name="arm_collision_checking",
                level=DiagnosticStatus.OK,
                message="Enabled" if self._state.enabled else "Disabled",
                values=[
                    KeyValue(key="arm_collision_available", value="true"),
                    KeyValue(key="arm_collision_enabled", value=str(self._state.enabled).lower()),
                ],
            ))

    def destroy(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self.node.destroy_timer(self._timer)
        self.node.destroy_service(self._service)
        self.node.destroy_publisher(self._publisher)
