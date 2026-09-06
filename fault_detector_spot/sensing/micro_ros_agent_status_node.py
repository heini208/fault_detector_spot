"""Publish the local micro-ROS Agent endpoint and process health."""

import ipaddress
import socket
from typing import Callable, Iterable, Optional, Tuple

from fault_detector_msgs.msg import MicroRosAgentState
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
import psutil
import rclpy
from rclpy.node import Node


STATUS_TOPIC = "fault_detector/state/micro_ros_agent"
_AUTO_ADDRESS_VALUES = {"", "auto"}
_AGENT_PROCESS_TOKEN = "micro_ros_agent"


def _usable_ipv4(value: str) -> Optional[str]:
    """Return a normalized non-loopback IPv4 address, when usable."""
    try:
        address = ipaddress.IPv4Address(value.strip())
    except ipaddress.AddressValueError:
        return None
    if address.is_unspecified or address.is_loopback or address.is_link_local:
        return None
    return str(address)


def _route_ipv4() -> Optional[str]:
    """Resolve the IPv4 chosen by the host's default route without sending."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route_socket:
            route_socket.connect(("192.0.2.1", 9))
            return _usable_ipv4(route_socket.getsockname()[0])
    except OSError:
        return None


def _interface_ipv4_candidates() -> Tuple[str, ...]:
    candidates = set()
    for addresses in psutil.net_if_addrs().values():
        for address in addresses:
            if address.family != socket.AF_INET:
                continue
            normalized = _usable_ipv4(address.address)
            if normalized:
                candidates.add(normalized)
    return tuple(sorted(candidates))


def select_advertised_ipv4(configured: str = "auto") -> Tuple[str, str]:
    """Choose the endpoint address shown to sensor-mount users."""
    normalized = configured.strip().lower()
    if normalized not in _AUTO_ADDRESS_VALUES:
        address = _usable_ipv4(configured)
        if address:
            return address, "configured advertised address"
        return "", f"invalid advertised IPv4 address: {configured}"

    routed = _route_ipv4()
    if routed:
        return routed, "address selected from the default IPv4 route"

    candidates = _interface_ipv4_candidates()
    if candidates:
        return candidates[0], "address selected from a network interface"
    return "", "no usable LAN IPv4 address found"


def micro_ros_agent_owns_udp_port(
    port: int,
    connections: Optional[Iterable] = None,
    process_factory: Callable = psutil.Process,
) -> Tuple[bool, str]:
    """Return whether a micro_ros_agent process owns the UDP/IPv4 port."""
    try:
        udp_connections = (
            tuple(connections)
            if connections is not None
            else psutil.net_connections(kind="udp4")
        )
    except (OSError, psutil.Error) as exception:
        return False, f"cannot inspect UDP listeners: {exception}"

    for connection in udp_connections:
        local_address = getattr(connection, "laddr", None)
        if not local_address or getattr(local_address, "port", None) != port:
            continue
        process_id = getattr(connection, "pid", None)
        if process_id is None:
            continue
        try:
            process = process_factory(process_id)
            identity = " ".join(
                [process.name(), *process.cmdline()]
            ).lower()
        except (OSError, psutil.Error):
            continue
        if _AGENT_PROCESS_TOKEN in identity:
            return True, f"micro-ROS Agent owns UDP port {port}"
    return False, f"micro-ROS Agent is not listening on UDP port {port}"


class MicroRosAgentStatusNode(Node):
    """Publish actual local Agent socket ownership for UI consumers."""

    def __init__(self):
        super().__init__("micro_ros_agent_status")
        self.declare_parameter("agent.transport", "udp4")
        self.declare_parameter("agent.port", 8888)
        self.declare_parameter("agent.advertised_address", "auto")
        self.declare_parameter("agent.poll_period_sec", 1.0)

        self.transport = str(
            self.get_parameter("agent.transport").value
        ).strip()
        self.port = int(self.get_parameter("agent.port").value)
        self.configured_address = str(
            self.get_parameter("agent.advertised_address").value
        )
        poll_period = max(
            0.2,
            float(self.get_parameter("agent.poll_period_sec").value),
        )

        self.publisher = self.create_publisher(
            MicroRosAgentState,
            STATUS_TOPIC,
            LATCHED_QOS,
        )
        self._last_log_signature = None
        self.timer = self.create_timer(poll_period, self.publish_status)
        self.publish_status()

    def publish_status(self) -> None:
        address, address_detail = select_advertised_ipv4(
            self.configured_address
        )
        if self.transport != "udp4":
            running = False
            process_detail = (
                f"status inspection does not support {self.transport}"
            )
        else:
            running, process_detail = micro_ros_agent_owns_udp_port(
                self.port
            )

        message = MicroRosAgentState()
        message.stamp = self.get_clock().now().to_msg()
        message.running = running
        message.transport = self.transport
        message.advertised_address = address
        message.port = self.port
        message.detail = f"{process_detail}; {address_detail}"
        self.publisher.publish(message)

        signature = (running, address, self.port, message.detail)
        if signature == self._last_log_signature:
            return
        self._last_log_signature = signature
        log = self.get_logger().info if running else self.get_logger().warning
        log(message.detail)


def main(args=None):
    rclpy.init(args=args)
    node = MicroRosAgentStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
