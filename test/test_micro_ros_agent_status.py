"""Tests for host micro-ROS Agent status discovery."""

from types import SimpleNamespace

from fault_detector_spot.sensing import micro_ros_agent_status_node as status


class _Process:

    def __init__(self, _process_id, command="micro_ros_agent"):
        self.command = command

    def name(self):
        return self.command

    def cmdline(self):
        return [self.command, "udp4", "--port", "8888"]


def _connection(port, process_id=42):
    return SimpleNamespace(
        laddr=SimpleNamespace(port=port),
        pid=process_id,
    )


def test_configured_advertised_address_is_used_verbatim():
    address, detail = status.select_advertised_ipv4("192.168.178.69")

    assert address == "192.168.178.69"
    assert "configured" in detail


def test_invalid_or_loopback_advertised_address_is_rejected():
    assert status.select_advertised_ipv4("127.0.0.1")[0] == ""
    assert status.select_advertised_ipv4("not-an-ip")[0] == ""


def test_auto_address_prefers_default_route(monkeypatch):
    monkeypatch.setattr(status, "_route_ipv4", lambda: "192.168.1.23")
    monkeypatch.setattr(
        status,
        "_interface_ipv4_candidates",
        lambda: ("10.0.0.4",),
    )

    assert status.select_advertised_ipv4("auto")[0] == "192.168.1.23"


def test_agent_must_own_the_configured_udp_port():
    running, detail = status.micro_ros_agent_owns_udp_port(
        8888,
        connections=[_connection(8888)],
        process_factory=_Process,
    )

    assert running
    assert "owns UDP port 8888" in detail


def test_unrelated_process_or_port_does_not_report_running():
    def unrelated(process_id):
        return _Process(process_id, "other-process")

    running, _ = status.micro_ros_agent_owns_udp_port(
        8888,
        connections=[_connection(9999), _connection(8888)],
        process_factory=unrelated,
    )

    assert not running
