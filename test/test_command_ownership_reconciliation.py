"""Exercise controller, ROS envelopes, BT receipts and owner reconciliation offline."""

from types import SimpleNamespace

from fault_detector_msgs.msg import CommandStatus
from fault_detector_msgs.srv import GetCommandStatus
from py_trees.common import Status

from fault_detector_spot.application.behaviour_tree.behaviours.buffer_and_status_publisher import BufferStatusPublisher
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.behaviour_tree.command_status_ledger import CommandStatusLedger
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.command_controller import CommandController, CommandControllerState
from fault_detector_spot.application.ros.command_transport import RosCommandTransport
from test_command_ros_transport import FakeNode, FakePublisher, make_request


def pipeline():
    now, proof = [0.0], [False]
    node, ledger = FakeNode(auto_discovery=False), CommandStatusLedger()
    ledger.activate()
    controller = CommandController(monotonic_clock=lambda: now[0])
    states = []
    controller.add_status_listener(states.append)
    transport = RosCommandTransport(node, controller, monotonic_clock=lambda: now[0])
    subscriber = CommandSubscriber(status_ledger=ledger)
    subscriber.node = node
    subscriber.blackboard = SimpleNamespace(command_buffer=[], estop_flag=False,
                                             last_command=None, command_tree_status=Status.INVALID)
    subscriber.request_status_publisher = FakePublisher()
    publisher = BufferStatusPublisher(status_ledger=ledger, execution_idle=lambda: proof[0],
                                      stop_runtime_idle=lambda: proof[0])
    publisher.node = node
    publisher.blackboard = subscriber.blackboard
    publisher.structured_status_pub = FakePublisher()
    discovery, future = node.status_client.calls[-1]
    future.set_result(subscriber.query_status(discovery, GetCommandStatus.Response()))
    return SimpleNamespace(now=now, proof=proof, node=node, controller=controller, states=states,
                           transport=transport, subscriber=subscriber, publisher=publisher,
                           wire=node.publishers['fault_detector/_internal/commands/request'].messages)


def complete_query(p):
    request, future = p.node.status_client.calls[-1]
    response = p.subscriber.query_status(request, GetCommandStatus.Response())
    future.set_result(response)


def test_lost_receipts_duplicate_delivery_and_lost_terminal_only_advance_after_owner_proof():
    p = pipeline()
    first, second = make_request(CommandID.READY_ARM), make_request(CommandID.STOW_ARM)
    p.controller.submit(first)
    p.controller.submit(second)
    p.subscriber.append_request_to_buffer(p.wire[0])
    p.subscriber.append_request_to_buffer(p.wire[0])
    p.subscriber.update()
    assert len(p.subscriber.blackboard.command_buffer) == 1
    # Drop every receipt. The controller retains ownership and queries, without replay.
    p.now[0] = 4.0
    p.controller.poll()
    assert next(status for status in reversed(p.states) if status.request_id == first.request_id).state is CommandControllerState.UNKNOWN
    assert len(p.wire) == 1
    complete_query(p)
    assert p.controller.active_request_id == first.request_id
    command = p.subscriber.blackboard.command_buffer.pop()
    p.subscriber.blackboard.last_command = command
    p.subscriber.blackboard.command_tree_status = Status.SUCCESS
    p.publisher.update()
    p.transport.handle_command_status(p.publisher.structured_status_pub.messages[-1])
    assert p.controller.active_request_id == first.request_id
    assert len(p.wire) == 1
    # Owner later proves idle. Drop this publication too and recover it by query.
    p.proof[0] = True
    p.publisher.update()
    p.now[0] = 8.0
    p.controller.poll()
    complete_query(p)
    assert p.controller.active_request_id == second.request_id
    assert len(p.wire) == 2
    p.subscriber.append_request_to_buffer(p.wire[0])
    assert p.subscriber.pending_msgs == []
    late = p.subscriber.request_status_publisher.messages[-1]
    assert late.state == CommandStatus.STATE_SUCCEEDED and late.execution_idle
    assert not p.transport.handle_command_status(late)


def test_stop_fences_a_delayed_original_and_confirms_each_lane_owner():
    p = pipeline()
    original = make_request(CommandID.READY_ARM)
    p.controller.submit(original)
    stop_id = p.controller.cancel_all()
    p.controller.poll()
    later = make_request(CommandID.STOW_ARM)
    p.controller.submit(later)
    # Deliver stop before the original request, then deliver that motion late.
    p.subscriber.append_request_to_buffer(p.wire[1])
    p.subscriber.append_request_to_buffer(p.wire[0])
    assert p.subscriber.pending_msgs == []
    assert all(command.command_id is CommandID.EMERGENCY_CANCEL
               for command in p.subscriber.blackboard.command_buffer)
    for receipt in p.subscriber.request_status_publisher.messages:
        p.transport.handle_command_status(receipt)
    assert p.controller.active_request_id == original.request_id
    assert len(p.wire) == 2
    command = p.subscriber.blackboard.command_buffer.pop()
    command.stop_confirmed = True
    p.subscriber.blackboard.last_command = command
    p.subscriber.blackboard.command_tree_status = Status.SUCCESS
    p.proof[0] = True
    p.publisher.update()
    confirmations = p.publisher.structured_status_pub.messages
    assert {status.request_id for status in confirmations} == {original.request_id, stop_id}
    for index, status in enumerate(confirmations):
        assert status.state == CommandStatus.STATE_STOP_CONFIRMED and status.execution_idle
        p.transport.handle_command_status(status)
        assert len(p.wire) == (2 if index == 0 else 3)
    p.transport._query_status("")
    complete_query(p)
    p.controller.poll()
    assert p.controller.active_request_id == later.request_id
