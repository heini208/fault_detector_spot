# In fault_detector_spot/behaviour_tree/nodes/command_status_publisher.py

import py_trees
from py_trees.common import Status
from std_msgs.msg import String


class BufferStatusPublisher(py_trees.behaviour.Behaviour):
    """
    Every tick, reads:
      - blackboard.command_buffer  (a list of SimpleCommand)
      - blackboard.command_tree_status (a py_trees Status)
    and publishes them as JSON strings on ROS2 topics.
    """

    def __init__(self, name: str = "CmdStatusPub"):
        super().__init__(name)
        self.status_pub = None
        self.blackboard = None
        self.buffer_pub = None
        self.last_status = ""
        self.node = None

    def setup(self, **kwargs) -> bool:
        self.node = kwargs['node']
        self.blackboard = self.attach_blackboard_client()
        self.buffer_pub = self.node.create_publisher(
            String,
            'fault_detector/command_buffer',
            10
        )
        self.status_pub = self.node.create_publisher(
            String,
            'fault_detector/command_tree_status',
            10
        )
        # make sure keys exist
        self.blackboard.register_key("command_buffer", access=py_trees.common.Access.READ)
        self.blackboard.register_key("command_tree_status", access=py_trees.common.Access.READ)
        self.blackboard.register_key("last_command", access=py_trees.common.Access.READ)
        return True

    def update(self) -> Status:
        # read from the blackboard
        if self.blackboard.command_buffer is None:
            buffer_list = []
        else:
            buffer_list = self.blackboard.command_buffer

        # publish buffer as a list of IDs
        ids = [self._command_id_text(cmd) for cmd in buffer_list]
        buf_msg = String()
        buf_msg.data = f"[{','.join(ids)}]"
        self.buffer_pub.publish(buf_msg)

        # Publish each state transition once. Repeated terminal messages would
        # keep resetting the UI's post-motion settling window.
        stat_msg = self.get_status_message()
        if stat_msg.data != self.last_status:
            self.status_pub.publish(stat_msg)
            self.last_status = stat_msg.data

        return Status.SUCCESS

    def get_status_message(self) -> str:
        stat = self.blackboard.command_tree_status
        name = String()
        if stat is None:
            name.data = "IDLE"
        elif stat == Status.RUNNING:
            name.data = (
                "Running: "
                f"{self._command_id_text(self.blackboard.last_command)}"
            )
        else:
            command = self.blackboard.last_command
            command_id = (
                self._command_id_text(command)
                if command is not None
                else "unknown"
            )
            name.data = f"{stat.name}: {command_id}"
        return name

    @staticmethod
    def _command_id_text(command) -> str:
        """Normalize plain strings and string-valued Enum identifiers."""
        command_id = command.command_id
        return str(getattr(command_id, "value", command_id))
