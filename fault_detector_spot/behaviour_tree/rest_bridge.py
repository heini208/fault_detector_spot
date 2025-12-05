import threading
import time
import typing as t
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI

import rclpy
from fault_detector_msgs.msg import ComplexCommand, BasicCommand
from fault_detector_spot.behaviour_tree.QOS_PROFILES import COMMAND_QOS
from rclpy.node import Node
from std_msgs.msg import Header


class FaultDetectorRestBridge(Node):
    def __init__(self):
        super().__init__("fault_detector_rest_bridge")
        self.complex_command_publisher = self.create_publisher(
            ComplexCommand, "fault_detector/commands/complex_command", COMMAND_QOS
        )

    def build_basic_command(self, command_id: str) -> BasicCommand:
        cmd = BasicCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.command_id = command_id
        return cmd

    def handle_simple_command(self, command_id: str):
        cmd = self.build_basic_command(command_id)
        as_complex_command = ComplexCommand()
        as_complex_command.command = cmd
        self.complex_command_publisher.publish(as_complex_command)


node: t.Optional[FaultDetectorRestBridge] = None


def ros_thread_init():
    global node

    rclpy.init()
    node = FaultDetectorRestBridge()
    rclpy.spin(node)


def start_ros_thread():
    global node

    ros_thread = threading.Thread(target=ros_thread_init, daemon=True)
    ros_thread.start()
    wait_seconds = 0.0
    while node is None and wait_seconds <= 3:
        time.sleep(0.05)
        wait_seconds += 0.05
    return ros_thread


###
start_ros_thread()


@asynccontextmanager
async def lifespan(app: FastAPI):
    yield
    # Shutdown logic
    global node
    if node is not None:
        node.get_logger().info("Shutting down FaultDetectorRestBridge node")
        node.destroy_node()
        import rclpy
        rclpy.shutdown()


app = FastAPI(title="Fault Detector REST Bridge")


@app.post("/send_basic_command/{command_id}")
def send_basic_command(command_id: str):
    global node
    if node is None:
        return {"status": "error", "message": "ROS node not initialized"}
    node.handle_simple_command(command_id)
    return {"status": "success", "message": f"Command {command_id} sent"}


def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)