import threading
import rclpy
from rclpy.node import Node

class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__('keyboard_input_node')
        self.get_logger().info("Type 't' + Enter to trigger move_to_goal()")
        thread = threading.Thread(target=self.listen_for_input)
        thread.daemon = True
        thread.start()

    def listen_for_input(self):
        while rclpy.ok():
            key = input().strip()
            if key == 't':
                self.get_logger().info("Pressed 't' — trigger move_to_goal() here")
            else:
                self.get_logger().info(f"Ignored key: {key}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
