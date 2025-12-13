import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class ReadNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.sub = self.create_subscription(String,"novel_channel",self.recv_callback,10)
    def recv_callback(self,msg):
        self.get_logger().info(f"收到了{msg.data}")
def main():
    rclpy.init()
    node = ReadNode("read_node")
    rclpy.spin(node)
    rclpy.shutdown()