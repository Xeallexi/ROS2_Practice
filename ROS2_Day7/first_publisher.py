import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class WriteNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub = self.create_publisher(String,"novel_channel",10)
        self.time =self.create_timer(1.0,self.timer_callback)
        self.count = 0
    def timer_callback(self):
        msg = String()
        msg.data = f"第{self.count}章开始"
        self.pub.publish(msg)
        self.get_logger().info(f"{msg.data}已发送")
        self.count += 1
def main():
    rclpy.init()
    node = WriteNode("writer_node")
    rclpy.spin(node)
    rclpy.shutdown
