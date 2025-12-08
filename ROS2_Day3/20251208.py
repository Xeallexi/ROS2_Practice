# 20251208 Day11
"""
import rclpy
from rclpy.node import Node
class HelloNOde(Node):
    def __init__(self):
        super.__init__("hi")
        self.get_logger.info("hello")
def main():
    rclpy.init()
    node = HelloNOde
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
"""
#发布者模板
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__("topic_helloworld_pub")   # 直接写死名字，更清晰！
        
        self.pub = self.create_publisher(String, "chatter", 10)
        self.create_timer(0.5, self.timer_callback)  # 直接写定时器

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "Hello World"')

def main():
    rclpy.init()
    node = PublisherNode()          # 直接创建，不用传名字
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#订阅者模板
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__("topic_helloworld_sub")           # 节点名直接写死
        self.create_subscription(String, "chatter", self.callback, 10)

    def callback(self, msg):                               # 收到消息就执行
        self.get_logger().info(f"I heard: {msg.data}")     # 打印收到的内容

def main():
    rclpy.init()
    node = SubscriberNode()
    rclpy.spin(node)           # 让节点一直活着
    rclpy.shutdown()

if __name__ == '__main__':
    main()
