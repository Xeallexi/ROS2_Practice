import rclpy                     # 1. 导入ROS2库
from rclpy.node import Node      # 2. 导入节点爹

class HelloNode(Node):           # 3. 创建你的节点类
    def __init__(self):
        super().__init__("hello_node")   # 4. 给节点起名
        self.get_logger().info("Hello ROS2! 我活了！")  # 5. 打印一句话

def main():                      # 6. 主函数（固定写法）
    rclpy.init()                 # 7. ROS2开机
    node = HelloNode()           # 8. 创建节点
    rclpy.spin(node)             # 9. 让节点开始运行（最重要！没有这行啥都不打印）
    rclpy.shutdown()             # 10. ROS2关机

main()                           # 运行main