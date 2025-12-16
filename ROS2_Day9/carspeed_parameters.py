import rclpy
from rclpy.node import Node

# 定义一个类，继承自 Node
class CarSpeedNode(Node):
    
    def __init__(self):
        # 1. 初始化父类，节点名叫 'speed_node'
        super().__init__('speed_node')
        
        # ==========================================
        # 核心步骤 A：声明参数 (Declare)
        # 意思：告诉系统，我有一个参数叫 'speed'，默认值是 1.0
        # 这一步相当于在字典里新建一个 Key
        # ==========================================
        self.declare_parameter('speed', 1.0)
        self.declare_parameter("carname","Tesla")
        
        # 创建一个定时器，每 1 秒执行一次 print_speed 函数
        self.timer = self.create_timer(1.0, self.timer_print_speed)

    def timer_print_speed(self):
        # ==========================================
        # 核心步骤 B：获取参数 (Get)
        # 意思：从系统里读出 'speed' 当前的值
        # 注意：.value 才是真正的数据，不加 .value 拿到的是一个对象
        # ==========================================
        current_speed = self.get_parameter('speed').value
        current_carname = self.get_parameter('carname').value
        # 打印出来
        self.get_logger().info(f'我是:{current_carname}当前车速是: {current_speed} km/h')

def main(args=None):
    rclpy.init(args=args)
    node = CarSpeedNode()
    rclpy.spin(node) # 保持运行
    node.destroy_node()
    rclpy.shutdown()