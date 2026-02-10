import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
from sensor_msgs.msg import LaserScan    # 导入雷达扫描消息类型
from geometry_msgs.msg import Twist       # 导入用于控制移动的速度消息类型
class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")
        
        # 1. 创建发布者：负责给小车发“走”或“停”的指令
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10) 
        
        # 2. 创建订阅者：负责听雷达传回的距离数据
        self.subscription = self.create_subscription(
            LaserScan, 
            "scan", 
            self.scan_callback, 
            10) 
            
        self.get_logger().info("避障节点已就绪，正在监听雷达数据...")

    def scan_callback(self, msg):
    # 1. 提取前方一小块扇区的数据（防止单点误差）
    # 假设 0 是正中心，取左右各 10 个点
        all_ranges = [r for r in msg.ranges if r > 0.05]
        min_dist = min(all_ranges)
    
    # 2. 创建速度消息对象
        twist = Twist()
    
    # 3. 核心决策逻辑
        if min_dist < 0.6:  # 稍微留点余量，比如 0.6米 
            twist.linear.x = 0.0  # 停车 [cite: 2026-02-08]
            self.get_logger().warn(f"危险！最近距离 {min_dist:.2f}m，执行制动")
        else:
            twist.linear.x = 0.2  # 安全，继续慢速前进
        
    # 4. 发布速度
        self.publisher_.publish(twist)
        
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()