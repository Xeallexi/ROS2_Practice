import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

class SimpleListener(Node):
    def __init__(self):
        super().__init__('simple_listener')

        # === 1. 准备工作 ===
        self.target = 'turtle1'
        self.source = 'turtle2'
        
        # 创建 TF 缓存（记忆）和监听器（耳朵）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建发布者：控制 turtle2 移动
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # 创建客户端
        self.spawner = self.create_client(Spawn, 'spawn')
        # 等待服务上线 (最多等 2 秒，等不到就算了)
        self.spawner.wait_for_service(timeout_sec=2.0)
        
        # 发送请求
        req = Spawn.Request()
        req.name = self.source
        req.x, req.y, req.theta = 4.0, 2.0, 0.0
        self.spawner.call_async(req) 


        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        # === 核心逻辑：查位置 -> 算速度 -> 跑 ===
        
        try:
            # 1. 查：turtle1 到底在 turtle2 的哪个方位？
            # 这里的 when=rclpy.time.Time() 代表“查最新的一刻”
            t = self.tf_buffer.lookup_transform(
                self.source,    # 我 (参考系)
                self.target,    # 目标
                rclpy.time.Time())
        except TransformException:
            # 如果 turtle2 还没生出来，或者 TF 还没连上
            # 这里直接“静音”，不做任何事，等下一次循环
            return

        # 2. 算：数学计算 (这部分是死的公式)
        msg = Twist()
        
        # x, y 是目标相对于我的距离
        x = t.transform.translation.x
        y = t.transform.translation.y

        # 转向：用 atan2 算出角度差，乘以 4.0 是为了转快点
        msg.angular.z = 4.0 * math.atan2(y, x)
        
        # 前进：用勾股定理算距离，乘以 0.5 是为了跑慢点
        distance = math.sqrt(x ** 2 + y ** 2)
        msg.linear.x = 0.5 * distance

        # 3. 跑
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = SimpleListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()