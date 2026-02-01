import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster  # 广播员工具
from geometry_msgs.msg import TransformStamped # 信封格式
from turtlesim.msg import Pose # 海龟位置格式

# === 1. 数学翻译官 (直接用，不用管原理) ===
# 作用：把欧拉角 (roll, pitch, yaw) 转换成 四元数 (x, y, z, w)
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    q = [0.0] * 4
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

class TurtleTFBroadcaster(Node):

    def __init__(self):
        super().__init__('turtle_tf_broadcaster')

        # 声明一个参数：这就好比给广播员发个工牌，写上负责谁
        # 默认负责 'turtle1'
        self.declare_parameter('turtlename', 'turtle1')
        self.turtlename = self.get_parameter('turtlename').get_parameter_value().string_value

        # === 2. 创建广播员 ===
        self.tf_broadcaster = TransformBroadcaster(self)

        # === 3. 创建订阅者 ===
        # 任务：监听 /turtle1/pose
        # 一旦收到消息，就触发 self.handle_turtle_pose 函数
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1)
        self.get_logger().info(f'开始广播 {self.turtlename} 的坐标...')

    def handle_turtle_pose(self, msg):
        # === 4. 收到位置后，打包并广播 ===
        t = TransformStamped()

        # A. 填头信息 (盖戳)
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'          # 参考系：世界
        t.child_frame_id = self.turtlename   # 我是谁：turtle1

        # B. 填位置 (平移)
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # C. 填角度 (旋转) - 调用上面的翻译官函数
        # 海龟只有平面旋转 (theta)，所以前两个参数是 0
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # D. 发送！
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = TurtleTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()