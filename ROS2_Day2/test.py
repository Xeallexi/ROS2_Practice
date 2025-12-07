#2025.12.7 Day11
#复习节点
import rclpy
import time
from rclpy.node import Node
class helloNode(Node):
    def __init__(self):
        super().__init__("hello")
        self.get_logger().info("节点启动")
        while rclpy.ok():
            self.get_logger().info("hello")
            time.sleep(0.5)
def main():
    rclpy.init()
    node = helloNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
#1. 导入爹和库：          import rclpy + from rclpy.node import Node
#2. 创建你的儿子：        class 节点名(Node):
#3. 出生先起名：          super().__init__("节点名字")
#4. 想干啥写这儿：        self.get_logger().info("hello")
#5. 主函数开机：          rclpy.init()
#6. 生出儿子：            node = 节点名()
#7. 让他活起来：          rclpy.spin(node)     ← 灵魂！！
#8. 优雅关机：            rclpy.shutdown()
#9. 防止乱跑：            if __name__ == '__main__':
#10. 启动大戏：           main()

#话题
#节点间的传递数据的桥梁
#发布者和订阅者可以不唯一  可以不同时间 也就是异步
#.msg 文件定义通信的消息结构
# publisher 创建接口初始化 创建节点初始化 创建发布者对象 创建并填充话题消息 发布话题消息 销毁节点关闭接口
# subsriber 编程接口初始化 创建节点初始化 创建订阅者对象 回调函数处理话题数据 销毁节点关闭窗口
"""
import rclpy                                     # 拿到了ROS2的“微信”
from rclpy.node import Node                      # 拿到了“微信号”的模板
from std_msgs.msg import String                  # 拿到了“文字消息”的格式

class PublisherNode(Node):                       # 我要注册一个新微信号，名字叫PublisherNode
    def __init__(self, name):                     # 一出生就要干的事
        super().__init__(name)                    # 1. 先给这个号起个名字（比如叫topic_helloworld_pub）
        
        self.pub = self.create_publisher(         # 2. 开通一个“发消息”的权限
            String,                               #    只能发文字消息
            "chatter",                            #    发到叫“chatter”的群里
            10)                                   #    最多缓存10条消息
        
        self.timer = self.create_timer(0.5, self.timer_callback)  
        # 3. 设置一个闹钟：每0.5秒叫醒我一次，去执行timer_callback这个函数

    def timer_callback(self):                     # 闹钟响了！我要干的事
        msg = String()                            #    造一条空白文字消息
        msg.data = 'Hello World'                  #    填上内容
        self.pub.publish(msg)                     #    发到“chatter”群里
        self.get_logger().info('Publishing: "Hello World"')  # 终端里打印：我刚发了啥

def main():                                       # 程序的起点
    rclpy.init()                                  # 打开微信
    node = PublisherNode("topic_helloworld_pub") # 注册微信号
    rclpy.spin(node)                              # 一直挂着微信（不关机），等着闹钟叫我发消息
    rclpy.shutdown()                              # 手动退出时关闭微信

main()                                            # 启动！
"""
"""
import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类
from std_msgs.msg import String                  # ROS2标准定义的String消息

class SubscriberNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.sub = self.create_subscription(\
            String, "chatter", self.listener_callback, 10)        # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）

    def listener_callback(self, msg):                             # 创建回调函数，执行收到话题消息后对数据的处理
        self.get_logger().info('I heard: "%s"' % msg.data)        # 输出日志信息，提示订阅收到的话题消息
        
def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = SubscriberNode("topic_helloworld_sub")    # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
"""

