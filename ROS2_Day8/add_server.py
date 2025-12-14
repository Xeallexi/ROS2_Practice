import rclpy
from rclpy.node import Node
# 👇 注意这里变了！我们要导入服务类型 AddTwoInts
# 它的结构是：
# int64 a
# int64 b
# ---
# int64 sum
from example_interfaces.srv import AddTwoInts
import time
class AddServer(Node):
    def __init__(self, name):
        super().__init__(name)
        # 1. 创建服务
        # 语法：create_service(服务类型, '服务名', 回调函数)
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.handle_add_two_ints)
        self.get_logger().info("天才数学家已上线，准备计算...")

    # 2. 回调函数 (核心)
    # 系统会把请求包在 request 里，把空信封包在 response 里给你
    def handle_add_two_ints(self, request, response):
        self.get_logger().info(f"收到请求 {request.a} + {request.b}，正在疯狂计算中...")
        time.sleep(5.0)
        # 干活：取出 a 和 b，相加
        result = request.a + request.b
        
        # 填表：把结果写进 response 的 sum 字段
        response.sum = result
        
        # 打印日志（方便调试）
        self.get_logger().info(f'收到请求: {request.a} + {request.b} = {result}')
        
        # 3. 必须把填好的信封退回去！
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddServer("add_server_node")
    rclpy.spin(node)
    rclpy.shutdown()

# ==============================================================================
# 📝 学习笔记：ROS2 服务端 (Service Server) 开发总结
# 📅 日期：2025年12月14日
# ==============================================================================
#
# 1. 核心逻辑 (Logic)
# - Service 是“一问一答”模式（Client发起 -> Server处理 -> Server回复）。
# - 区别于 Topic（话题）：Topic 是单向广播，Service 是双向同步交易。
# - 消息类型 (.srv)：比如 AddTwoInts，被 "---" 分为两部分：上层是 Request，下层是 Response。
#
# 2. 代码关键点 (Code)
# - 导包：需导入服务类型接口，如 `from example_interfaces.srv import AddTwoInts`。
# - 创建：`self.create_service(Type, 'name', callback)`。
# - 回调函数：必须接收两个参数 `(self, request, response)`。
#   - request: 客人填好的单子（读取数据，如 request.a）。
#   - response: 给客人的回执（写入结果，如 response.sum = ...）。
# - 返回值：回调函数最后**必须**写 `return response`，否则客户端会一直死等（卡死）。
#
# 3. 运行配置 (Setup)
# - setup.py：必须在 'console_scripts' 里添加入口：'命令名 = 包名.文件名:main'。
# - 语法陷阱：setup.py 里上一行代码末尾必须加逗号 `,`，否则编译报错。
# - 编译流程：修改完 python 文件或 setup.py 后，必须 `colcon build` 并 `source`。
#
# 4. 调试技巧 (Debug)
# - 命令行测试：不需要写客户端也能测服务端。
# - 指令：`ros2 service call /服务名 包名/srv/类型 "{参数: 值}"`
# - 示例：`ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"`
#
# ==============================================================================