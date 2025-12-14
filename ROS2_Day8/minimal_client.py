import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # 1. 创建客户端
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # 2. 循环等待服务上线 (官方标准写法)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务未上线，正在等待...')
        # 3. 预先创建请求对象
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # 4. 异步发送，返回 future (取餐号)
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    node = MinimalClientAsync()

    # --- 核心三步走 ---
    # 1. 发送请求
    future = node.send_request(4, 1)

    # 2. 阻塞等待结果 (Spin until complete)
    rclpy.spin_until_future_complete(node, future)

    # 3. 获取结果
    # (此处省略 try...except 是为了简洁，实际上生产环境建议加)
    response = future.result()
    node.get_logger().info(f'计算结果: {response.sum}')
    # ----------------

    node.destroy_node()
    rclpy.shutdown()