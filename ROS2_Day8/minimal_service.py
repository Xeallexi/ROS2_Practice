import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        # 标准写法：create_service(类型, '服务名', 回调)
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('服务端就绪，等待请求...')

    def add_two_ints_callback(self, request, response):
        # 1. 处理逻辑
        response.sum = request.a + request.b
        # 2. 打印日志 (官方推荐在这里打Log)
        self.get_logger().info(f'收到请求: {request.a} + {request.b}')
        # 3. 必须返回 response
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MinimalService()
    rclpy.spin(node)
    rclpy.shutdown()