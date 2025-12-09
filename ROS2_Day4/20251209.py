#20251209 Day 12
#服务 service client
#客户端/服务器模型 同步通信
#Client
#编程接口初始化
#创建节点并初始化
#创建客户端对象
#创建并发送请求数据
#等待服务器端应答数据 
#销毁节点并关闭接口
import sys
import rclpy
from rclpy.node import Node
from learning_interface.srv import AddTwoInts

class AdderClient(Node):
    def __init__(self):
        super().__init__("service_adder_client")
        
        # 创建客户端 + 死等服务端上线
        self.client = self.create_client(AddTwoInts, "add_two_ints")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("服务端没好，再等1秒...")

        # 发请求（从命令行拿两个数）
        self.req = AddTwoInts.Request()
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.client.call_async(self.req)

def main():
    rclpy.init()
    node = AdderClient()

    # 一直等结果
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
                node.get_logger().info(f"结果：{node.req.a} + {node.req.b} = {response.sum}")
            except Exception as e:
                node.get_logger().info(f"服务调用失败：{e}")
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
class Client(Node):
    def __init__(self):
        super().__init__("节点名")
        self.cli = self.create_client(接口, "服务名")
        while not self.cli.wait_for_service(1.0):
            self.get_logger().info("等服务端...")

        req = 接口.Request()
        req.字段 = 值
        self.future = self.cli.call_async(req)

def main():
    rclpy.init()
    node = Client()
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            res = node.future.result()
            # 打印结果
            break
    rclpy.shutdown()
"""

#Service
#编程接口初始化 
#创建节点并初始化
#创建服务器端对象
#通过回调函数处进行服务
#向客户端反馈应答结果 
#销毁节点并关闭接口
import rclpy
from rclpy.node import Node
from learning_interface.srv import AddTwoInts

class AdderServer(Node):
    def __init__(self):
        super().__init__("service_adder_server")
        self.create_service(AddTwoInts, "add_two_ints", self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"收到请求：{request.a} + {request.b} = {response.sum}")
        return response

def main():
    rclpy.init()
    node = AdderServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
class Server(Node):
    def __init__(self):
        super().__init__("节点名")
        self.create_service(接口, "服务名", 回调函数)

    def 回调函数(self, request, response):
        # 填 response.xxx
        return response

def main():
    rclpy.init()
    node = Server()
    rclpy.spin(node)
    rclpy.shutdown()
    """