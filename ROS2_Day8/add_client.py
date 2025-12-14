import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import time # 记得导入 time

class AddClient(Node):
    def __init__(self, name):
        super().__init__(name)
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务没上线...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    node = AddClient("add_client_node")
    
    # 1. 发出请求
    future = node.send_request(3, 4)
    print("----------------------------------")
    print("【Client】: 请求已发送，我要开始去玩手机了！")
    print("----------------------------------")

    # 2. 核心实验代码：只要结果没回来(not done)，我就一直玩
    while not future.done():
        # 这里模拟“做其他事”
        print("【Client】: 玩手机中... Server 还没回话...")
        
        # ⚠️ 关键动作：虽然我在玩，但我每隔0.5秒要瞄一眼邮箱（处理回调）
        # 如果不写 spin_once，就算结果回来了，Client 也收不到！
        rclpy.spin_once(node, timeout_sec=0)
        
        time.sleep(1.0) # 每秒喊一次

    # 3. 循环结束，说明结果回来了
    print("----------------------------------")
    print("【Client】: 居然算好了！放下手机看结果。")
    try:
        response = future.result()
        node.get_logger().info(f'最终计算结果: {response.sum}')
    except Exception as e:
        node.get_logger().error(f'失败: {e}')
    
    node.destroy_node()
    rclpy.shutdown()