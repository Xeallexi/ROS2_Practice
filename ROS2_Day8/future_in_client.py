import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # 1. 创建客户端：我要去 'add_two_ints' 这个窗口
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        # 2. 等待服务端上线（标准写法，防止发给空气）
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务没上线，再等等...')
            
        # 3. 创建一张空白单子
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        # 4. 填单子
        self.req.a = a
        self.req.b = b
        
        # ==========================================
        # 👇 就是这一行！整个程序的灵魂！
        # ==========================================
        self.future = self.cli.call_async(self.req)
        
        # 把这个“凭证”交回给 main 函数去等
        return self.future

def main(args=None):
    rclpy.init(args=args)
    
    # 实例化节点
    client_node = MinimalClientAsync()
    
    # --- A. 动作：发射！ ---
    # 调用上面的 send_request，执行那句 call_async
    # 此时，请求飞出去了，手里拿到了 future (取餐号)
    future = client_node.send_request(4, 1)

    # --- B. 动作：死等！ ---
    # 只要 future 没完成，就一直卡在这里
    rclpy.spin_until_future_complete(client_node, future)

    # --- C. 动作：收货！ ---
    # 程序能走到这里，说明 B 结束了，结果到了
    try:
        response = future.result() # 打开凭证拿结果
        client_node.get_logger().info(f'计算结果是: {response.sum}')
    except Exception as e:
        client_node.get_logger().error(f'请求失败: {e}')

    client_node.destroy_node()
    rclpy.shutdown()
