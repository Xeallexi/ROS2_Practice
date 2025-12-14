# 1. 进货：拿 ROS2 核心库、节点基类、服务类型
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # 提示：还是用那个 AddTwoInts，虽然我们要算乘法

# 2. 定义类：继承 Node
class AreaServer(Node):
    
    def __init__(self, name):
        super().__init__(name)
        
        # 3. 开张：创建一个服务
        # 语法：self.create_service(类型, '服务名', 回调函数)
        # 咱们给服务起名叫 'calculate_area'
        self.srv = self.create_service(AddTwoInts, 'area_calculate', self.handle_area_calc)
        
        self.get_logger().info("面积计算服务已启动...")

    # 4. 干活：回调函数
    # 必须接住两个东西：客人的请求(request)，回信的信封(response)
    def handle_area_calc(self, request, response):
        # 逻辑：长(a) 乘以 宽(b)
        area = request.a * request.b
        
        # 填表：把结果塞进 response 的 sum 字段里 (虽然叫sum，但我们可以存乘积)
        response.sum = area
        
        self.get_logger().info(f'收到长宽: {request.a}, {request.b}, 计算面积为: {area}')
        
        # 5. 关键动作：必须把信封退回去！
        return response

# 6. 通电
def main(args=None):
    rclpy.init(args=args)
    node = AreaServer("area_server_node")
    rclpy.spin(node)
    rclpy.shutdown()