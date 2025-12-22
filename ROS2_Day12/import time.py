import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# 1. 导入接口
# 注意：这里导入的是我们在 dishwasher_interfaces 包里定义的 Action
from dishwasher_interfaces.action import WashDishes

class DishwasherActionServer(Node):

    def __init__(self):
        super().__init__('dishwasher_server_node') # 节点名
        
        # 2. 创建 Action Server 对象
        self._action_server = ActionServer(
            self,
            WashDishes,              # 接口类型
            'wash_dishes',           # Action 名字 (话题名)
            self.execute_callback    # 回调函数
        )
        self.get_logger().info('洗碗机器人(Server)已启动，等待订单...')

    def execute_callback(self, goal_handle):
        """
        核心逻辑：当有人发动作请求时，会触发这个函数
        """
        self.get_logger().info('收到洗碗订单！准备开始...')

        # --- A. 解析 Goal ---
        # goal_handle.request 包含了 .action 文件中 Goal 部分定义的数据
        target_dish_count = goal_handle.request.dish_id # 我们把 dish_id 当作数量来用
        
        # --- B. 初始化 Feedback ---
        feedback_msg = WashDishes.Feedback()
        
        # --- C. 初始化 Result ---
        result = WashDishes.Result()
        result.dishes_washed = []

        # --- D. 执行循环 (模拟耗时任务) ---
        for i in range(1, target_dish_count + 1):
            # 1. 模拟因为某些原因需要取消任务 (健壮性检查)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('任务被取消！')
                return WashDishes.Result()

            # 2. 模拟工作耗时 (1秒洗一个)
            time.sleep(1.0)
            
            # 3. 填充 Feedback 数据
            # 计算进度百分比
            feedback_msg.progress = (float(i) / target_dish_count) * 100.0
            
            # 4. 发布 Feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'正在洗第 {i} 个碗，进度: {feedback_msg.progress}%')
            
            # 5. 记录这一步的结果到 Result 列表里
            result.dishes_washed.append(f"碗_{i}")

        # --- E. 任务结束 ---
        # 标记当前目标为“成功”
        goal_handle.succeed()
        
        self.get_logger().info('所有碗清洗完毕，任务成功！')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = DishwasherActionServer()
    rclpy.spin(node) # 保持节点运行
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    # ==========================================
# 📅 学习记录：2025/12/22 - ROS2 Action (动作)
# ==========================================
#
# 1. 核心概念：为什么需要 Action？
# ------------------------------------------
# - 场景：适用于耗时较长、过程复杂、需要知道进度、或者可能中途取消的任务。
# - 核心模型 ("网约车模式")：
#   (1) Goal (目标): 客户端下单。
#   (2) Feedback (反馈): 服务端周期性汇报进度 (异步)。
#   (3) Result (结果): 任务结束时的最终状态 (成功/失败/取消)。
#   (4) Cancel (取消): 客户端拥有随时中断任务的权利。
#
# 2. 接口定义 (.action 三段式)
# ------------------------------------------
# int32 order_count      # Goal: 目标参数 (Client -> Server)
# ---
# string[] result_list   # Result: 最终结果 (Server -> Client)
# ---
# float32 progress       # Feedback: 实时反馈 (Server -> Client)
#
# 3. 标准开发流程 (双包架构)
# ------------------------------------------
# [步骤 A] 接口包 (dishwasher_interfaces) - C++编译类型
# - 作用：专门定义数据结构，生成 Python/C++ 都能用的依赖文件。
# - 关键：CMakeLists.txt 中必须包含 rosidl_generate_interfaces。
#
# [步骤 B] 代码包 (dishwasher_py) - Python编译类型
# - 作用：编写业务逻辑。
# - 依赖：必须在 package.xml 中依赖接口包 (dishwasher_interfaces)。
#
# 4. 重点避坑指南 (Debug 经验)
# ------------------------------------------
# [坑1] 文件结构嵌套 (ModuleNotFoundError 的元凶)
# - 错误：src/包名/代码.py
# - 正确：src/包名/包名/代码.py (必须有内层文件夹和 __init__.py)
#
# [坑2] 环境变量失效 (Package not found 的元凶)
# - 编译后系统不会自动刷新，必须手动执行：source install/setup.bash
# - 只有回到工作空间根目录 (dev_ws) 编译才是安全的。
#
# [坑3] 逻辑闭环
# - 任务结束必须调用 goal_handle.succeed() 并返回 Result 对象，
# - 否则客户端会一直处于“等待”状态。
# ==========================================