from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub",
        arguments=[
            '--x', '0.5',
            '--y', '0.0',
            '--z', '0.2',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_link']

    )
    return LaunchDescription([
        static_tf
    ])
# =======================================================================
# ROS2 静态 TF2 (Static Transform) 学习总结 (Cheat Sheet)
# =======================================================================
#
# 1. 核心概念
#    - 静态 TF (Static TF): 用于描述机器人的“固连”部件（如：用螺丝把雷达拧在车顶）。
#      既然是拧死的，它们之间的相对位置(xyz)和角度(rpy)就是永远不变的。
#    - 节点程序: static_transform_publisher (属于 tf2_ros 包)。
#
# 2. 坐标系口诀 (视觉铁律)
#    - 🔴 Red (红)   = X 轴 (前方)
#    - 🟢 Green (绿) = Y 轴 (左方)
#    - 🔵 Blue (蓝)  = Z 轴 (上方)
#    - 右手定则: 大拇指(Z)指天, 食指(X)指前, 中指(Y)指左。
#
# 3. Launch 文件写法 (关键知识点)
#    - 为什么刚才报错？
#      新版 ROS2 (如 Iron/Jazzy) 对参数检查变严了，旧版的纯数字写法 (x y z ...) 
#      容易被系统当成无法解析的垃圾参数。
#    - 正确写法 (带标签):
#      arguments=[
#          '--x', '0.5',          # 前后距离 (米)
#          '--y', '0.0',          # 左右距离
#          '--z', '0.2',          # 上下高度
#          '--yaw', '0',          # 旋转 (也可以用 --qx --qy... 四元数)
#          '--pitch', '0',
#          '--roll', '0',
#          '--frame-id', 'base_link',       # 父坐标系 (爸爸/基准)
#          '--child-frame-id', 'laser_link' # 子坐标系 (儿子/传感器)
#      ]
#
# 4. 验证方法 (Rviz2)
#    - 必须设置: Global Options -> Fixed Frame -> 改为 'base_link'。
#    - 必须添加: 点击 Add -> 选择 TF。
#    - 只要能看到两个坐标轴被一根黄线连着，就是成功了。
#
# =======================================================================