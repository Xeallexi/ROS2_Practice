from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. 获取参数变量 (占位符) ---
    target_turtle = LaunchConfiguration('target_turtle')

    # --- 2. 声明参数 (对外接口) ---
    # 允许运行：ros2 launch ... target_turtle:=t2
    target_turtle_arg = DeclareLaunchArgument(
        'target_turtle',
        default_value='t1',
        description='你要控制哪只乌龟？(t1 或 t2)'
    )

    # --- 3. 定义节点 ---
    # 乌龟 1
    turtle1 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        namespace='t1',
        name='sim',
    )

    # 乌龟 2
    turtle2 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        namespace='t2',
        name='sim',
    )

    # 键盘控制节点
    teleop_node = Node(
        package='turtlesim',
        executable='turtle_teleop_key',
        name='teleop',
        prefix='xterm -e', # 在新终端弹出
        
        # --- 4. 话题重映射 (关键) ---
        remappings=[
            # 原理：把节点原本想发的 '/turtle1/cmd_vel' 
            # 偷梁换柱成动态路径： '/<target_turtle>/turtle1/cmd_vel'
            # 注意：拼接变量时必须用列表 []
            ('/turtle1/cmd_vel', ['/', target_turtle, '/turtle1/cmd_vel'])
        ]
    )

    return LaunchDescription([
        target_turtle_arg,
        turtle1,
        turtle2,
        teleop_node
    ])

# =======================================================================
# ROS2 Launch 核心知识点总结 (Cheat Sheet) - 学习笔记
# =======================================================================
#
# 1. 核心模块 (Imports)
#    - Node: 定义要启动的节点 (package, executable, name)。
#    - LaunchDescription: 返回给 ROS2 系统的“任务清单”。
#    - DeclareLaunchArgument: 定义外部参数 (default_value, description)。
#    - LaunchConfiguration: 获取参数的值 (作为变量在内部使用)。
#
# 2. 常用参数说明
#    - namespace='xxx': 给节点加前缀，防止重名，实现隔离 (如 /t1/sim)。
#    - prefix='xterm -e': 让节点在新的终端窗口运行 (适合交互式节点)。
#    - remappings=[('旧名', '新名')]: 修改话题线路，实现节点间的“重连”。
#
# 3. 动态拼接技巧
#    - 在 Launch 文件中，不能直接用 "str" + variable 拼接。
#    - 必须使用列表形式: ['前缀字符串', 变量对象, '后缀字符串']。
#    - ROS2 会在运行时自动把它们解析并拼成一个完整的字符串。
#
# 4. 编译与运行
#    - 编译: colcon build --symlink-install (修改 Python 后无需重编译)。
#    - 运行: ros2 launch <包名> <文件名> [参数名:=值]。
#    - 查错: 记得在 setup.py 里把 launch 文件 install 到 share 目录！
#
# =======================================================================