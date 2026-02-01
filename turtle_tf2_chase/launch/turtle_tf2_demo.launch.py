from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动模拟器
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # 2. 启动 turtle1 的广播员 (Target)
        Node(
            package='turtle_tf2_chase',
            executable='broadcast',
            name='broadcaster1',
            parameters=[{'turtlename': 'turtle1'}]
        ),
        # 3. 启动 turtle2 的广播员 (Source) - 必须要有，不然无法计算相对坐标
        Node(
            package='turtle_tf2_chase',
            executable='broadcast',
            name='broadcaster2',
            parameters=[{'turtlename': 'turtle2'}]
        ),
        # 4. 启动监听者 (追逐者)
        Node(
            package='turtle_tf2_chase',
            executable='listener',
            name='listener',
            parameters=[{'target_frame': 'turtle1'}]
        ),
    ])