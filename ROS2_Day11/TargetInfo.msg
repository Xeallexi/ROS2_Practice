string target_name
float64 x
float64 y
bool is_reachable
# ==============================================================================
# [ROS2 备忘录] 自定义接口 (Interface) 搭建全流程 SOP
# ==============================================================================
#
# 1. 终端指令：创建骨架
# ------------------------------------------------------------------------------
# cd ~/ros2_ws/src
# # 注意：接口包必须用 ament_cmake，即使主要用 Python 开发
# ros2 pkg create --build-type ament_cmake <包名_interfaces>
# cd <包名_interfaces>
# mkdir msg
# touch msg/<消息名>.msg  (注意：文件名首字母大写，如 TargetInfo.msg)
#
# 2. 编写消息内容 (在 .msg 文件中)
# ------------------------------------------------------------------------------
# string target_name
# float64 x
# float64 y
# bool is_reachable
#
# 3. 修改 package.xml (如果不加 member_of_group 会报错！)
# ------------------------------------------------------------------------------
# <build_depend>rosidl_default_generators</build_depend>
# <exec_depend>rosidl_default_runtime</exec_depend>
# <member_of_group>rosidl_interface_packages</member_of_group>
#
# 4. 修改 CMakeLists.txt (三步走)
# ------------------------------------------------------------------------------
# # A. 找工具 (放在 find_package ament_cmake 下面)
# find_package(rosidl_default_generators REQUIRED)
#
# # B. 生成代码 (注意路径要带 msg/)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/<消息名>.msg"
# )
#
# # C. 导出依赖 (放在 ament_package() 之前)
# ament_export_dependencies(rosidl_default_runtime)
#
# 5. 编译与生效
# ------------------------------------------------------------------------------
# cd ~/ros2_ws
# colcon build --packages-select <包名_interfaces>
# source install/setup.bash
#
# 6. 验证指令
# ------------------------------------------------------------------------------
# ros2 interface show <包名_interfaces>/msg/<消息名>
#
# 7. 在 Python 节点中如何引用？
# ------------------------------------------------------------------------------
# from <包名_interfaces>.msg import <消息名>
# msg = <消息名>()
# msg.x = 1.0
# ==============================================================================