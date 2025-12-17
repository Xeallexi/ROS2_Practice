# 注意：是从 包名.msg 导入 文件名
from tutorial_interfaces.msg import Student

# 使用方法：
msg = Student()
msg.name = "张三"
msg.age = 20
msg.score = 100
"""

# 📝 ROS2 接口 (Interface) 极简总结

## 1. 核心概念：它是什么？

* **比喻**：它不是干活的工人（那是代码），它是工人之间填写的**“标准单据”**。
* **作用**：为了让 Python 节点和 C++ 节点能互相看懂对方的数据。
* **本质**：一个规定了数据类型的**文件**（`.msg`, `.srv`）。

## 2. 三大家族 (Types)

| 后缀 | 名字 | 场景 | 比喻 | 关键特征 |
| --- | --- | --- | --- | --- |
| **.msg** | 消息 (Message) | 话题 (Topic) | **广播 / 报纸** | 单向发，不回嘴。 |
| **.srv** | 服务 (Service) | 服务 (Service) | **银行柜台** | 一问一答 (`---` 分隔)。 |
| **.action** | 动作 (Action) | 动作 (Action) | **网约车** | 长期任务，有进度反馈。 |

## 3. 怎么造一个新接口？(5步流程)

这是你刚才觉得最难的地方。**记住：这是固定套路，死记硬背即可。**

1. **建包**：必须用 `ament_cmake` 类型。
```bash
ros2 pkg create --build-type ament_cmake my_interfaces

```


2. **写文件**：在 `msg` 文件夹里新建文本文件（如 `Student.msg`）。
* **注意**：千万别在终端里直接敲内容！ 要用编辑器 (`nano` 或 VS Code) 写进去保存。
* **内容格式**：`类型 变量名` (如 `int32 age`)。


3. **改 CMakeLists.txt**：告诉机器要把这个文件“印刷”出来。
* 加 `find_package(rosidl_default_generators REQUIRED)`
* 加 `rosidl_generate_interfaces(...)`


4. **改 package.xml**：声明依赖。
* 加 `build_depend`, `exec_depend`, `member_of_group` 那三行。


5. **编译与刷新**：
```bash
colcon build ...  # (编译)
source install/setup.bash # (最重要！不刷这就找不到)

```



## 4. 怎么在 Python 里用？

一旦编译成功，它就变成了一个普通的 Python 模块。

```python
# 格式：from 包名.msg import 文件名
from my_interfaces.msg import Student 

msg = Student()       # 1. 拿单子
msg.name = "Gemini"   # 2. 填单子
pub.publish(msg)      # 3. 交单子

```

## 5. 新手必坑指南 (避雷针) ⚡️

* **坑 1：VS Code 划红线报错。**
* **真相**：它是瞎报的。只要终端能运行，就别理它。


* **坑 2：运行报错 `ModuleNotFoundError`。**
* **真相**：你编译完忘了输入 `source install/setup.bash`。每次开新终端都要输！


* **坑 3：不知道变量名叫 `data` 还是 `value`。**
* **解法**：永远不要猜。先运行 `ros2 interface show <接口名>` 看一眼。
"""