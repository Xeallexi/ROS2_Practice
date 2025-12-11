"""
class Father (object):
    def __init__(self):
        self.surname = "王"
        print("父类初始化完成")
    def skill(self):
        print("我会木工活")
class Son(Father):
    def __init__(self,name):
        super().__init__()
        self.name = name
        print(f"儿子{self.name}")
    def play_games(self):
        print("我会打游戏")
xiaowang = Son("小王")
xiaowang.skill()
print(xiaowang.surname)

class Node(object):
    def __init__(self,name):
        self.name = name
        print(f"ROS{name}初始化成功")
    def log(self,msg):
        print(f"{msg}")
class MyRobot(Node):
    def __init__(self, name):
        super().__init__(name)
        self.log(f"我是{name}我启动了")
bot = MyRobot("瓦力")
bot1 = MyRobot("dabai")
"""
class Bicycle(object):
    def __init__(self):
        self.speed = 0
        print("一辆普通自行车做好了")
    def pedal(self):
        self.speed += 5
        print(f"蹬了一脚，当前速度：{self.speed}")
class EBike(Bicycle):
    def __init__(self):
        super().__init__()
        battery = 100
        self.battery = battery
        print("并且装上了电池，升级为电动车")
    def electric_run(self):
         if self.battery > 0:
             self.speed += 20
             self.battery -=50
             print(f"电门加速,速度飙升至{self.speed}，剩余电量{self.battery}")
         else:
             print("没电了")
my_bike = EBike()
my_bike.electric_run()
my_bike.pedal()
my_bike.electric_run()