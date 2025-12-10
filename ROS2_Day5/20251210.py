"""
class Hero:
    def __init__(self,name,hp):
        self.name = name
        self.hp = hp
    def take_damage(self):
        self.hp -= 10
        print(f"{self.name}掉血了，剩余血量{self.hp}")
hero1 = Hero("亚瑟",100)
hero2 = Hero("鲁班",80)
hero1.take_damage()
hero2.take_damage()
hero1.take_damage()

class PiggyBank():
    def __init__(self):
        self.money = 0
    def add(self,amount):
        self.money += amount
        print(f"现在存入了{amount},现在总共有{self.money}")
    def minus(self,amount):
        self.money -= amount
        print(f"现在取出了{amount}，现在总共有{self.money}")    
        
my_bank = PiggyBank()
your_bank = PiggyBank()
big_bank = PiggyBank()
big_bank.add(10)
big_bank.add(10)
big_bank.add(10)
big_bank.minus(10)
your_bank.add(100)
your_bank.minus(50)
1. 类（Class）与 实例（Instance）
你的理解：类是图纸/大类，实例是真机/具体的例子。

核心逻辑：

class PiggyBank: 只是在纸上画了个存钱罐（不能存钱）。

my_bank = PiggyBank() 才是真正根据图纸造出了一个实物（可以存钱）。

ROS2 映射：代码里的 class MyNode 是图纸，你运行起来的那个节点才是干活的真机。

2. self（自我/我的）
你的理解：self 相当于调用这个例子里的功能，无论谁用都能用，但数据只属于他自己。

核心逻辑：

独立性：my_bank 和 your_bank 虽然用的是同一套代码（add 方法），但 self 保证了大家的钱（self.money）互不干扰。

记忆力：self 能把数据“存住”。你在 add 里算出的结果，下次再调用 minus 时还能记得住。

ROS2 映射：机器人的电量、坐标、计数器，都要挂在 self 下面，否则函数运行完数据就丢了。

3. __init__（初始化）
你的理解：相当于初始化，出厂设置。

核心逻辑：

它在“造真机”的那一瞬间自动运行。

它的作用是把该有的东西（如 self.money = 0）准备好。如果不写它，直接去存钱，程序就会报错说“找不到钱包”。

ROS2 映射：ROS2 节点一启动，就要在 __init__ 里把发布者、订阅者这些“工具”都准备好。

4. 方法（Method）
你的理解：比如 add 和 minus，是进入游戏后的技能键。

核心逻辑：

捏脸（Init）只能做一次，但技能（Method）想按几次按几次。

技能里通过 self.xxx 来修改对象的状态。
"""
class AirConditioner:
    def __init__(self):
        self.teperature = 26
        print("空调已安装，默认26度")
    def cool_down(self,vol):
        self.teperature -= vol
        print(f"正在制冷...当前温度降到了 {self.teperature} 度")
    def heat_hp(self,vol):
        self.teperature += vol
        print(f"正在制热...当前温度升到了 {self.teperature} 度")
bedroom_ac = AirConditioner()
living_room_ac = AirConditioner()
bedroom_ac.cool_down(3)
living_room_ac.heat_hp(4)
bedroom_ac.cool_down(1)