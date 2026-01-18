# --- 1. 父类 Robot (刚才我们修正过的) ---
class Robot:
    def __init__(self, name: str):
        self.name = name
        self._battery = 100 

    def status(self):
        print(f"[{self.name}] 当前电量: {self._battery}%")

    def charge(self):
        self._battery = 100
        print(f"[{self.name}] 充电完毕！")

    # 封装电量属性
    @property
    def battery(self):
        return self._battery

    @battery.setter
    def battery(self, value):
        if value < 0:
            self._battery = 0
        elif value > 100:
            self._battery = 100
        else:
            self._battery = value

# --- 2. 子类 AGV (刚才修正过的) ---
class AGV(Robot):
    def __init__(self, name: str):
        super().__init__(name)
        self.position = [0, 0]

    def move_to(self, x, y):
        if self.battery < 10:
            print(f"[{self.name}] ⚠️ 电量不足 ({self.battery}%)，无法移动！")
            return

        self.battery -= 10
        self.position = [x, y]
        print(f"[{self.name}] 移动到了 {self.position}，剩余电量 {self.battery}%")

# --- 3. 子类 RoboticArm (你刚写好的！) ---
class RoboticArm(Robot):
    def __init__(self, name: str):
        super().__init__(name)
        self.payload = 0.0

    def grab(self, weight):
        if self.battery < 15:
            print(f"[{self.name}] ⚠️ 电量不足 ({self.battery}%)，无法抓取！")
            return
        
        self.battery -= 15
        self.payload = weight
        print(f"[{self.name}] 成功抓取了 {weight}kg 物体，剩余电量 {self.battery}%")

# ==========================================
# --- 4. 模拟工厂运行 (测试代码) ---
# ==========================================

print("=== 工厂启动 ===")
# 实例化
agv_1 = AGV("搬运工-01")
arm_1 = RoboticArm("大力士-01")

# 正常工作
agv_1.move_to(10, 20)
arm_1.grab(50.0)

print("\n=== 疯狂加班模式 ===")
# 让机械臂连续工作，测试没电的情况
for i in range(6):
    print(f"第 {i+1} 次尝试抓取...")
    arm_1.grab(10.0)

print("\n=== 维护阶段 ===")
# 充电
arm_1.charge()
arm_1.status()
arm_1.grab(20.0) # 充电后应该又能抓了