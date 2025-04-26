import sys
import os
import time

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)
sys.path.append(os.path.join(ROOT_DIR, "Python"))
sys.path.append(os.path.join(ROOT_DIR, "Python", "Robotic_Arm"))
from Robotic_Arm.rm_robot_interface import *


class Rqgripper():

    def __init__(self):
        # 实例化RoboticArm类
        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)

        # 创建机械臂连接，打印连接id
        self.handle = self.arm.rm_create_robot_arm("192.168.1.18", 8080)
        print(self.arm.rm_close_modbus_mode(1))
        print(self.arm.rm_set_modbus_mode(1, 115200, 5))
        self.write_params = rm_peripheral_read_write_params_t(1, 1000, 9, 3)
        self.read_params = rm_peripheral_read_write_params_t(1, 2000, 9, 3)

    def activate(self):
        print(self.arm.rm_write_registers(self.write_params, [0, 0, 0, 0, 0, 0]))
        print(self.arm.rm_write_registers(self.write_params, [1, 0, 0, 0, 0, 0]))


    def goToPosition_once(self, position, speed = 0, force = 0 ):

        # print(self.arm.rm_write_registers(self.write_params, [9, 0, 0, 0, 0, 0]))
        return self.arm.rm_write_registers(self.write_params, [9, 0, 0, position, speed, force])


    def goToPosition(self, position, speed = 0, force = 0 ):

        self.arm.rm_write_registers(self.write_params, [9, 0, 0, position, speed, force])

        while not (self.getPosition() >= position):
            continue

        return 0



    def getPosition(self):
        data = self.arm.rm_read_multiple_input_registers(self.read_params)

        return data[1][4]

    def getActionstatus(self):

        data = self.arm.rm_read_multiple_input_registers(self.read_params)

        byte_value = data[1][0]

        # 确保输入在0-255范围内
        if byte_value < 0 or byte_value > 255:
            raise ValueError("Input must be between 0 and 255")

        # 右移3位并与1进行与运算，获取gGTO位
        gGTO_bit = (byte_value >> 3) & 1

        return gGTO_bit



    def stop(self):

        self.arm.rm_write_registers(self.write_params, [1, 0, 0, 0, 0, 0])


    def delete(self):
        self.arm.rm_delete_robot_arm()


def main():
    gripper = Rqgripper()
    # gripper.activate()  # Example of activating the gripper
    gripper.goToPosition(128)
    data = gripper.getPosition()
    print(data)
    # time.sleep(4)
    data = gripper.getPosition()
    print(data)


    gripper.goToPosition(5)

    while True:

        pos = gripper.getPosition()

        gripper.goToPosition(pos + 15)



if __name__ == "__main__":
    gripper = Rqgripper()
    gripper.activate()
    gripper.goToPosition_once(150)