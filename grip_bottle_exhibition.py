from robotic_arm_package.robotic_arm import *
from roh_registers import *


ARM_IP = "192.168.1.18"

COM_PORT = 1
ROH_ADDR = 2


def L_BYTE(v):
    return v & 0xFF


def H_BYTE(v):
    return (v >> 8) & 0xFF


robot = Arm(RM65, ARM_IP)

if __name__ == "__main__":
    joint0 = [0, 0, 0, 0, 0, 0]
    joint1 = [81.739, -66.847, -110.628, 0.692, 87.338, 62.775] # 初始抓取关节角度
    pose0 = [0.057, 0.384, 0.131, -1.573, -0.474, -0.155] # 中间初始位置

    pose1 = [0.037, 0.494, 0.101, -1.573, -0.474, -0.055] # 抓中间
    pose2 = [0.037, 0.494, 0.151, -1.573, -0.474, -0.155] # 抬起
    pose3 = [0.2145, 0.494, 0.151, -1.573, -0.474, -0.155] # 抬起移到右边
    pose4 = [0.2145, 0.494, 0.101, -1.573, -0.474, -0.055]  # 放下右边
    pose5 = [0.2145, 0.344, 0.101, -1.573, -0.474, -0.155] # 右边往后抽
    pose6 = [-0.1205, 0.344, 0.101, -1.573, -0.474, -0.155] # 瞄准左边
    pose7 = [-0.1405, 0.494, 0.101, -1.573, -0.474, -0.055] # 抓左边
    pose8 = [-0.1405, 0.494, 0.151, -1.573, -0.474, -0.155] # 左边抬起
    pose9 = [0.037, 0.494, 0.151, -1.573, -0.474, -0.155] # 抬起移到中间，放下为pose1
    pose10 = [0.037, 0.344, 0.101, -1.573, -0.474, -0.155]  # 中间往后抽
    pose11 = [0.2145, 0.344, 0.101, -1.573, -0.474, -0.155]  # 瞄准右边，抓右边为pose4
    pose12 = [0.2145, 0.494, 0.151, -1.573, -0.474, -0.155]  # 右边抬起
    pose13 = [0.2145, 0.344, 0.151, -1.573, -0.474, -0.155] # 右边抬起往后抽
    pose14 = [-0.1405, 0.344, 0.151, -1.573, -0.474, -0.155] # 抬起瞄准左边，移到左边为pose8
    pose15 = [-0.1405, 0.494, 0.101, -1.573, -0.474, -0.155] # 左边放下
    pose16 = [-0.1405, 0.364, 0.101, -1.573, -0.474, -0.155]  # 左边往后抽
    pose17 = [0.057, 0.344, 0.101, -1.573, -0.474, -0.155]  # 移到中间

    robot.Close_Modbustcp_Mode()

    robot.Set_Modbus_Mode(1, 115200, 1, True)

    robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
                          [
                              H_BYTE(0), L_BYTE(0),
                              H_BYTE(0), L_BYTE(0),
                              H_BYTE(0), L_BYTE(0),
                              H_BYTE(0), L_BYTE(0),
                              H_BYTE(0), L_BYTE(0),
                              H_BYTE(0), L_BYTE(0),
                          ],
                          ROH_ADDR,
                          True,
                          )
    ret = robot.Movej_Cmd(joint0, 50, 0, True)

    ret = robot.Movej_Cmd(joint1, 30, 0, True)  # 初始抓取关节角度



    counter = 0

    while counter < 3:


        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(65535), L_BYTE(65535),
            ],
            ROH_ADDR,
       True,
        )
        time.sleep(1)
        
        ret = robot.Movel_Cmd(pose1, 50, 0, True)# 抓中间

        robot.Write_Registers(COM_PORT,ROH_FINGER_POS_TARGET0,6,
            [
                H_BYTE(8000), L_BYTE(8000),
                H_BYTE(23000),L_BYTE(23000),
                H_BYTE(25000),L_BYTE(25000),
                H_BYTE(22000),L_BYTE(22000),
                H_BYTE(15000),L_BYTE(15000),
                H_BYTE(65535),L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)
 
        ret = robot.Movel_Cmd(pose2, 50, 0, True) # 中间抬起
        ret = robot.Movel_Cmd(pose3, 50, 0, True) # 抬起移到右边
        ret = robot.Movel_Cmd(pose4, 50, 0, True) # 放下右边

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(65535), L_BYTE(65535),
            ],
            ROH_ADDR,
           True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose5, 50, 0, True) # 右边往后抽

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
            ],
            ROH_ADDR,
           True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose6, 50, 0, True) # 瞄准左边

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(65535), L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose7, 50, 0, True) # 抓左边

        robot.Write_Registers(COM_PORT,ROH_FINGER_POS_TARGET0,6,
        [
            H_BYTE(8000), L_BYTE(8000),
            H_BYTE(23000), L_BYTE(23000),
            H_BYTE(25000), L_BYTE(25000),
            H_BYTE(22000), L_BYTE(22000),
            H_BYTE(15000), L_BYTE(15000),
            H_BYTE(65535), L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose8, 50, 0, True) # 左边抬起
        ret = robot.Movel_Cmd(pose9, 50, 0, True) # 抬起移到中间
        ret = robot.Movel_Cmd(pose1, 50, 0, True) # 中间放下

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(65535), L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose10, 50, 0, True) # 中间往后抽

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
                [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose11, 50, 0, True) # 瞄准右边

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(65535), L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose4, 50, 0, True) # 抓右边

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
            H_BYTE(8000), L_BYTE(8000),
            H_BYTE(23000), L_BYTE(23000),
            H_BYTE(25000), L_BYTE(25000),
            H_BYTE(22000), L_BYTE(22000),
            H_BYTE(15000), L_BYTE(15000),
            H_BYTE(65535), L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose12, 50, 0, True) # 右边抬起
        ret = robot.Movel_Cmd(pose13, 50, 0, True) # 右边抬起往后抽
        ret = robot.Movel_Cmd(pose14, 50, 0, True) # 抬起瞄准左边
        ret = robot.Movel_Cmd(pose8, 50, 0, True) # 抬起移到左边
        ret = robot.Movel_Cmd(pose15, 50, 0, True) # 左边放下

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(65535), L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose16, 50, 0, True) # 左边往后抽

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose17, 50, 0, True) # 移到中间


        counter += 1
        print("已运行：", counter, "次")

        # if counter == 5:
        #     sys.exit()  # 立即停止程序，返回状态码 0
