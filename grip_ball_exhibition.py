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
    joint1 = [89.568, -54.037, -129.78, 5.377, 93.872, -28.31] # 初始抓取关节角度
    pose0 = [0.016, 0.334, 0.181, 1.568, -1.07, 3.042] # 初始位置
    pose1 = [0.016, 0.516, 0.181, 2.252, -0.901, 2.468] # 移到中间
    pose2 = [0.016, 0.516, 0.141, 2.252, -0.901, 2.468] # 抓中间

    pose3 = [0.016, 0.516, 0.221, 2.252, -0.901, 2.468] # 抬起
    pose4 = [0.1935, 0.516, 0.221, 2.252, -0.901, 2.468] # 抬起移到右边
    pose5 = [0.1935, 0.516, 0.141, 2.252, -0.901, 2.468]  # 放下右边
    pose6 = [0.1935, 0.516, 0.181, 2.252, -0.901, 2.468] # 右边抬起
    pose7 = [0.1935, 0.334, 0.181, 1.568, -1.07, 3.042] # 右边往后抽

    pose8 = [-0.1615, 0.334, 0.181, 1.568, -1.07, 3.042] # 移到左边
    pose9 = [-0.1615, 0.516, 0.141, 2.268, -0.901, 2.448] # 抓左边
    pose10 = [-0.1615, 0.516, 0.221, 2.268, -0.901, 2.448] # 左边抬起
    pose11 = [0.016, 0.516, 0.221, 2.268, -0.901, 2.448] # 抬起移到中间，放下为pose2
    pose12 = [0.016, 0.516, 0.181, 2.252, -0.901, 2.468]  # 中间抬起，往后抽为pose0
    pose13 = [0.1935, 0.334, 0.181, 1.568, -1.07, 3.042]  # 移到右边，抓右边为pose5
    pose14 = [0.1935, 0.516, 0.241, 2.268, -0.901, 2.448]  # 右边抓抬起
    pose15 = [-0.1615, 0.516, 0.241, 2.268, -0.901, 2.448] # 右边抓抬起移到左边，放下为pose9
    pose16 = [-0.1615, 0.516, 0.181, 2.268, -0.901, 2.448] # 左边抬起，移到中间为pose0


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
                H_BYTE(2000), L_BYTE(2000),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(65535), L_BYTE(65535),
            ],
            ROH_ADDR,
       True,
        )

        ret = robot.Movel_Cmd(pose1, 50, 0, True)# 移到中间
        ret = robot.Movel_Cmd(pose2, 50, 0, True)  # 抓中间

        robot.Write_Registers(COM_PORT,ROH_FINGER_POS_TARGET0,6,
            [
                H_BYTE(8000), L_BYTE(8000),
                H_BYTE(12000),L_BYTE(12000),
                H_BYTE(14000),L_BYTE(14000),
                H_BYTE(0),L_BYTE(0),
                H_BYTE(0),L_BYTE(0),
                H_BYTE(65535),L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose3, 50, 0, True) # 中间抬起
        ret = robot.Movel_Cmd(pose4, 50, 0, True) # 抬起移到右边
        ret = robot.Movel_Cmd(pose5, 50, 0, True) # 放下右边
    #
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


        ret = robot.Movel_Cmd(pose6, 50, 0, True) # 右边抬起

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

        ret = robot.Movel_Cmd(pose7, 50, 0, True)  # 右边往后抽
        ret = robot.Movel_Cmd(pose8, 50, 0, True)  # 移到左边

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(2000), L_BYTE(2000),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(65535), L_BYTE(65535),
            ],
            ROH_ADDR,
           True,
        )

        ret = robot.Movel_Cmd(pose9, 50, 0, True)  # 抓左边

        robot.Write_Registers(COM_PORT,ROH_FINGER_POS_TARGET0,6,
            [
                H_BYTE(8000), L_BYTE(8000),
                H_BYTE(12000),L_BYTE(12000),
                H_BYTE(14000),L_BYTE(14000),
                H_BYTE(0),L_BYTE(0),
                H_BYTE(0),L_BYTE(0),
                H_BYTE(65535),L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose10, 50, 0, True)  # 左边抬起
        ret = robot.Movel_Cmd(pose11, 50, 0, True)  # 移到中间
        ret = robot.Movel_Cmd(pose2, 50, 0, True)  # 中间放下

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

        ret = robot.Movel_Cmd(pose12, 50, 0, True)  # 中间抬起

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

        ret = robot.Movel_Cmd(pose0, 50, 0, True)  # 中间往后抽
        ret = robot.Movel_Cmd(pose13, 50, 0, True)  # 移到右边

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(2000), L_BYTE(2000),
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

        ret = robot.Movel_Cmd(pose5, 50, 0, True)  # 抓右边

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
        H_BYTE(8000), L_BYTE(8000),
        H_BYTE(12000), L_BYTE(12000),
        H_BYTE(14000), L_BYTE(14000),
        H_BYTE(0), L_BYTE(0),
        H_BYTE(0), L_BYTE(0),
        H_BYTE(65535), L_BYTE(65535),
        ],
        ROH_ADDR,
       True,
        )
        time.sleep(1)

        ret = robot.Movel_Cmd(pose14, 50, 0, True)  # 右边抓抬起
        ret = robot.Movel_Cmd(pose15, 50, 0, True)  # 移到左边
        ret = robot.Movel_Cmd(pose9, 50, 0, True)  # 左边放下

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(2000), L_BYTE(2000),
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

        ret = robot.Movel_Cmd(pose16, 50, 0, True)  # 左边抬起

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

        ret = robot.Movel_Cmd(pose0, 50, 0, True)  # 回到中间


        counter += 1
        print("已运行：", counter, "次")
    #
    #     # if counter == 5:
    #     #     sys.exit()  # 立即停止程序，返回状态码 0
