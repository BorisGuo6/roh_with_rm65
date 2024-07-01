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

    joint1_bottle = [81.739, -66.847, -110.628, 0.692, 87.338, 62.775]  # 初始抓取水瓶关节角度
    pose0_bottle = [0.057, 0.384, 0.131, -1.573, -0.474, -0.155]  # 抓水瓶中间初始位置
    pose1_bottle = [0.037, 0.494, 0.101, -1.573, -0.474, -0.055]  # 抓中间水瓶
    pose2_bottle = [0.037, 0.494, 0.151, -1.573, -0.474, -0.155] # 抬起
    pose3_bottle = [0.2145, 0.494, 0.151, -1.573, -0.474, -0.155] # 抬起移到右边
    pose4_bottle = [0.2145, 0.494, 0.101, -1.573, -0.474, -0.055]  # 放下右边
    pose5_bottle = [0.2145, 0.334, 0.101, -1.573, -0.474, -0.155] # 右边往后抽

    pose6_bottle = [0.2145, 0.334, 0.101, -1.573, -0.474, -0.155]  # 瞄准右边，抓右边为pose4_bollle
    pose7_bottle = [0.2145, 0.494, 0.151, -1.573, -0.474, -0.155]  # 右边抬起
    pose8_bottle = [0.2145, 0.334, 0.151, -1.573, -0.474, -0.155]  # 右边抬起往后抽
    pose9_bottle = [-0.1405, 0.334, 0.151, -1.573, -0.474, -0.155]  # 抬起瞄准左边
    pose10_bottle = [-0.1405, 0.494, 0.151, -1.573, -0.474, -0.155]  # 移到左边
    pose11_bottle = [-0.1405, 0.494, 0.101, -1.573, -0.474, -0.155]  # 左边放下
    pose12_bottle = [-0.1405, 0.364, 0.101, -1.573, -0.474, -0.155]  # 左边往后抽
    pose13_bottle = [0.057, 0.334, 0.101, -1.573, -0.474, -0.155]  # 移到中间
    pose14_bottle = [0.057, 0.494, 0.151, -1.573, -0.474, -0.155]  # 中间放水位
    pose15_bottle = [0.057, 0.314, 0.101, -1.573, -0.474, -0.155]  # 中间结束位

    joint1_ball = [89.568, -54.037, -129.78, 5.377, 93.872, -28.31]  # 初始抓取球关节角度
    pose0_ball = [0.016, 0.334, 0.181, 1.568, -1.07, 3.042]  # 抓球中间初始位置
    pose1_ball = [0.016, 0.516, 0.141, 2.252, -0.901, 2.468] # 抓中间球
    pose2_ball = [0.016, 0.516, 0.221, 2.252, -0.901, 2.468] # 抬起

    pose3_ball = [0.1935, 0.516, 0.221, 2.252, -0.901, 2.468] # 抬起移到右边
    pose4_ball = [0.1935, 0.516, 0.141, 2.252, -0.901, 2.468]  # 放下右边
    pose5_ball = [0.1935, 0.516, 0.181, 2.252, -0.901, 2.468] # 右边抬起

    pose6_ball = [-0.1615, 0.334, 0.181, 1.568, -1.07, 3.042] # 瞄准左边
    pose7_ball = [-0.1615, 0.334, 0.181, 1.568, -1.07, 3.042]  # 移到左边
    pose8_ball = [-0.1615, 0.516, 0.141, 2.268, -0.901, 2.448]  # 抓左边
    pose9_ball = [-0.1615, 0.516, 0.221, 2.268, -0.901, 2.448]  # 左边抬起
    pose10_ball = [0.016, 0.516, 0.221, 2.268, -0.901, 2.448]  # 抬起移到中间

    pose11_ball = [0.1935, 0.334, 0.181, 1.568, -1.07, 3.042]  # 移到右边
    pose12_ball = [-0.1615, 0.314, 0.181, 1.568, -1.07, 3.042]  # 左边往后抽





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

    ret = robot.Movej_Cmd(joint1_bottle, 30, 0, True)  # 初始抓取水瓶关节角度



    counter = 0

    while counter < 1:


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
        
        ret = robot.Movel_Cmd(pose1_bottle, 50, 0, True)# 抓中间水

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
 
        ret = robot.Movel_Cmd(pose2_bottle, 50, 0, True) # 中间抬起
        ret = robot.Movel_Cmd(pose3_bottle, 50, 0, True) # 抬起移到右边
        ret = robot.Movel_Cmd(pose4_bottle, 50, 0, True) # 放下右边

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

        ret = robot.Movel_Cmd(pose5_bottle, 50, 0, True) # 右边往后抽

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

        ret = robot.Movel_Cmd(pose6_ball, 50, 0, True) # 瞄准左边球

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

        ret = robot.Movel_Cmd(pose7_ball, 50, 0, True) # 移到左边
        ret = robot.Movel_Cmd(pose8_ball, 50, 0, True)  # 抓左边

        robot.Write_Registers(COM_PORT,ROH_FINGER_POS_TARGET0,6,
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

        ret = robot.Movel_Cmd(pose9_ball, 50, 0, True) # 左边抬起
        ret = robot.Movel_Cmd(pose10_ball, 50, 0, True) # 抬起移到中间
        ret = robot.Movel_Cmd(pose1_ball, 50, 0, True) # 中间放下

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

        ret = robot.Movel_Cmd(pose2_ball, 50, 0, True) # 中间抬起
        ret = robot.Movel_Cmd(pose0_ball, 50, 0, True)  # 中间往后抽

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

        ret = robot.Movel_Cmd(pose6_bottle, 50, 0, True) # 瞄准右边水


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

        ret = robot.Movel_Cmd(pose4_bottle, 50, 0, True) # 抓右边

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

        ret = robot.Movel_Cmd(pose7_bottle, 50, 0, True) # 右边抬起
        ret = robot.Movel_Cmd(pose8_bottle, 50, 0, True) # 右边抬起往后抽
        ret = robot.Movel_Cmd(pose9_bottle, 50, 0, True) # 抬起瞄准左边
        ret = robot.Movel_Cmd(pose10_bottle, 50, 0, True) # 抬起移到左边
        ret = robot.Movel_Cmd(pose11_bottle, 50, 0, True) # 左边放下

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

        ret = robot.Movel_Cmd(pose12_bottle, 50, 0, True) # 左边往后抽
        time.sleep(1)

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

        ret = robot.Movel_Cmd(pose0_ball, 50, 0, True) # 移到中间

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

        ret = robot.Movel_Cmd(pose1_ball, 50, 0, True)  # 抓中间球

        robot.Write_Registers(COM_PORT,ROH_FINGER_POS_TARGET0,6,
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

        ret = robot.Movel_Cmd(pose2_ball, 50, 0, True)  # 抬起
        ret = robot.Movel_Cmd(pose3_ball, 50, 0, True)  # 抬起移到右边
        ret = robot.Movel_Cmd(pose4_ball, 50, 0, True)  # 放下右边

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

        ret = robot.Movel_Cmd(pose5_ball, 50, 0, True)  # 右边抬起
        ret = robot.Movel_Cmd(pose5_bottle, 50, 0, True)  # 右边往后抽
        time.sleep(1)

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

        ret = robot.Movel_Cmd(pose9_bottle, 50, 0, True)  # 瞄准左边水


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
        ret = robot.Movel_Cmd(pose11_bottle, 50, 0, True)  # 抓左边

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

        ret = robot.Movel_Cmd(pose10_bottle, 50, 0, True)  # 左边抬起
        ret = robot.Movel_Cmd(pose9_bottle, 50, 0, True)  # 左边往后抽
        ret = robot.Movel_Cmd(pose14_bottle, 50, 0, True)  # 移到中间
        ret = robot.Movel_Cmd(pose1_bottle, 50, 0, True)  # 中间位置

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

        ret = robot.Movel_Cmd(pose0_bottle, 50, 0, True)  # 中间往后抽
        time.sleep(1)

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

        ret = robot.Movel_Cmd(pose11_ball, 50, 0, True)  # 瞄准右边球

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

        ret = robot.Movel_Cmd(pose3_ball, 50, 0, True)  # 移到右边

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

        ret = robot.Movel_Cmd(pose4_ball, 50, 0, True)  # 抓右边球

        robot.Write_Registers(COM_PORT,ROH_FINGER_POS_TARGET0,6,
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

        ret = robot.Movel_Cmd(pose5_ball, 50, 0, True)  # 右边抬起
        ret = robot.Movel_Cmd(pose11_ball, 50, 0, True)  # 往后抽
        ret = robot.Movel_Cmd(pose6_ball, 50, 0, True)  # 瞄准左边
        ret = robot.Movel_Cmd(pose8_ball, 50, 0, True)  # 左边放下

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

        ret = robot.Movel_Cmd(pose9_ball, 50, 0, True)  # 左边抬起
        time.sleep(1)

        ret = robot.Movel_Cmd(pose6_ball, 50, 0, True)  # 左边往后抽
        ret = robot.Movel_Cmd(pose15_bottle, 50, 0, True)  # 回到中间

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







        counter += 1
        print("已运行：", counter, "次")

        # if counter == 5:
        #     sys.exit()  # 立即停止程序，返回状态码 0
