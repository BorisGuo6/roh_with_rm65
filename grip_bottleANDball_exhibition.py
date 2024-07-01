from robotic_arm_package.robotic_arm import *
from roh_registers import *


ARM_IP = "192.168.1.18"

COM_PORT = 1
ROH_ADDR = 2

LOOP_TIME = 10 #循环次数
ARM_SPEED = 90 #机械臂速度/%
DELAY_TIME = 0.5 #延时时间/s

TABLE_HEIGHT = 0.72 #桌面高度/m
BASE_HEIGHT = 0.80 #底座高度/m

OBJECT_DISTANCE = 0.56 #底座接线口右侧中心离中间孔距离/m
HOLE_DISTANCE = 0.1775 #孔距/m

OFFSET_BASE_TABLE = TABLE_HEIGHT - BASE_HEIGHT #采用桌面底座高度差，计算Z偏移值
def L_BYTE(v):
    return v & 0xFF


def H_BYTE(v):
    return (v >> 8) & 0xFF


robot = Arm(RM65, ARM_IP)

if __name__ == "__main__":

    joint0 = [0, 0, 0, 0, 0, 0]
    joint_V1 = [78.609, 2.912, -123.641, -2.634, 110.595, -18.045]  # 比耶中间角度
    joint_V2 = [78.609, 2.912, -123.641, -2.634, 110.595, -48.045]  # 比耶左边角度
    joint_V3 = [78.609, 2.912, -123.641, -2.634, 110.595, 11.954]  # 比耶右边角度
    joint_Dance =[89.638, -59.8, -124.738, 3.685, 111.726, -17.166]  # 手指舞角度

    joint1_bottle = [81.739, -66.847, -110.628, 0.692, 87.338, 62.775]  # 初始抓取水瓶关节角度

    pose0_bottle = [0.057, -0.176 + OBJECT_DISTANCE, 0.211 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155]  # 抓水瓶中间初始位置
    pose1_bottle = [0.037, -0.066 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.055]  # 抓中间水瓶
    pose2_bottle = [0.037, -0.066 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155] # 抬起
    pose3_bottle = [0.037 + HOLE_DISTANCE, -0.066 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155] # 抬起移到右边
    pose4_bottle = [0.037 + HOLE_DISTANCE, -0.066 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.055]  # 放下右边
    pose5_bottle = [0.037 + HOLE_DISTANCE, -0.226 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155] # 右边往后抽

    pose6_bottle = [0.037 + HOLE_DISTANCE, -0.226 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155]  # 瞄准右边，抓右边为pose4_bollle
    pose7_bottle = [0.037 + HOLE_DISTANCE, -0.066 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155]  # 右边抬起
    pose8_bottle = [0.037 + HOLE_DISTANCE, -0.226 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155]  # 右边抬起往后抽
    pose9_bottle = [0.037 - HOLE_DISTANCE, -0.226 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155]  # 抬起瞄准左边
    pose10_bottle = [0.037 - HOLE_DISTANCE, -0.066 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155]  # 移到左边
    pose11_bottle = [0.037 - HOLE_DISTANCE, -0.066 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155]  # 左边放下
    pose12_bottle = [0.037 - HOLE_DISTANCE, -0.196 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155]  # 左边往后抽
    pose13_bottle = [0.057, -0.226 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155]  # 移到中间
    pose14_bottle = [0.057, -0.066 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155]  # 中间放水位
    pose15_bottle = [0.057, -0.226 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155]  # 中间结束位

    pose0_ball = [0.016, -0.226 + OBJECT_DISTANCE, 0.261 + OFFSET_BASE_TABLE, 1.568, -1.07, 3.042]  # 抓球中间初始位置
    pose1_ball = [0.016, -0.044 + OBJECT_DISTANCE, 0.221 + OFFSET_BASE_TABLE, 2.252, -0.901, 2.468] # 抓中间球
    pose2_ball = [0.016, -0.044 + OBJECT_DISTANCE, 0.301 + OFFSET_BASE_TABLE, 2.252, -0.901, 2.468] # 抬起

    pose3_ball = [0.016 + HOLE_DISTANCE, -0.044 + OBJECT_DISTANCE, 0.301 + OFFSET_BASE_TABLE, 2.252, -0.901, 2.468] # 抬起移到右边
    pose4_ball = [0.016 + HOLE_DISTANCE, -0.044 + OBJECT_DISTANCE, 0.221 + OFFSET_BASE_TABLE, 2.252, -0.901, 2.468]  # 放下右边
    pose5_ball = [0.016 + HOLE_DISTANCE, -0.044 + OBJECT_DISTANCE, 0.261 + OFFSET_BASE_TABLE, 2.252, -0.901, 2.468] # 右边抬起

    pose6_ball = [0.016 - HOLE_DISTANCE, -0.226 + OBJECT_DISTANCE, 0.261 + OFFSET_BASE_TABLE, 1.568, -1.07, 3.042] # 瞄准左边
    pose7_ball = [0.016 - HOLE_DISTANCE, -0.226 + OBJECT_DISTANCE, 0.261 + OFFSET_BASE_TABLE, 1.568, -1.07, 3.042]  # 移到左边
    pose8_ball = [0.016 - HOLE_DISTANCE, -0.044 + OBJECT_DISTANCE, 0.221 + OFFSET_BASE_TABLE, 2.268, -0.901, 2.448]  # 抓左边
    pose9_ball = [0.016 - HOLE_DISTANCE, -0.044 + OBJECT_DISTANCE, 0.301 + OFFSET_BASE_TABLE, 2.268, -0.901, 2.448]  # 左边抬起
    pose10_ball = [0.016, -0.044 + OBJECT_DISTANCE, 0.301 + OFFSET_BASE_TABLE, 2.268, -0.901, 2.448]  # 抬起移到中间

    pose11_ball = [0.016 + HOLE_DISTANCE, -0.226 + OBJECT_DISTANCE, 0.261 + OFFSET_BASE_TABLE, 1.568, -1.07, 3.042]  # 移到右边
    pose12_ball = [0.016 - HOLE_DISTANCE, -0.246 + OBJECT_DISTANCE, 0.261 + OFFSET_BASE_TABLE, 1.568, -1.07, 3.042]  # 左边往后抽





    robot.Close_Modbustcp_Mode()

    robot.Set_Modbus_Mode(1, 115200, 1, True)


    counter = 0

    while counter < LOOP_TIME:
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

        ret = robot.Movej_Cmd(joint_Dance, 30, 0, True)  # 手指舞角度
        # 手指舞
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET4, 1,[H_BYTE(65535), L_BYTE(65535),],ROH_ADDR,True,)
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET3, 1, [H_BYTE(65535), L_BYTE(65535), ], ROH_ADDR, True, )
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET2, 1, [H_BYTE(65535), L_BYTE(65535), ], ROH_ADDR, True, )
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET1, 1, [H_BYTE(65535), L_BYTE(65535), ], ROH_ADDR, True, )
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET5, 1, [H_BYTE(65535), L_BYTE(65535), ], ROH_ADDR, True, )
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 1, [H_BYTE(65535), L_BYTE(65535), ], ROH_ADDR, True, )
        time.sleep(1)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 1, [H_BYTE(0), L_BYTE(0), ], ROH_ADDR, True, )
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET5, 1, [H_BYTE(0), L_BYTE(0), ], ROH_ADDR, True, )
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET1, 1, [H_BYTE(0), L_BYTE(0), ], ROH_ADDR, True, )
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET2, 1, [H_BYTE(0), L_BYTE(0), ], ROH_ADDR, True, )
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET3, 1, [H_BYTE(0), L_BYTE(0), ], ROH_ADDR, True, )
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET4, 1, [H_BYTE(0), L_BYTE(0), ], ROH_ADDR, True, )
        time.sleep(1)
        #握拳1

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET5, 1, [H_BYTE(65535), L_BYTE(65535), ], ROH_ADDR, True, )
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET1, 4,
        [
        H_BYTE(65535), L_BYTE(65535),
        H_BYTE(65535), L_BYTE(65535),
        H_BYTE(65535), L_BYTE(65535),
        H_BYTE(65535), L_BYTE(65535),

        ],
        ROH_ADDR,
        True,
        )
        time.sleep(0.3)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 1, [H_BYTE(65535), L_BYTE(65535), ], ROH_ADDR, True, )
        time.sleep(1)
        # 握拳2
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 1, [H_BYTE(0), L_BYTE(0), ], ROH_ADDR, True, )
        time.sleep(1)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET1, 4,
        [
        H_BYTE(0), L_BYTE(0),
        H_BYTE(0), L_BYTE(0),
        H_BYTE(0), L_BYTE(0),
        H_BYTE(0), L_BYTE(0),

        ],
        ROH_ADDR,
        True,
        )
        time.sleep(0.5)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET1, 4,
        [
        H_BYTE(65535), L_BYTE(65535),
        H_BYTE(65535), L_BYTE(65535),
        H_BYTE(65535), L_BYTE(65535),
        H_BYTE(65535), L_BYTE(65535),

        ],
        ROH_ADDR,
        True,
        )
        time.sleep(0.3)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 1, [H_BYTE(65535), L_BYTE(65535), ], ROH_ADDR, True, )
        time.sleep(1)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 1, [H_BYTE(0), L_BYTE(0), ], ROH_ADDR, True, )
        time.sleep(1)
        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET1, 5,
        [
         H_BYTE(0), L_BYTE(0),
         H_BYTE(0), L_BYTE(0),
         H_BYTE(0), L_BYTE(0),
         H_BYTE(0), L_BYTE(0),
         H_BYTE(0), L_BYTE(0),
         ],
         ROH_ADDR,
         True,
         )
        time.sleep(2)



        #ret = robot.Movej_Cmd(joint1_bottle, 30, 0, True)  # 初始抓取水瓶关节角度
        ret = robot.Movel_Cmd(pose0_bottle, 30, 0, True)  # 初始抓取水瓶关节角度

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
        time.sleep(DELAY_TIME)
        
        ret = robot.Movel_Cmd(pose1_bottle, ARM_SPEED, 0, True)# 抓中间水

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
        time.sleep(DELAY_TIME)
 
        ret = robot.Movel_Cmd(pose2_bottle, ARM_SPEED, 0, True) # 中间抬起
        ret = robot.Movel_Cmd(pose3_bottle, ARM_SPEED, 0, True) # 抬起移到右边
        ret = robot.Movel_Cmd(pose4_bottle, ARM_SPEED, 0, True) # 放下右边

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
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose5_bottle, ARM_SPEED, 0, True) # 右边往后抽

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
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose6_ball, ARM_SPEED, 0, True) # 瞄准左边球

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(55535), L_BYTE(55535),
            ],
            ROH_ADDR,
            True,
        )


        ret = robot.Movel_Cmd(pose7_ball, ARM_SPEED, 0, True) # 移到左边
        ret = robot.Movel_Cmd(pose8_ball, ARM_SPEED, 0, True)  # 抓左边

        robot.Write_Registers(COM_PORT,ROH_FINGER_POS_TARGET0,6,
        [
            H_BYTE(9000), L_BYTE(9000),
            H_BYTE(13000), L_BYTE(13000),
            H_BYTE(16000), L_BYTE(16000),
            H_BYTE(0), L_BYTE(0),
            H_BYTE(0), L_BYTE(0),
            H_BYTE(55535), L_BYTE(55535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose9_ball, ARM_SPEED, 0, True) # 左边抬起
        ret = robot.Movel_Cmd(pose10_ball, ARM_SPEED, 0, True) # 抬起移到中间
        ret = robot.Movel_Cmd(pose1_ball, ARM_SPEED, 0, True) # 中间放下

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(55535), L_BYTE(55535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose2_ball, ARM_SPEED, 0, True) # 中间抬起
        ret = robot.Movel_Cmd(pose0_ball, ARM_SPEED, 0, True)  # 中间往后抽

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
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose6_bottle, ARM_SPEED, 0, True) # 瞄准右边水


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
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose4_bottle, ARM_SPEED, 0, True) # 抓右边

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
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose7_bottle, ARM_SPEED, 0, True) # 右边抬起
        ret = robot.Movel_Cmd(pose8_bottle, ARM_SPEED, 0, True) # 右边抬起往后抽
        ret = robot.Movel_Cmd(pose9_bottle, ARM_SPEED, 0, True) # 抬起瞄准左边
        ret = robot.Movel_Cmd(pose10_bottle, ARM_SPEED, 0, True) # 抬起移到左边
        ret = robot.Movel_Cmd(pose11_bottle, ARM_SPEED, 0, True) # 左边放下

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
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose12_bottle, ARM_SPEED, 0, True) # 左边往后抽
        time.sleep(DELAY_TIME)

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
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose0_ball, ARM_SPEED, 0, True) # 移到中间

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(55535), L_BYTE(55535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose1_ball, ARM_SPEED, 0, True)  # 抓中间球

        robot.Write_Registers(COM_PORT,ROH_FINGER_POS_TARGET0,6,
        [
            H_BYTE(9000), L_BYTE(9000),
            H_BYTE(13000), L_BYTE(13000),
            H_BYTE(16000), L_BYTE(16000),
            H_BYTE(0), L_BYTE(0),
            H_BYTE(0), L_BYTE(0),
            H_BYTE(55535), L_BYTE(55535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose2_ball, ARM_SPEED, 0, True)  # 抬起
        ret = robot.Movel_Cmd(pose3_ball, ARM_SPEED, 0, True)  # 抬起移到右边
        ret = robot.Movel_Cmd(pose4_ball, ARM_SPEED, 0, True)  # 放下右边

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(55535), L_BYTE(55535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose5_ball, ARM_SPEED, 0, True)  # 右边抬起
        ret = robot.Movel_Cmd(pose5_bottle, ARM_SPEED, 0, True)  # 右边往后抽
        time.sleep(DELAY_TIME)

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
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose9_bottle, ARM_SPEED, 0, True)  # 瞄准左边水


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
        time.sleep(DELAY_TIME)
        ret = robot.Movel_Cmd(pose11_bottle, ARM_SPEED, 0, True)  # 抓左边

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
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose10_bottle, ARM_SPEED, 0, True)  # 左边抬起
        ret = robot.Movel_Cmd(pose2_bottle, ARM_SPEED, 0, True)  # 移到中间
        ret = robot.Movel_Cmd(pose1_bottle, ARM_SPEED, 0, True)  # 中间位置放下

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
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose0_bottle, ARM_SPEED, 0, True)  # 中间往后抽
        time.sleep(DELAY_TIME)

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
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose11_ball, ARM_SPEED, 0, True)  # 瞄准右边球

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
       [
       H_BYTE(0), L_BYTE(0),
       H_BYTE(0), L_BYTE(0),
       H_BYTE(0), L_BYTE(0),
       H_BYTE(0), L_BYTE(0),
       H_BYTE(0), L_BYTE(0),
       H_BYTE(55535), L_BYTE(55535),
       ],
       ROH_ADDR,
       True,
        )
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose4_ball, ARM_SPEED, 0, True)  # 抓右边球

        robot.Write_Registers(COM_PORT,ROH_FINGER_POS_TARGET0,6,
        [
            H_BYTE(9000), L_BYTE(9000),
            H_BYTE(13000), L_BYTE(13000),
            H_BYTE(16000), L_BYTE(16000),
            H_BYTE(0), L_BYTE(0),
            H_BYTE(0), L_BYTE(0),
            H_BYTE(55535), L_BYTE(55535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose5_ball, ARM_SPEED, 0, True)  # 右边抬起
        ret = robot.Movel_Cmd(pose11_ball, ARM_SPEED, 0, True)  # 往后抽
        ret = robot.Movel_Cmd(pose6_ball, ARM_SPEED, 0, True)  # 瞄准左边
        ret = robot.Movel_Cmd(pose9_ball, ARM_SPEED, 0, True)  # 移到左边
        ret = robot.Movel_Cmd(pose8_ball, ARM_SPEED, 0, True)  # 左边放下

        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(55535), L_BYTE(55535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose9_ball, ARM_SPEED, 0, True)  # 左边抬起
        time.sleep(DELAY_TIME)

        ret = robot.Movel_Cmd(pose6_ball, ARM_SPEED, 0, True)  # 左边往后抽
        ret = robot.Movel_Cmd(pose15_bottle, ARM_SPEED, 0, True)  # 回到中间
        ret = robot.Movej_Cmd(joint_V1, ARM_SPEED, 0, True)  # 比耶



        robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        [
                H_BYTE(65535), L_BYTE(65535),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(0), L_BYTE(0),
                H_BYTE(65535), L_BYTE(65535),
                H_BYTE(65535), L_BYTE(65535),
                H_BYTE(65535), L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(DELAY_TIME)
        ret = robot.Movej_Cmd(joint_V2, ARM_SPEED, 0, True)  # 比耶左
        ret = robot.Movej_Cmd(joint_V3, ARM_SPEED, 0, True)  # 比耶右


        counter += 1
        print("已运行：", counter, "次")

        # if counter == 5:
        #     sys.exit()  # 立即停止程序，返回状态码 0
