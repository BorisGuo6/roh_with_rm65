import socket
from robotic_arm_package.robotic_arm import *
from roh_registers_v1 import *


ARM_IP = "192.168.1.18"

COM_PORT = 1
ROH_ADDR = 2

RIGHT_HAND = 0
LEFT_HAND = 1

HAND_SELECTED = LEFT_HAND #选择左手或者右手

LOOP_TIME = 100 #循环次数
ARM_SPEED = 70 #机械臂速度/%
DELAY_TIME = 0.5 #延时时间/s

TABLE_HEIGHT = 0.72 #桌面高度/m
BASE_HEIGHT = 0.80 #+ 0.01 #底座高度/m

OBJECT_DISTANCE = 0.56 #底座接线口右侧中心离中间孔距离/m
HOLE_DISTANCE = 0.1775 #孔距/m

OFFSET_BASE_TABLE = TABLE_HEIGHT - BASE_HEIGHT #采用桌面底座高度差，计算Z偏移值

POS_LEFT = 0
POS_MIDDLE = 1
POS_RIGHT = 2

def L_BYTE(v):
    return v & 0xFF


def H_BYTE(v):
    return (v >> 8) & 0xFF

#
# finger control


def finger_move(target, num_finger, HAND_POS, delay):
    robot.Write_Registers(
        COM_PORT,
        target,
        num_finger,
        HAND_POS,
        ROH_ADDR,
        True
    )
    time.sleep(delay)


MAX_SINGLE_FINGER_POS = [H_BYTE(65535), L_BYTE(65535)]

MIN_SINGLE_FINGER_POS = [H_BYTE(0), L_BYTE(0)]

HAND_FIST_CLOSE_POS = [
    H_BYTE(0), L_BYTE(0),
    H_BYTE(65535), L_BYTE(65535),
    H_BYTE(65535), L_BYTE(65535),
    H_BYTE(65535), L_BYTE(65535),
    H_BYTE(65535), L_BYTE(65535),
    H_BYTE(65535), L_BYTE(65535),
]

HAND_FIST_OPEN_POS = [
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(65535), L_BYTE(65535),
]

HAND_VICTORY_POS = [
    H_BYTE(65535), L_BYTE(65535),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(65535), L_BYTE(65535),
    H_BYTE(65535), L_BYTE(65535),
    H_BYTE(65535), L_BYTE(65535),
]
target_fingers = [
    ROH_FINGER_POS_TARGET0,
    ROH_FINGER_POS_TARGET1,
    ROH_FINGER_POS_TARGET2,
    ROH_FINGER_POS_TARGET3,
    ROH_FINGER_POS_TARGET4,
    ROH_FINGER_POS_TARGET5,
]

#
# poses for ball

POSE_LIFT_LEFT_BALL_BACK_HIGH =   [
    [ 0.016 - HOLE_DISTANCE, -0.226 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 1.568, -1.07, 3.042],
    [-0.024 - HOLE_DISTANCE, -0.226 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 1.568, -1.07, 3.042]
]
POSE_LIFT_MIDDLE_BALL_BACK_HIGH = [
    [ 0.016,                 -0.226 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 1.568, -1.07, 3.042],
    [-0.024,                 -0.226 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 1.568, -1.07, 3.042]
]
POSE_LIFT_RIGHT_BALL_BACK_HIGH =  [
    [ 0.016 + HOLE_DISTANCE, -0.226 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 1.568, -1.07, 3.042],
    [-0.024 + HOLE_DISTANCE, -0.226 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 1.568, -1.07, 3.042]
]

POSE_LIFT_BALL_BACK_HIGH = [
    POSE_LIFT_LEFT_BALL_BACK_HIGH[HAND_SELECTED],
    POSE_LIFT_MIDDLE_BALL_BACK_HIGH[HAND_SELECTED],
    POSE_LIFT_RIGHT_BALL_BACK_HIGH[HAND_SELECTED]
]

POSE_GRASP_LEFT_BALL =   [
    [ 0.016 - HOLE_DISTANCE, -0.084 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361],
    [-0.024 - HOLE_DISTANCE, -0.084 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361]
]
POSE_GRASP_MIDDLE_BALL = [
    [ 0.016                , -0.084 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361],
    [-0.024                , -0.084 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361]
]
POSE_GRASP_RIGHT_BALL =  [
    [ 0.016 + HOLE_DISTANCE, -0.084 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361],
    [-0.024 + HOLE_DISTANCE, -0.084 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361]
]

POSE_GRASP_BALL = [
    POSE_GRASP_LEFT_BALL[HAND_SELECTED],
    POSE_GRASP_MIDDLE_BALL[HAND_SELECTED],
    POSE_GRASP_RIGHT_BALL[HAND_SELECTED]
]

POSE_LIFT_LEFT_BALL_HIGH =   [
    [ 0.016 - HOLE_DISTANCE, -0.084 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361],
    [-0.024 - HOLE_DISTANCE, -0.084 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361]
]
POSE_LIFT_MIDDLE_BALL_HIGH = [
    [ 0.016,                 -0.084 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361],
    [-0.024,                 -0.084 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361]
]
POSE_LIFT_RIGHT_BALL_HIGH =  [
    [ 0.016 + HOLE_DISTANCE, -0.084 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361],
    [-0.024 + HOLE_DISTANCE, -0.084 + OBJECT_DISTANCE, 0.281 + OFFSET_BASE_TABLE, 2.382, -0.832, 2.361]
]

POSE_LIFT_BALL_HIGH = [
    POSE_LIFT_LEFT_BALL_HIGH[HAND_SELECTED],
    POSE_LIFT_MIDDLE_BALL_HIGH[HAND_SELECTED],
    POSE_LIFT_RIGHT_BALL_HIGH[HAND_SELECTED]
]

HAND_READY_GRASP_BALL_POS = [
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(50535),L_BYTE(50535),
]

HAND_GRASP_BALL_POS = [
    H_BYTE(9000), L_BYTE(9000),
    H_BYTE(18000), L_BYTE(18000),
    H_BYTE(20000), L_BYTE(20000),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(50535), L_BYTE(50535),
]

#
# poses for bottle

POSE_LIFT_LEFT_BOTTLE_BACK_HIGH =   [
    [ 0.037 - HOLE_DISTANCE, -0.256 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155],
    [-0.037 - HOLE_DISTANCE, -0.256 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE,  1.573,  0.474, -3.034]
]
POSE_LIFT_MIDDLE_BOTTLE_BACK_HIGH = [
    [ 0.037,                -0.256 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155],
    [-0.037,                -0.256 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE,  1.573,  0.474, -3.034]
]
POSE_LIFT_RIGHT_BOTTLE_BACK_HIGH = [
    [ 0.037 + HOLE_DISTANCE, -0.256 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155],
    [-0.037 + HOLE_DISTANCE, -0.256 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE,  1.573,  0.474, -3.034]
]

POSE_LIFT_BOTTLE_BACK_HIGH = [
    POSE_LIFT_LEFT_BOTTLE_BACK_HIGH[HAND_SELECTED],
    POSE_LIFT_MIDDLE_BOTTLE_BACK_HIGH[HAND_SELECTED],
    POSE_LIFT_RIGHT_BOTTLE_BACK_HIGH[HAND_SELECTED]
]

POSE_GRASP_LEFT_BOTTLE = [
    [ 0.037 - HOLE_DISTANCE, -0.112 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.055],
    [-0.037 - HOLE_DISTANCE, -0.112 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE,  1.573,  0.474,  3.149]
]
POSE_GRASP_MIDDLE_BOTTLE = [
    [ 0.037,                 -0.112 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.055],
    [-0.037,                 -0.112 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE,  1.573,  0.474,  3.149]
]
POSE_GRASP_RIGHT_BOTTLE = [
    [ 0.037 + HOLE_DISTANCE, -0.112 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.055],
    [-0.037 + HOLE_DISTANCE, -0.112 + OBJECT_DISTANCE, 0.181 + OFFSET_BASE_TABLE,  1.573,  0.474,  3.149]
]

POSE_GRASP_BOTTLE = [
    POSE_GRASP_LEFT_BOTTLE[HAND_SELECTED],
    POSE_GRASP_MIDDLE_BOTTLE[HAND_SELECTED],
    POSE_GRASP_RIGHT_BOTTLE[HAND_SELECTED]
]

POSE_LIFT_LEFT_BOTTLE_HIGH = [
    [ 0.037 - HOLE_DISTANCE, -0.112 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155],
    [-0.037 - HOLE_DISTANCE, -0.112 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE,  1.573,  0.474,  3.149]
]
POSE_LIFT_MIDDLE_BOTTLE_HIGH = [
    [ 0.037,                -0.112 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155],
    [-0.037,                -0.112 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE,   1.573,  0.474,  3.149]
]
POSE_LIFT_RIGHT_BOTTLE_HIGH = [
    [ 0.037 + HOLE_DISTANCE, -0.112 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE, -1.573, -0.474, -0.155],
    [-0.037 + HOLE_DISTANCE, -0.112 + OBJECT_DISTANCE, 0.231 + OFFSET_BASE_TABLE,  1.573,  0.474,  3.149]
]

POSE_LIFT_BOTTLE_HIGH = [
    POSE_LIFT_LEFT_BOTTLE_HIGH[HAND_SELECTED],
    POSE_LIFT_MIDDLE_BOTTLE_HIGH[HAND_SELECTED],
    POSE_LIFT_RIGHT_BOTTLE_HIGH[HAND_SELECTED]
]

HAND_READY_GRASP_BOTTLE_POS = [
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(0), L_BYTE(0),
    H_BYTE(65535),L_BYTE(65535),
]

HAND_GRASP_BOTTLE_POS = [
    H_BYTE(15000),L_BYTE(15000),
    H_BYTE(30000),L_BYTE(30000),
    H_BYTE(21000),L_BYTE(21000),
    H_BYTE(19000),L_BYTE(19000),
    H_BYTE(16000),L_BYTE(16000),
    H_BYTE(65535),L_BYTE(65535),
]

HAND_INITIAL_POS = [
    H_BYTE(0),L_BYTE(0),
    H_BYTE(0),L_BYTE(0),
    H_BYTE(0),L_BYTE(0),
    H_BYTE(0),L_BYTE(0),
    H_BYTE(0),L_BYTE(0),
    H_BYTE(0),L_BYTE(0),
]


joint_initial = [78.609, 0, 0, 0, 0, 0] # 机械臂零位角度（第一关节预设定，防止摆动范围过大）
joint_Victory = [
    [78.609, 2.912, -123.641, -2.634, 110.595, -48.045], # 比耶左边角度
    [78.609, 2.912, -123.641, -2.634, 110.595, -18.045], # 比耶中间角度
    [78.609, 2.912, -123.641, -2.634, 110.595, 11.954]   # 比耶右边角度
]
joint_Dance =[89.638, -59.8, -124.738, 3.685, 111.726, -17.166] # 手指舞角度


def move_ball(from_pos, to_pos):
    ret = robot.Movel_Cmd(POSE_LIFT_BALL_BACK_HIGH[from_pos], ARM_SPEED, 0, True)

    finger_move(target_fingers[0], 6, HAND_READY_GRASP_BALL_POS, DELAY_TIME)

    ret = robot.Movel_Cmd(POSE_LIFT_BALL_HIGH[from_pos], ARM_SPEED, 0, True)
    ret = robot.Movel_Cmd(POSE_GRASP_BALL[from_pos], ARM_SPEED, 0, True)

    finger_move(target_fingers[0], 6, HAND_GRASP_BALL_POS, DELAY_TIME)

    ret = robot.Movel_Cmd(POSE_LIFT_BALL_HIGH[from_pos], ARM_SPEED, 0, True)

    ret = robot.Movel_Cmd(POSE_LIFT_BALL_BACK_HIGH[from_pos], ARM_SPEED, 0, True)
    ret = robot.Movel_Cmd(POSE_LIFT_BALL_BACK_HIGH[to_pos], ARM_SPEED, 0, True)
    ret = robot.Movel_Cmd(POSE_LIFT_BALL_HIGH[to_pos], ARM_SPEED, 0, True)
    ret = robot.Movel_Cmd(POSE_GRASP_BALL[to_pos], ARM_SPEED, 0, True)

    finger_move(target_fingers[0], 6, HAND_READY_GRASP_BALL_POS, DELAY_TIME)

    ret = robot.Movel_Cmd(POSE_LIFT_BALL_HIGH[to_pos], ARM_SPEED, 0, True)

    finger_move(target_fingers[0], 6, HAND_INITIAL_POS, DELAY_TIME)

    ret = robot.Movel_Cmd(POSE_LIFT_BALL_BACK_HIGH[to_pos], ARM_SPEED, 0, True)


def move_bottle(from_pos, to_pos):
    ret = robot.Movel_Cmd(POSE_LIFT_BOTTLE_BACK_HIGH[from_pos], ARM_SPEED, 0, True)

    finger_move(target_fingers[0], 6, HAND_READY_GRASP_BOTTLE_POS, DELAY_TIME)

    ret = robot.Movel_Cmd(POSE_GRASP_BOTTLE[from_pos], ARM_SPEED, 0, True)

    robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, 6,
        HAND_GRASP_BOTTLE_POS,
        ROH_ADDR,
        True,
    )
    time.sleep(DELAY_TIME)


    ret = robot.Movel_Cmd(POSE_LIFT_BOTTLE_HIGH[from_pos], ARM_SPEED, 0, True)

    ret = robot.Movel_Cmd(POSE_LIFT_BOTTLE_BACK_HIGH[from_pos], ARM_SPEED, 0, True)
    ret = robot.Movel_Cmd(POSE_LIFT_BOTTLE_BACK_HIGH[to_pos], ARM_SPEED, 0, True)
    ret = robot.Movel_Cmd(POSE_LIFT_BOTTLE_HIGH[to_pos], ARM_SPEED, 0, True)
    ret = robot.Movel_Cmd(POSE_GRASP_BOTTLE[to_pos], ARM_SPEED, 0, True)

    finger_move(target_fingers[0], 6, HAND_READY_GRASP_BOTTLE_POS, DELAY_TIME)

    ret = robot.Movel_Cmd(POSE_LIFT_BOTTLE_BACK_HIGH[to_pos], ARM_SPEED, 0, True)

    finger_move(target_fingers[0], 6, HAND_INITIAL_POS, DELAY_TIME)


while True:
    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    try: 
      s.connect((ARM_IP, 8080))
      s.close()
      break
    except socket.error:
      print("connect time out, try again")

    time.sleep(1)


print("Init arm")
robot = Arm(RM65, ARM_IP)


if __name__ == "__main__":

    robot.Close_Modbustcp_Mode()
    robot.Set_Modbus_Mode(1, 115200, 1, True)

    counter = 0

    #while counter < LOOP_TIME:
    while True:
        finger_move(target_fingers[0], 6, HAND_INITIAL_POS, DELAY_TIME)

        ret = robot.Movej_Cmd(joint_initial, 30, 0, True)

        ret = robot.Movej_Cmd(joint_Dance, 30, 0, True)  # 手指舞角度

        # 手指依次动
        for target in reversed(target_fingers):
            finger_move(target, 1, MAX_SINGLE_FINGER_POS, DELAY_TIME)

        for target in target_fingers:
            finger_move(target, 1, MIN_SINGLE_FINGER_POS, DELAY_TIME)

        # 握拳1
        finger_move(target_fingers[5], 1, MAX_SINGLE_FINGER_POS, DELAY_TIME)
        finger_move(target_fingers[0], 6, HAND_FIST_CLOSE_POS, DELAY_TIME)
        finger_move(target_fingers[0], 1, MAX_SINGLE_FINGER_POS, 1)

        # 握拳2
        finger_move(target_fingers[0], 1, MIN_SINGLE_FINGER_POS, DELAY_TIME)
        finger_move(target_fingers[0], 6, HAND_FIST_OPEN_POS, DELAY_TIME)
        time.sleep(0.5)

        finger_move(target_fingers[0], 6, HAND_FIST_CLOSE_POS, DELAY_TIME)
        finger_move(target_fingers[0], 1, MAX_SINGLE_FINGER_POS, 1)
        finger_move(target_fingers[0], 1, MIN_SINGLE_FINGER_POS, 1)
        finger_move(target_fingers[0], 6, HAND_FIST_OPEN_POS, DELAY_TIME)
        finger_move(target_fingers[5], 1, MIN_SINGLE_FINGER_POS, DELAY_TIME)

        time.sleep(1)

        # initial state: bottle @ middle, ball @ left 

        move_bottle(POS_MIDDLE, POS_RIGHT)  # bottle -> right, ball @ left
        time.sleep(DELAY_TIME)

        move_ball(POS_LEFT, POS_MIDDLE)     # bottle @ right, ball -> middle
        time.sleep(DELAY_TIME)

        move_bottle(POS_RIGHT, POS_LEFT)    # bottle -> left, ball @ middle
        time.sleep(DELAY_TIME)

        move_ball(POS_MIDDLE, POS_RIGHT)    # bottle @ left, ball -> right
        time.sleep(DELAY_TIME)

        move_bottle(POS_LEFT, POS_MIDDLE)   # bottle -> middle, ball @ right
        time.sleep(DELAY_TIME)

        move_ball(POS_RIGHT, POS_LEFT)     # bottle @ middle, ball -> left
        time.sleep(DELAY_TIME)


        ret = robot.Movej_Cmd(joint_Victory[POS_MIDDLE], 30, 0, True)  # 比耶

        finger_move(target_fingers[0], 6, HAND_VICTORY_POS, DELAY_TIME)

        ret = robot.Movej_Cmd(joint_Victory[POS_LEFT], ARM_SPEED, 0, True)  # 比耶左
        ret = robot.Movej_Cmd(joint_Victory[POS_RIGHT], ARM_SPEED, 0, True)  # 比耶右


        counter += 1
        print("已运行：", counter, "次")

        # if counter == 5:
        #     sys.exit()  # 立即停止程序，返回状态码 0
