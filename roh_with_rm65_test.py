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
    joint1 = [150.760, -64.495, -105.606, 163.92, -83.519, -142.282]
    joint2 = [161.357, -88.091, -63.051, 151.644, -61.901, -132.859]
    joint3 = [81.323, -18.067, -40.871, 154.945, 35.793, -128.350]
    joint4 = [0, 0, 0, 0, 0, 0]
    joint5 = [0, 0, 0, 0, 0, 0]

    counter = 0

    while counter < 10:
        robot.Close_Modbustcp_Mode()

        robot.Set_Modbus_Mode(1, 115200, 1, True)

        robot.Write_Registers(
            COM_PORT,
            ROH_FINGER_POS_TARGET0,
            6,
            [
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
            ],
            ROH_ADDR,
            True,
        )  # 后两位是大拇指旋转  #前两位大拇指弯曲
        ret = robot.Movej_Cmd(joint0, 30, 0, True)
        time.sleep(2)

        robot.Write_Registers(
            COM_PORT,
            ROH_FINGER_POS_TARGET0,
            6,
            [
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(65535),
                L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(1)

        ret = robot.Movej_Cmd(joint1, 10, 0, True)
        time.sleep(2)

        ret = robot.Movej_Cmd(joint2, 10, 0, True)
        time.sleep(2)

        robot.Write_Registers(
            COM_PORT,
            ROH_FINGER_POS_TARGET0,
            6,
            [
                H_BYTE(32000),
                L_BYTE(32000),
                H_BYTE(32000),
                L_BYTE(32000),
                H_BYTE(35000),
                L_BYTE(35000),
                H_BYTE(65535),
                L_BYTE(65535),
                H_BYTE(65535),
                L_BYTE(65535),
                H_BYTE(65535),
                L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(2)

        # ret = robot.Movej_Cmd(joint1, 10, 0, True)
        # time.sleep(2)

        ret = robot.Movej_Cmd(joint3, 10, 0, True)
        time.sleep(2)

        robot.Write_Registers(
            COM_PORT,
            ROH_FINGER_POS_TARGET0,
            6,
            [
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(0),
                L_BYTE(0),
                H_BYTE(65535),
                L_BYTE(65535),
            ],
            ROH_ADDR,
            True,
        )
        time.sleep(2)

        ret = robot.Movej_Cmd(joint1, 10, 0, True)
        time.sleep(2)

        counter += 1
        print("已运行：", counter, "次")

        # if counter == 5:
        #     sys.exit()  # 立即停止程序，返回状态码 0
