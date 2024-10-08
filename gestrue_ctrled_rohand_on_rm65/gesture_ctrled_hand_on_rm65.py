import os
import cv2
from roh_registers_v1 import *
from robotic_arm_package.robotic_arm import *

from cvzone.HandTrackingModule import HandDetector
from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient


ARM_IP = "192.168.1.18"

COM_PORT = 1
ROH_ADDR = 2

robot = Arm(RM65, ARM_IP)

file_path = os.path.abspath(os.path.dirname(__file__))
detector = HandDetector(maxHands=1, detectionCon=0.8)
video = cv2.VideoCapture(0)

robot.Close_Modbustcp_Mode()
robot.Set_Modbus_Mode(1, 115200, 1, True)

# client = ModbusSerialClient(COM_PORT, FramerType.RTU, 115200)
# client.connect()

prev_gesture = [0, 0, 0, 0, 0, 0]

def L_BYTE(v):
    return v & 0xFF


def H_BYTE(v):
    return (v >> 8) & 0xFF

Joint_Ready = [90, 0, -90, 0, 90, -12]

ret = robot.Movej_Cmd(Joint_Ready, 30, 0, True)

while True:
    _, img = video.read()
    img = cv2.flip(img, 1)
    hand = detector.findHands(img, draw=False)
    gesture_pic = cv2.imread(file_path + "/gestures/0.png")
    gesture = [45000, 65535, 65535, 65535, 65535, 0]

    if hand:
        lmlist = hand[0]

        if lmlist and lmlist[0]:
            finger_up = detector.fingersUp(lmlist[0])

            for i in range(5):
                gesture[i] = gesture[i] * (1 - finger_up[i])

            # print(gesture)

            if finger_up == [0, 1, 0, 0, 0]:
                gesture_pic = cv2.imread(file_path + "/gestures/1.png")
            elif finger_up == [0, 1, 1, 0, 0]:
                gesture_pic = cv2.imread(file_path + "/gestures/2.png")
            elif finger_up == [0, 1, 1, 1, 0]:
                gesture_pic = cv2.imread(file_path + "/gestures/3.png")
            elif finger_up == [0, 1, 1, 1, 1]:
                gesture_pic = cv2.imread(file_path + "/gestures/4.png")
            elif finger_up == [1, 1, 1, 1, 1]:
                gesture_pic = cv2.imread(file_path + "/gestures/5.png")

    # print(gesture_pic)
    if gesture_pic.any():
        gesture_pic = cv2.resize(gesture_pic, (161, 203))
        img[0:203, 0:161] = gesture_pic
        cv2.imshow("Video", img)

    if (prev_gesture != gesture):
        gesture_bytes = []
        
        for i in range(len(gesture)):
            gesture_bytes.append(H_BYTE(gesture[i]))
            gesture_bytes.append(L_BYTE(gesture[i]))

        resp = robot.Write_Registers(COM_PORT, ROH_FINGER_POS_TARGET0, len(gesture), gesture_bytes, ROH_ADDR,True)
        print("client.write_registers() returned", resp)
        prev_gesture = gesture

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()

