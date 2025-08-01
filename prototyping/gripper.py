from dk1.DM_Control_Python.DM_CAN import *
import serial
import time

motor1=Motor(DM_Motor_Type.DM4310,0x01,0x11)

serial_device = serial.Serial('/dev/tty.usbmodem00000000050C1', 921600, timeout=0.5)
control=MotorControl(serial_device)
control.addMotor(motor1)

if control.switchControlMode(motor1, Control_Type.Torque_Pos):
    print("switch MIT success")

control.enable(motor1)

while True:
    print(motor1.getPosition())
    # control.controlMIT(motor1, 35, 0.1, 0, 0, 0)
    control.control_pos_force(motor1, 0.0, 1000, 1000)

    time.sleep(0.001)
    