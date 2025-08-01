from dk1.DM_Control_Python.DM_CAN import *
import serial
import time


motor1=Motor(DM_Motor_Type.DM4310, 0x01, 0x11)

serial_device = serial.Serial('/dev/tty.usbmodem00000000050C1', 921600, timeout=0.5)
control=MotorControl(serial_device)
control.addMotor(motor1)

if control.switchControlMode(motor1, Control_Type.VEL):
    print("Switch to VEL success")
    
control.enable(motor1)

open_reached = False
control.control_Vel(motor1, 2.0)


while not open_reached:
    control.refresh_motor_status(motor1)
    torque = motor1.getTorque()
    # print(f"Torque: {torque:.3f}")
    if abs(torque) > 0.4:
        open_reached = True
    time.sleep(0.001)

print("Open reached")
control.control_Vel(motor1, -2.0)
time.sleep(0.1)
control.refresh_motor_status(motor1)
gripper_open_position = motor1.getPosition()
print(f"Gripper open position: {gripper_open_position:.3f}")
time.sleep(0.1)
closed_reached = False
control.control_Vel(motor1, -2.0)

while not closed_reached:
    control.refresh_motor_status(motor1)
    torque = motor1.getTorque()
    # print(f"Torque: {torque:.3f}")
    if abs(torque) > 0.4:
        closed_reached = True
    time.sleep(0.001)

print("Closed reached")
gripper_closed_position = motor1.getPosition()
print(f"Gripper closed position: {gripper_closed_position:.3f}")

print("Calibration done")

control.disable(motor1)