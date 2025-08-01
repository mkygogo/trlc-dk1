from dk1.DM_Control_Python.DM_CAN import *
import serial
import time


DM4310_TORQUE_CONSTANT = 0.945 # Nm/A
DM4340_TORQUE_CONSTANT = 4.074 # Nm/A

VELOCITY_SCALE = 100 # rad/s
CURRENT_SCALE = 1000 # A


motor1=Motor(DM_Motor_Type.DM4340, 0x01, 0x11)

serial_device = serial.Serial('/dev/tty.usbmodem00000000050C1', 921600, timeout=0.5)
control=MotorControl(serial_device)
control.addMotor(motor1)

# control.set_zero_position(motor1)

if control.switchControlMode(motor1, Control_Type.Torque_Pos):
    print("switch MIT success")

control.enable(motor1)

max_velocity = 10.0 # rad/s
max_torque = 2.0 # Nm

for i in range(10000):
    i+=1
    
    measured_torque = motor1.getTorque()
    
    print(f"pos: {motor1.getPosition():.3f}, torque: {measured_torque:.3f}")
    if abs(measured_torque) > max_torque:
        print("torque too high")
        
    control.control_pos_force(Motor=motor1, Pos_des=0.0, Vel_des=max_velocity*VELOCITY_SCALE, i_des=max_torque/DM4340_TORQUE_CONSTANT*CURRENT_SCALE)

    time.sleep(0.001)
    
control.disable(motor1)
