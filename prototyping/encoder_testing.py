import serial
import time

from dk1.DM_Control_Python.DM_CAN import *
import serial

motor1=Motor(DM_Motor_Type.DM4340, 0x02, 0x12)

serial_device = serial.Serial('/dev/tty.usbmodem00000000050C1', 921600, timeout=0.5)
control=MotorControl(serial_device)
control.addMotor(motor1)

control.switchControlMode(motor1, Control_Type.POS_VEL)
control.change_motor_param(motor1, DM_variable.KP_APR, 120)
control.enable(motor1)

# Change this to your actual serial port (e.g. 'COM3' on Windows, '/dev/ttyUSB0' or '/dev/ttyACM0' on Linux)
SERIAL_PORT = '/dev/tty.usbserial-BG0038JI'
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for Arduino to reset
print("Reading angles from Arduino...\nPress Ctrl+C to stop.\n")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
    print("Reading angles from Arduino...\nPress Ctrl+C to stop.\n")
    
    
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                angle = float(line)
                print(f"Angle: {angle:.2f} rad")
                
                control.control_Pos_Vel(motor1, P_desired=angle , V_desired=5.0)
                
                
            except ValueError:
                print(f"Invalid data: {line}")
                


except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    control.disable(motor1)

    if 'ser' in locals() and ser.is_open:
        ser.close()
