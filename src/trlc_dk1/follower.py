from dataclasses import dataclass, field
from functools import cached_property
import serial
import time
import logging
from typing import Any

from lerobot.cameras import CameraConfig
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots import Robot, RobotConfig
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from trlc_dk1.motors.DM_Control_Python.DM_CAN import *

logger = logging.getLogger(__name__)

def map_range(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


@RobotConfig.register_subclass("dk1_follower")
@dataclass
class DK1FollowerConfig(RobotConfig):
    port: str
    
    disable_torque_on_disconnect: bool = False

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    # cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    

class DK1Follower(Robot):
    """
    TRLC-DK1 Follower Arm designed by The Robot Learning Company.
    """
    
    config_class = DK1FollowerConfig
    name = "dk1_follower"
    
    def __init__(self, config: DK1FollowerConfig):
        super().__init__(config)
        
        self.config = config
        self.motors = {
            "joint_1": Motor(DM_Motor_Type.DM4340, 0x01, 0x11),
            "joint_2": Motor(DM_Motor_Type.DM4340, 0x02, 0x12),
            "joint_3": Motor(DM_Motor_Type.DM4340, 0x03, 0x13),
            "joint_4": Motor(DM_Motor_Type.DM4310, 0x04, 0x14),
            "joint_5": Motor(DM_Motor_Type.DM4310, 0x05, 0x15),
            "joint_6": Motor(DM_Motor_Type.DM4310, 0x06, 0x16),
            "gripper": Motor(DM_Motor_Type.DM4310, 0x07, 0x17),
        }
        self.control = None
        self.serial_device = None
        self.bus_connected = False
        
        self.gripper_open_pos = 0.0
        self.gripper_closed_pos = -4.6
        
        self.cameras = make_cameras_from_configs(config.cameras)
        
    
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.motors}
    
    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }
        
    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft
    
    @property
    def is_connected(self) -> bool:
        return self.bus_connected and all(cam.is_connected for cam in self.cameras.values())
    
    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        
        self.serial_device = serial.Serial(self.config.port, 921600, timeout=0.5)
        time.sleep(0.5)
        
        self.control = MotorControl(self.serial_device)
        self.bus_connected = True
        self.configure()
            
        for cam in self.cameras.values():
            cam.connect()
    
    @property
    def is_calibrated(self) -> bool:
        return True
    
    def calibrate(self) -> None:
        pass
    
    def configure(self) -> None:
        
        for key, motor in self.motors.items():
            self.control.addMotor(motor)
            
            for _ in range(3):
                self.control.refresh_motor_status(motor)
                time.sleep(0.01)
            
            if self.control.read_motor_param(motor, DM_variable.CTRL_MODE) is not None:
                print(f"{key} ({motor.MotorType.name}) is connected.")
                
                self.control.switchControlMode(motor, Control_Type.POS_VEL)
                self.control.enable(motor)
            else:
                raise Exception(f"Unable to read from {key} ({motor.MotorType.name}).")
            
        for joint in ["joint_1", "joint_2", "joint_3"]:
            # self.control.change_motor_param(self.motors[joint], DM_variable.ACC, 10.0)
            # self.control.change_motor_param(self.motors[joint], DM_variable.DEC, -10.0)
            self.control.change_motor_param(self.motors[joint], DM_variable.KP_APR, 160)
            self.control.change_motor_param(self.motors[joint], DM_variable.KI_APR, 10)
        
        for joint in ["gripper"]:
            self.control.change_motor_param(self.motors[joint], DM_variable.KP_APR, 100)
            
        # Open gripper and set zero position
        self.control.switchControlMode(self.motors["gripper"], Control_Type.VEL)
        self.control.control_Vel(self.motors["gripper"], 2.0)
        while True:
            self.control.refresh_motor_status(self.motors["gripper"])
            tau = self.motors["gripper"].getTorque()
            if tau > 0.6:
                self.control.control_Vel(self.motors["gripper"], 0.0)
                self.control.disable(self.motors["gripper"])
                self.control.set_zero_position(self.motors["gripper"])
                time.sleep(0.2)
                self.control.enable(self.motors["gripper"])
                break
            time.sleep(0.01)
        self.control.switchControlMode(self.motors["gripper"], Control_Type.POS_VEL)  
        
    
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()
        
        obs_dict = {}
        for key, motor in self.motors.items():
            self.control.refresh_motor_status(motor)
            if key == "gripper":
                # Normalize gripper position between 1 (closed) and 0 (open)
                obs_dict[f"{key}.pos"] = map_range(motor.getPosition(), self.gripper_open_pos, self.gripper_closed_pos, 0.0, 1.0)
            else:
                obs_dict[f"{key}.pos"] = motor.getPosition()        
        
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict
    
        
    def send_action(self, action: dict[str, Any], V_desired: float = 0.5) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            raise NotImplementedError("Max relative target is not implemented for the follower.")
            present_pos = {}
            for key, motor in self.motors.items():
                self.control.refresh_motor_status(motor)
                present_pos[key] = motor.getPosition()
            
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send goal position to the arm
        for key, motor in self.motors.items():
            if key == "gripper":
                goal_pos[key] = map_range(goal_pos[key], 0.0, 1.0, self.gripper_open_pos, self.gripper_closed_pos)
            self.control.control_Pos_Vel(motor, goal_pos[key], V_desired)
        
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}
    
     
    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.config.disable_torque_on_disconnect:
            for motor in self.motors.values():
                self.control.disable(motor)
        else:
            self.control.serial_.close()
        self.bus_connected = False
            
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")