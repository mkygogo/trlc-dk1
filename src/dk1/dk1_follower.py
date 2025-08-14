import serial
from dataclasses import dataclass, field
from functools import cached_property
from typing import Any
import time
import logging

from dk1.DM_Control_Python.DM_CAN import *

from lerobot.robots.robot import Robot
from lerobot.robots.config import RobotConfig
from lerobot.cameras import CameraConfig
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.utils import ensure_safe_goal_position

logger = logging.getLogger(__name__)


@RobotConfig.register_subclass("dk1_follower")
@dataclass
class DK1FollowerConfig(RobotConfig):
    port: str
    disable_torque_on_disconnect: bool = True
    max_relative_target: int | None = None
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    use_degrees: bool = True


class DK1Follower(Robot):
    config_class = DK1FollowerConfig
    name = "dk1_follower"

    def __init__(self, config: DK1FollowerConfig):
        super().__init__(config)
        self.config = config
        
        # MAKE PROPER DM_CAN BUS CLASS
        self.motors = {
            "joint_1": Motor(DM_Motor_Type.DM4340, 0x01, 0x11),
            "joint_2": Motor(DM_Motor_Type.DM4340, 0x02, 0x12),
            "joint_3": Motor(DM_Motor_Type.DM4340, 0x03, 0x13),
            "joint_4": Motor(DM_Motor_Type.DM4310, 0x04, 0x14),
            "joint_5": Motor(DM_Motor_Type.DM4310, 0x05, 0x15),
            "joint_6": Motor(DM_Motor_Type.DM4310, 0x06, 0x16),
        }
        
        self.cameras = make_cameras_from_configs(config.cameras)
        self.connected = False
        
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
        return self.connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        
        # DM Control Connection
        self.serial_device = serial.Serial(self.config.port, 921600, timeout=0.5)
        self.control=MotorControl(self.serial_device)

        # Add motors to control
        for motor in self.motors.values():
            self.control.addMotor(motor)
            self.control.switchControlMode(motor, Control_Type.POS_VEL)
            self.control.refresh_motor_status(motor)
            self.control.change_motor_param(motor, DM_variable.KP_APR, 120)
            print(f"KP_APR set to {self.control.read_motor_param(motor, DM_variable.KP_APR)}")
            self.control.enable(motor)

        # Switch control mode to position velocity
        # for motor in self.motors.values():
        #     self.control.switchControlMode(motor, Control_Type.POS_VEL)
        
        # self.bus.connect()
        # if not self.is_calibrated and calibrate:
        #     logger.info(
        #         "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
        #     )
        #     self.calibrate()
        
        self.connected = True

        for cam in self.cameras.values():
            cam.connect()

        # self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        pass
        # if self.calibration:
        #     # self.calibration is not empty here
        #     user_input = input(
        #         f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
        #     )
        #     if user_input.strip().lower() != "c":
        #         logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
        #         self.bus.write_calibration(self.calibration)
        #         return

        # logger.info(f"\nRunning calibration of {self}")
        # self.bus.disable_torque()
        # for motor in self.bus.motors:
        #     self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        # input(f"Move {self} to the middle of its range of motion and press ENTER....")
        # homing_offsets = self.bus.set_half_turn_homings()

        # print(
        #     "Move all joints sequentially through their entire ranges "
        #     "of motion.\nRecording positions. Press ENTER to stop..."
        # )
        # range_mins, range_maxes = self.bus.record_ranges_of_motion()

        # self.calibration = {}
        # for motor, m in self.bus.motors.items():
        #     self.calibration[motor] = MotorCalibration(
        #         id=m.id,
        #         drive_mode=0,
        #         homing_offset=homing_offsets[motor],
        #         range_min=range_mins[motor],
        #         range_max=range_maxes[motor],
        #     )

        # self.bus.write_calibration(self.calibration)
        # self._save_calibration()
        # print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        pass
        # with self.bus.torque_disabled():
        #     self.bus.configure_motors()
        #     for motor in self.bus.motors:
        #         self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        #         # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
        #         self.bus.write("P_Coefficient", motor, 16)
        #         # Set I_Coefficient and D_Coefficient to default value 0 and 32
        #         self.bus.write("I_Coefficient", motor, 0)
        #         self.bus.write("D_Coefficient", motor, 32)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()

        obs_dict = {}
        for motor_name, motor in self.motors.items():
            self.control.refresh_motor_status(motor)
            obs_dict[f"{motor_name}.pos"] = motor.getPosition()
        
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any], velocity: float = 1.0) -> dict[str, Any]:
        """Command arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            the action sent to the motors, potentially clipped.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send goal position to the arm        
        for motor_name, motor in self.motors.items():
            self.control.control_Pos_Vel(motor, P_desired=goal_pos[motor_name], V_desired=velocity)
        
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for motor in self.motors.values():
            self.control.disable(motor)

        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")