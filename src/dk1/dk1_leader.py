from dataclasses import dataclass
import logging
import time
import numpy as np

from lerobot.teleoperators.teleoperator import Teleoperator, TeleoperatorConfig
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import (
    DriveMode,
    DynamixelMotorsBus,
    OperatingMode,
)

logger = logging.getLogger(__name__)


@TeleoperatorConfig.register_subclass("dk1_leader")
@dataclass
class DK1LeaderConfig(TeleoperatorConfig):
    port: str
    use_degrees: bool = True
    gripper_open_position: int = 2290
    gripper_closed_position: int = 1667
    
    
class DK1Leader(Teleoperator):
    config_class = DK1LeaderConfig
    name = "dk1_leader"

    def __init__(self, config: DK1LeaderConfig):
        super().__init__(config)
        self.config = config
        self.bus = DynamixelMotorsBus(
            port=self.config.port,
            motors={
                "joint_1": Motor(1, "xl330-m077", MotorNormMode.DEGREES),
                "joint_2": Motor(2, "xl330-m077", MotorNormMode.DEGREES),
                "joint_3": Motor(3, "xl330-m077", MotorNormMode.DEGREES),
                "joint_4": Motor(4, "xl330-m077", MotorNormMode.DEGREES),
                "joint_5": Motor(5, "xl330-m077", MotorNormMode.DEGREES),
                "joint_6": Motor(6, "xl330-m077", MotorNormMode.DEGREES),
                "gripper": Motor(7, "xl330-m077", MotorNormMode.DEGREES),
            },
        )
        print(self.calibration_dir)

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        # if not self.is_calibrated and calibrate:
        #     logger.info(
        #         "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
        #     )
        #     self.calibrate()
        

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        pass

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        self.bus.disable_torque()
        self.bus.configure_motors()
        
        self.bus.write("Torque_Enable", "gripper", 0, normalize=False)
        self.bus.write("Operating_Mode", "gripper", OperatingMode.CURRENT_POSITION.value, normalize=False)
        self.bus.write("Current_Limit", "gripper", 100, normalize=False)
        self.bus.write("Torque_Enable", "gripper", 1, normalize=False)
        self.bus.write("Goal_Position", "gripper", self.config.gripper_open_position, normalize=False)

        # Use 'position control current based' for gripper to be limited by the limit of the current.
        # For the follower gripper, it means it can grasp an object without forcing too much even tho,
        # its goal position is a complete grasp (both gripper fingers are ordered to join and reach a touch).
        # For the leader gripper, it means we can use it as a physical trigger, since we can force with our finger
        # to make it move, and it will move back to its original target position when we release the force.
        # self.bus.write("Operating_Mode", "gripper", OperatingMode.CURRENT_POSITION.value)
        # Set gripper's goal pos in current position mode so that we can use it as a trigger.
        # self.bus.enable_torque("gripper")
        # if self.is_calibrated:
        #     self.bus.write("Goal_Position", "gripper", self.config.gripper_open_pos)

    def setup_motors(self) -> None:
        for motor in self.bus.motors:
        # for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        #action = self.bus.sync_read("Present_Position")
        
        directions = {
            "joint_1": 1,
            "joint_2": 1,
            "joint_3": 1,
            "joint_4": 1,
            "joint_5": 1,
            "joint_6": 1,
        }


        action = self.bus.sync_read(normalize=False, data_name="Present_Position")
        # action = {f"{motor}.pos": (val/4096*360-180)*directions[motor] for motor, val in action.items()}
        
        action = {f"{motor}.pos": (val/4096*2*np.pi-np.pi)*directions[motor] if motor != "gripper" else val for motor, val in action.items()}
        # Normalize gripper position between 1 (closed) and 0 (open)
        gripper_range = self.config.gripper_open_position - self.config.gripper_closed_position
        action["gripper.pos"] = 1 - (action["gripper.pos"] - self.config.gripper_closed_position) / gripper_range
        # action["gripper.pos"] = action["gripper.pos"] * 100
        
        
        #action = {f"{motor}.pos": val for motor, val in action.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        logger.info(f"{self} disconnected.")
