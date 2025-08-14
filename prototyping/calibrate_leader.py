from dk1.dk1_leader import DK1LeaderConfig, DK1Leader
import time
import numpy as np

leader_config = DK1LeaderConfig(
    port="/dev/tty.usbmodem58FA0824281",
)

leader = DK1Leader(leader_config)
leader.connect()

directions = {
    "joint_1": 1,
    "joint_2": -1,
    "joint_3": -1,
    "joint_4": 1,
    "joint_5": 1,
    "joint_6": 1,
}


while True:
    # action = leader.bus.sync_read(normalize=False, data_name="Present_Position")
    # action = {f"{motor}.pos": (val/4096*360-180)*directions[motor] for motor, val in action.items()}
    # print(action)
    # time.sleep(0.01)
    action = leader.get_action()
    print(action)
    time.sleep(0.01)

