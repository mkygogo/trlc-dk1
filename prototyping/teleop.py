from dk1.dk1_follower import DK1FollowerConfig, DK1Follower
from dk1.dk1_leader import DK1LeaderConfig, DK1Leader
import time

follower_config = DK1FollowerConfig(
    port="/dev/tty.usbmodem00000000050C1",
    id="my_red_robot_arm",
    cameras={},
)

leader_config = DK1LeaderConfig(
    port="/dev/tty.usbmodem58FA0824281",
)

follower = DK1Follower(follower_config)
follower.connect()

leader = DK1Leader(leader_config)
leader.connect()

homing_position = {
    "joint_1.pos": 0.0,
    "joint_2.pos": 0.0,
    "joint_3.pos": 0.0,
    "joint_4.pos": 0.0,
    "joint_5.pos": 0.0,
    "joint_6.pos": 0.0
}

frequency = 200 # Hz


follower.send_action(homing_position, velocity=0.5)
time.sleep(3)


for i in range(10*frequency):
    action = leader.get_action()
    print(action)
    follower.send_action(action, velocity=4.0)
    time.sleep(1/frequency)
    

follower.send_action(homing_position, velocity=0.5)
time.sleep(3)
follower.disconnect()
