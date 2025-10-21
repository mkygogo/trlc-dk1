# from trlc_dk1.follower import DK1Follower, DK1FollowerConfig
# from trlc_dk1.leader import DK1Leader, DK1LeaderConfig
from trlc_dk1.follower_new import DK1Follower, DK1FollowerConfig
from leader_Feetech import DK1Leader, DK1LeaderConfig
import time


follower_config = DK1FollowerConfig(
    port="/dev/ttyACM1",
    joint_velocity_scaling=1.0,
)

leader_config = DK1LeaderConfig(
    port="/dev/ttyACM0"
)

leader = DK1Leader(leader_config)
leader.connect()

follower = DK1Follower(follower_config)
follower.connect()

freq = 200 # Hz

def action_filter(action):
    new_action = {'joint_1.pos': action["joint_1.pos"], 
                'joint_2.pos': 0, 
                'joint_3.pos': 0, 
                'joint_4.pos': 0, 
                'joint_5.pos': 0, 
                'joint_6.pos': 0}

    return new_action

try:
    while True:
        action = leader.get_action()

        action = action_filter(action)
        print(action)

        follower.send_action(action)    
        time.sleep(1/freq)
except KeyboardInterrupt:
    print("\nStopping teleop...")
    leader.disconnect()
    follower.disconnect()
