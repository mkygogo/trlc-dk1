from trlc_dk1.follower import DK1Follower, DK1FollowerConfig
from trlc_dk1.leader import DK1Leader, DK1LeaderConfig
import time


follower_config = DK1FollowerConfig(
    port="/dev/tty.usbmodem00000000050C1",
    disable_torque_on_disconnect=True,
)

leader_config = DK1LeaderConfig(
    port="/dev/tty.usbmodem58FA0824311"
)

leader = DK1Leader(leader_config)
leader.connect()

follower = DK1Follower(follower_config)
follower.connect()


freq = 100
duration = 60

for i in range(duration*freq):
    action = leader.get_action()
    formatted_action = {key: f"{val:.2f}" for key, val in action.items()}
    print(formatted_action)
    follower.send_action(action, V_desired=6.0)    
    time.sleep(1/freq)
    