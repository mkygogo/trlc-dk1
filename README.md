![DK1 Robot](https://github.com/robot-learning-co/trlc-dk1/blob/main/media/dk1.png)

# TRLC-DK1
Open-source **D**evelopment **K**it for AI-native Robotics 

### Development
Teleoperate a single leader-follower setup:
```
lerobot-teleoperate \
--robot.type=dk1_follower \
--robot.port=/YOUR/FOLLOWER_PORT \
--robot.id=my_awesome_follower_arm \
--robot.cameras="{wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
--teleop.type=dk1_leader \
--teleop.port=/YOUR/LEADER_PORT \
--teleop.id=my_awesome_leader_arm \
--display_data=true
```