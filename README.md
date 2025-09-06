
<p align="center">
    <img src="media/xray.jpg">
</p>
<p align="center">
    <a href="https://docs.robot-learning.co/">
        <img src="https://img.shields.io/badge/Documentation-📕-blue" alt="Chat on Discord">        
    </a>
    <a href="https://discord.gg/PTZ3CN5WkJ">
        <img src="https://img.shields.io/discord/1409155673572249672?color=7289DA&label=Discord&logo=discord&logoColor=white">
    </a>
    <a href="https://x.com/JannikGrothusen">
        <img src="https://img.shields.io/twitter/follow/Jannik?style=social">
    </a>
</p>

<h2 align="center">TRLC-DK1: An Open Source Dev Kit for AI-native Robotics</h2>

### Demo
<p align="center">
    <img src="media/demo.gif">
</p>

### Examples
```bash
lerobot-teleoperate \
    --robot.type=dk1_follower \
    --robot.port=/dev/tty.usbmodem00000000050C1 \
    --robot.joint_velocity_scaling=1.0 \
    --robot.cameras="{ wrist: {type: opencv, index_or_path: 0, width: 640, height: 360, fps: 30}}" \
    --teleop.type=dk1_leader \
    --teleop.port=/dev/tty.usbmodem58FA0824311 \
    --display_data=true
```
```
lerobot-teleoperate \
    --robot.type=dk1_follower \
    --robot.port=/dev/tty.usbmodem00000000050C1 \
    --robot.joint_velocity_scaling=1.0 \
    --teleop.type=dk1_leader \
    --teleop.port=/dev/tty.usbmodem58FA0824311 \
```