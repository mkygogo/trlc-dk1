from dk1.dk1_leader import DK1Leader, DK1LeaderConfig


config = DK1LeaderConfig(port="/dev/tty.usbmodem58FA0824311")

def main():
    leader = DK1Leader(config)
    # leader.connect()
    # leader.configure()
    leader.setup_motors()
    # leader.disconnect()

if __name__ == "__main__":
    main()