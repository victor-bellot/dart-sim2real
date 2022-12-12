from dartv2b_real import DartV2

if __name__ == "__main__":
    my_bot = DartV2()

    print('Starting...')
    my_bot.calibration_compass()
    print('Calibrated!')

    my_bot.go_straight_to_obs_compass()

    my_bot.end()  # clean end of the robot mission
