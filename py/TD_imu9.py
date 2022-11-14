from dartv2b import DartV2
import time

if __name__ == "__main__":
    my_bot = DartV2()

    print('Starting...')
    my_bot.calibration_compass()
    print('Calibrated!')

    my_bot.set_speed(-75, 75)
    for _ in range(10):
        heading = my_bot.imu.heading_deg()
        print(heading)
        time.sleep(1)

    my_bot.stop()
    my_bot.end()  # clean end of the robot mission
