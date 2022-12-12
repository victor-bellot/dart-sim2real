from dartv2b_simu import DartV2
import time

if __name__ == "__main__":
    my_bot = DartV2()

    # place your work here
    my_bot.set_speed(100, -100)
    time.sleep(2.0)
    my_bot.set_speed(0, 0)
    
    my_bot.end()  # clean end of the robot mission
