import dartv2b
import time

if __name__ == "__main__":
    my_bot = dartv2b.DartV2()

    # place your work here
    print("Sonar Microcode Version : ", my_bot.sonars.get_version())
    print("Battery Voltage : %.2f V" % (my_bot.encoders.battery_voltage()))

    my_bot.set_speed(100, -100)
    time.sleep(0.5)
    my_bot.set_speed(0, 0)

    for itst in range(10): 
        time.sleep(0.5)
        print("Sonars front diag", my_bot.sonars.read_diag_both())

    my_bot.end()  # clean end of the robot mission
