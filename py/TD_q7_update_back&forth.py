from dartv2b_simu import DartV2
import time

if __name__ == "__main__":
    my_bot = DartV2()

    # place your work here
    print("Encoder Microcode Version : ", my_bot.encoders.get_version())
    print("Battery Voltage before : %.2f V" % (my_bot.encoders.battery_voltage()))
    print("Rear encoders before : ", my_bot.get_rear_odos())

    for _ in range(2):
        my_bot.set_speed(100, 100)
        for i in range(10):
            print("Rear encoders [L,R]", my_bot.get_rear_odos())
            time.sleep(0.5)
        my_bot.set_speed(100, -100)
        time.sleep(1.31)  # empirical !! may change with cpu !!!
    my_bot.set_speed(0, 0)

    print("Rear encoders after : ", my_bot.get_rear_odos())

    deltaOdoLeft = my_bot.delta_rear_odometers(side="left")
    deltaOdoRight = my_bot.delta_rear_odometers(side="right")

    print("Battery Voltage after : %.2f V" % (my_bot.encoders.battery_voltage()))
    print("Delta odometer left :", deltaOdoLeft)
    print("Delta odometer right :", deltaOdoRight)
    print("Delta odometers :", my_bot.delta_rear_odometers())
    
    my_bot.end()  # clean end of the robot mission
