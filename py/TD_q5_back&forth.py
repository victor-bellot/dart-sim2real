from dartv2b import DartV2
import time

if __name__ == "__main__":
    my_bot = DartV2()

    # place your work here
    print("Front encoders before : ", my_bot.get_front_odos())

    for ileg in range(2):
        my_bot.set_speed(100, 100)
        for i in range(10):
            print("Front encoders [L,R]", my_bot.get_front_odos())
            time.sleep(0.5)
        my_bot.set_speed(100, -100)
        time.sleep(1.31)  # empirical !! may change with cpu !!!
        my_bot.set_speed(0, 0)

    odo_left, odo_right = my_bot.get_front_odos()
    
    print("Front encoders after : ", [odo_left, odo_right])

    deltaOdoLeft = my_bot.delta_front_odometers(side="left")
    deltaOdoRight = my_bot.delta_front_odometers(side="right")

    print("Delta odometer left :", deltaOdoLeft)
    print("Delta odometer right :", deltaOdoRight)
    print("Delta odometers :", my_bot.delta_front_odometers())
    
    my_bot.end()  # clean end of the robot mission
