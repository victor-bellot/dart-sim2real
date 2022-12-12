from dartv2b_real import DartV2
import time

if __name__ == "__main__":
    my_bot = DartV2()

    # place your work here
    print("Front encoders before : ", my_bot.get_front_odos())
    
    my_bot.set_speed(100, -100)
    time.sleep(2.0)
    my_bot.set_speed(0, 0)

    odo_left, odo_right = my_bot.get_front_odos()
    
    print("Front encoders after : ", [odo_left, odo_right])

    deltaOdoLeft = my_bot.delta_front_odometers(side="left")
    deltaOdoRight = my_bot.delta_front_odometers(side="right")

    print("Delta odometer left :", deltaOdoLeft)
    print("Delta odometer right :", deltaOdoRight)
    print("Delta odometers :", my_bot.delta_front_odometers())
    
    my_bot.end()  # clean end of the robot mission
