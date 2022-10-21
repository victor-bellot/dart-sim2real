from dartv2b import DartV2
import time

if __name__ == "__main__":
    my_bot = DartV2()

    # place your work here

    print("Battery voltage before : ", my_bot.encoders.battery_voltage())
    print("Front encoders before : ", my_bot.get_front_odos())
    print("Rear encoders after : ", my_bot.encoders.read_encoders())

    my_bot.set_speed(100, -100)
    time.sleep(2.0)
    my_bot.set_speed(0, 0)

    print("Battery voltage after : ", my_bot.encoders.battery_voltage())
    print("Front encoders after : ", my_bot.get_front_odos())
    print("Rear encoders after : ", my_bot.encoders.read_encoders())

    my_bot.end()  # clean end of the robot mission
