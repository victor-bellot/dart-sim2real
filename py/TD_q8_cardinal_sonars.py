import dartv2b_real
import time

if __name__ == "__main__":
    my_bot = dartv2b_real.DartV2()

    # place your work here
    print("Sonar Microcode Version : ", my_bot.sonars.get_version())
    print("Battery Voltage : %.2f V" % (my_bot.encoders.battery_voltage()))

    # set range max to 1.5 m
    dmax = 1.5
    for isn in range(4):
        my_bot.sonars.set_dist_max(isn+1, 2.0)  # 2.O m for cardinals
    my_bot.sonars.set_dist_max(5, dmax)  # 1.5 m for diagonals

    # test mode 2 on 4 sonars
    print("test mode 2 (sync)  ...")
    mode = 2
    for isn in range(4):
        my_bot.sonars.set_mode(isn+1, mode)
    for itst in range(10): 
        time.sleep(0.1)
        print("Sonars cardinal (front, left, back, right)", my_bot.sonars.read_4_sonars())

    # test mode 1 on front sonar
    print("test mode 1 (one shot) on front and rear ...")
    mode = 0   # stop all
    for isn in range(4):
        my_bot.sonars.set_mode(isn+1, mode)

    mode = 1
    for itst in range(100): 
        for isn in range(4):
            my_bot.sonars.set_mode(isn+1, mode)
        time.sleep(0.5)
        print("Sonars front,rear,left,right", my_bot.sonars.read_front(), my_bot.sonars.read_rear(),
              my_bot.sonars.read_left(), my_bot.sonars.read_right())
    my_bot.end()  # clean end of the robot mission
