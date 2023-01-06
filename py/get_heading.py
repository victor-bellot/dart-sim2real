from dartv2b_simu import DartV2
import time
import sys
import math


def delta_heading(head_ref,head):
    head_err = head_ref-head
    while head_err > 180.0:
        head_err -= 360.0
    while head_err <= -180.0:
        head_err += 360.0
    # print ("heading error",head_ref,head,head_ref-head,head_err)
    return head_err


if __name__ == "__main__":
    my_bot = DartV2()

    duration = 1.0
    try:
        duration = float(sys.argv[1])
    except:
        pass

    dt = 0.1
    try:
        dt = float(sys.argv[2])
    except:
        pass

    # set fast mag calib data DART02
    magx_min = 902
    magx_max = 3894
    magy_min = -3510
    magy_max = -731
    # set fast mag calib data DART04
    magx_min = -825
    magx_max = 3088
    magy_min = -4548
    magy_max = -746

    # DART02 - 2021/12/2
    magx_min = 779
    magx_max = 4088
    magy_min = -3622
    magy_max = -553

    # DART05 - 2022/09/23
    magx_min = -187
    magx_max = 2207
    magy_min = -2035
    magy_max = 234

    # DART06 - 2022/11/25
    magx_min = 957
    magx_max = 4933
    magy_min = -6478
    magy_max = -2659

    # Simulated DART
    # magx_min = 0
    # magx_max = 1000
    # magy_min = 0
    # magy_max = 1000

    mag_x_min, mag_x_max, mag_y_min, mag_y_max = -1238, 2447, -4757, -1242
    my_bot.imu.fast_heading_calibration(magx_min, magx_max, magy_min, magy_max)

    print("Battery Voltage : %.2f V" % (my_bot.encoders.battery_voltage()))

    t0 = time.time()
    head_err_i = 0.0
    while True:
        if time.time()-t0 >= duration:
            break
        print("heading : %.2f"%(my_bot.imu.heading_deg()))
        time.sleep(dt)
    print("Battery Voltage : %.2f V"%(my_bot.encoders.battery_voltage()))

    my_bot.end()  # clean end of the robot mission
