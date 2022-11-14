import math


def mag2heading(mag_x, mag_y):
    return math.atan2(mag_y, mag_x)
