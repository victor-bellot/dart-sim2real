import math

pix2 = 2 * math.pi


def mag2heading(mag_x, mag_y):
    return math.atan2(mag_y, mag_x)


def sign_and_norm(value):
    n = abs(value)
    if n > 0:
        return value / n, n
    else:
        return 1, 0


def round_direction(angle):
    return round(angle / 90) * 90


def normalize_angle(angle):
    angle = angle % 360  # between 0° and 360°
    if angle > 180:
        return angle - 360
    return angle
