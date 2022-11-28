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
    """ Normalize between -180° and +180° """
    angle = angle % 360  # between 0° and 360°
    if angle > 180:
        return angle - 360
    return angle


def set_sonar_0_to_99(d):
    if d == 0.0:
        d = 99.9
    return d
