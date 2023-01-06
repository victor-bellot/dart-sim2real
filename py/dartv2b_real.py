from drivers.dartv2b_basis import DartV2Basis
from drivers.sonars import SonarsFilter
from tools import *
import numpy as np
import time
import sys


class DartV2(DartV2Basis):
    def __init__(self):
        # get init from parent class
        # drivers.dartv2b_basis.DartV2Basis.__init__(self)
        super().__init__()
        self.flt = SonarsFilter()

        self.turnCount = None
        self.dt = 0.05

        self.calibrationRevCount = 1  # number of revolution to calibrate compass
        self.minStageDuration = 1  # minimum amount of time (in seconds) before leaving
        self.longStageDuration = 4  # minimum amount of time (in seconds) to be in long corridors

        self.spdMin = 60  # minimum wheels speed
        self.followCapSpd = 70  # wheels speed for follow_cap
        self.followWallsSpd = 70  # wheels speed for follow_walls

        self.stopDistance = 0.42  # distance in meters to wall to stop
        self.centerToWall = 0.25  # distance to keep away from walls
        self.angularAccuracy = 3  # angular precision in degrees

        self.staCapConst = 0.3  # proportional constant for turns
        self.dynCapConst = self.followCapSpd * (2 / 90)  # proportional constant for cap regulation

        self.obsRegConst = 10  # proportional constant for forward speed regulation

        self.followWallsP = 50  # proportional constant for follow walls regulation
        self.followWallsD = 100 / self.dt  # derivative constant for follow walls regulation

        self.refHeadingDeg = None  # estimate of west heading in degrees
        self.halfRaceCount = 0  # number of half race already done

        self.sonars.init_4_sonars()  # initialize cardinals sonars in synchronous mode

    def fast_compass_calibration(self):
        """
        Hard code compass calibration from an other program
        """

        print("Fast compass calibration...")

        # mag_x_min, mag_x_max, mag_y_min, mag_y_max = -1238, 2447, -4757, -1242
        mag_x_min, mag_x_max, mag_y_min, mag_y_max = -1202, 2347, -4598, -1245
        self.imu.fast_heading_calibration(mag_x_min, mag_x_max, mag_y_min, mag_y_max)

    def follow_walls(self):
        """
        Follow walls until an obstacle is encountered
        """

        print("Go forward, following walls, until an obstacle is encounter.")

        t_init = time.time()
        spd_ask = self.followWallsSpd
        delta_dist_old = 0
        headings = []

        stage_in_progress = True
        self.set_speed(spd_ask, spd_ask)

        while stage_in_progress:
            t0 = time.time()

            dist_left, dist_front, dist_right = self.get_some_sonars(['left', 'front', 'right'])
            dist_center = dist_front - self.centerToWall
            # print('LEFT FRONT RIGHT', dist_left, dist_front, dist_right)

            headings.append(self.imu.heading_deg())

            if dist_front <= self.stopDistance and t0 - t_init > self.minStageDuration:
                stage_in_progress = False
            else:
                spd = max(min(spd_ask, self.obsRegConst * dist_center), self.spdMin)

                far_left = dist_left > 2 * self.centerToWall
                far_right = dist_right > 2 * self.centerToWall
                if far_left and far_right:
                    delta_dist = 0
                elif far_left:
                    delta_dist = self.centerToWall - dist_right
                elif far_right:
                    delta_dist = dist_left - self.centerToWall
                else:
                    delta_dist = (dist_left - dist_right) / 2

                delta_spd = self.followWallsP * delta_dist + self.followWallsD * (delta_dist - delta_dist_old)
                delta_dist_old = delta_dist
                self.set_speed(spd - delta_spd, spd + delta_spd)

                delta_time = time.time() - t0
                sleep_time = self.dt - delta_time
                if sleep_time > 0:
                    # print("Time left: ", sleep_time)
                    time.sleep(sleep_time)

        mission_duration = time.time() - t_init
        if mission_duration > self.longStageDuration:
            self.refHeadingDeg = (np.median(headings) + self.halfRaceCount * 180) % 360
            self.halfRaceCount += 1

        self.stop()

    def turn_compass(self, cap):
        """
        Turn toward a given cap
        """

        print("Turn using compass toward: ", cap)

        def sens_and_norm():
            # delta_heading is between -180° and +180°
            heading = self.imu.heading_deg()
            delta = (heading - cap) % 360
            delta = delta - 360 if delta > 180 else delta
            return sign_and_norm(delta)

        stage_in_progress = True

        while stage_in_progress:
            t0 = time.time()

            s, n = sens_and_norm()
            if n < self.angularAccuracy:
                stage_in_progress = False
            else:
                spd = s * (self.spdMin + n * self.staCapConst)
                self.set_speed(spd, -spd)

                delta_time = time.time() - t0
                sleep_time = self.dt - delta_time
                if sleep_time > 0:
                    # print("Time left: ", sleep_time)
                    time.sleep(sleep_time)

        self.stop()

    def turn_left(self):
        direction = self.get_direction()
        self.turn_compass(direction + 90)

    def turn_right(self):
        direction = self.get_direction()
        self.turn_compass(direction - 90)

    def get_direction(self):
        """
        Return the estimate of the nearest cardinal heading
        """

        heading = self.imu.heading_deg()
        print("HEADING", heading)
        return self.refHeadingDeg + round_direction(heading - self.refHeadingDeg)

    def get_free_turn(self):
        """
        Return the turn to do to go toward the free direction
        -> which way to go
        """

        # Distances are given in meters
        dist_left, dist_right = self.get_some_sonars(['left', 'right'])
        # print("LEFT RIGHT", dist_left, dist_right)

        if dist_left < 99.9 and dist_right < 99.9:
            if dist_left < dist_right:
                return 'right'
            else:
                return 'left'
        else:
            if dist_left < 99.9:
                return 'right'
            elif dist_right < 99.9:
                return 'left'
        return None

    def get_some_sonars(self, names=None):
        """
        Return sonars values of the given sonar names
        """

        self.update_cardinal_sonars()
        distances = self.flt.median_filter()
        sonar_keys = SonarsFilter.sonar_keys

        if names:
            return [distances[sonar_keys.index(name)] for name in names]
        else:
            return distances

    def wait(self, duration):
        print("Do nothing for", duration, "s")
        time.sleep(duration)

    def half_turn(self):
        print("Make an half-turn")
        self.set_speed(100, -100)
        time.sleep(1.31)  # empirical
        self.stop()

    def battery_voltage(self):
        return self.encoders.battery_voltage()

    def get_selected_odos(self, front_odos=False):
        if front_odos:
            odol, odor = self.get_front_odos()
        else:
            odol, odor = self.get_rear_odos()
        return odol, odor

    def get_all_odos(self):
        odofl, odofr = self.get_front_odos()
        odorl, odorr = self.get_rear_odos()
        return odofl, odofr, odorl, odorr

    def get_front_odos(self):
        """
        DART wheels are 12 cm diameter
        Virtual DART makes 200 ticks per wheel revolution
        Read from T-REX (xx ticks per revolution)
        return : odometer values (left and right) on 16 bits signed (-32768 to - 32767)
        """
        odofl, odofr = self.get_front_encoders()
        self.update_front_encoders(time.time(), odofl, odofr)
        return odofl, odofr

    def get_rear_odos(self):
        odorl, odorr = self.get_rear_encoders()
        while odorl < 0:
            time.sleep(0.0005)
            print("------ error I2C odorl")
            odorl, dummy = self.get_rear_encoders()
        while odorr < 0:
            time.sleep(0.0005)
            print("------ error I2C odorr")
            dummy, odorr = self.get_rear_encoders()
        self.update_rear_encoders(time.time(), odorl, odorr)
        return odorl, odorr

    def delta_odometers_without_jumps(self, odo_mem, odo_last):
        deltaOdo = odo_last - odo_mem  # last=new and mem=old
        if deltaOdo > 32767:  # check if high positive jumps !!
            deltaOdo -= 65536  # remove the jump (2^16)
        if deltaOdo < -32767:  # same for high negative jumps
            deltaOdo += 65536  # remove the jump (2^16)
        return deltaOdo

    def delta_front_odometers(self, side="both"):
        if side == "both":
            deltaOdoLeft = self.delta_odometers_without_jumps(
                self.encoders_front_left_mem, self.encoders_front_left_last)
            deltaOdoRight = self.delta_odometers_without_jumps(
                self.encoders_front_right_mem, self.encoders_front_right_last)
            return [deltaOdoLeft, deltaOdoRight]
        elif side == "left":
            deltaOdoLeft = self.delta_odometers_without_jumps(
                self.encoders_front_left_mem, self.encoders_front_left_last)
            return deltaOdoLeft
        if side == "right":
            deltaOdoRight = self.delta_odometers_without_jumps(
                self.encoders_front_right_mem, self.encoders_front_right_last)
            return deltaOdoRight

    def delta_rear_odometers(self, side="both"):
        if side == "both":
            deltaOdoLeft = self.delta_odometers_without_jumps(
                self.encoders_rear_left_mem, self.encoders_rear_left_last)
            deltaOdoRight = self.delta_odometers_without_jumps(
                self.encoders_rear_right_mem, self.encoders_rear_right_last)
            return [-deltaOdoLeft, -deltaOdoRight]
        elif side == "left":
            deltaOdoLeft = self.delta_odometers_without_jumps(
                self.encoders_rear_left_mem, self.encoders_rear_left_last)
            return -deltaOdoLeft
        if side == "right":
            deltaOdoRight = self.delta_odometers_without_jumps(
                self.encoders_rear_right_mem, self.encoders_rear_right_last)
            return -deltaOdoRight

    def update_cardinal_sonars(self):
        df, dl, db, dr = self.sonars.read_4_sonars()
        # print ("df,dl,db,dr",df,dl,db,dr)
        df = set_sonar_0_to_99(df / 100.0)
        dl = set_sonar_0_to_99(dl / 100.0)
        db = set_sonar_0_to_99(db / 100.0)
        dr = set_sonar_0_to_99(dr / 100.0)

        self.flt.add_measures((df, dl, db, dr))
        self.write_cardinal_sonars(time.time(), df, dl, db, dr)

    def get_diagonal_sonars(self):
        dl, dr = self.sonars.read_diag_all()
        self.update_diagonal_sonars(time.time(), dl, dr)
        return dl, dr


if __name__ == "__main__":
    print("start")
    if len(sys.argv) < 3:
        print("needs to fix left and right speeds :")
        print("python3 dart speedLeft speedRight")
        print("exit!")
        exit()

    myDart = DartV2()

    speedL = int(sys.argv[1])
    speedR = int(sys.argv[2])

    print("Sonar version : ", myDart.sonars.get_version())
    """
    mode = 2
    dmax = 1.5
    for isn in range(4):
        #print ("Mode",isn+1,":","%2.2X"%(myDart.sonars.get_mode(isn+1)))
        myDart.sonars.set_mode(isn+1,mode)
        myDart.sonars.set_dist_max(isn+1,dmax)
    myDart.sonars.set_dist_max(5,dmax)
    for isn in range(4):
        #print ("Mode",isn+1,":","%2.2X"%(myDart.sonars.get_mode(isn+1)))
        pass
    for isn in range(4):
        print ("State",isn+1,":","%2.2X"%(myDart.sonars.get_state(isn+1)))
    for itst in range(5):
        time.sleep(0.5)
        print ("Sonars cardinal",myDart.sonars.read_4_sonars())
        print ("Sonars front diag",myDart.sonars.read_diag_both())
        print ("Sonars front bytes",myDart.sonars.read_front_bytes())
        print ("Sonars diag word",myDart.sonars.read_diag_left_word(),myDart.sonars.read_diag_right_word())
    """
    print("Encoder version : ", myDart.encoders.get_version())
    print("Battery voltage :", myDart.encoders.battery_voltage())
    print("Motor directions :", myDart.encoders.read_motors_direction())

    myDart.encoders.reset_both()
    encs_front_0 = myDart.get_front_encoders()
    encs_rear_0 = myDart.encoders.read_encoders()

    print("Encoders (Front T-REX)) :", encs_front_0)
    print("Encoders :", encs_rear_0)

    myDart.set_speed(speedL, speedR)
    time.sleep(1.0)

    print("Encoders :", myDart.encoders.read_encoders())
    # myDart.encoders.reset_left()
    time.sleep(1.0)
    print("Encoders :", myDart.encoders.read_encoders())
    # myDart.encoders.reset_right()
    time.sleep(1.0)
    print("Encoders :", myDart.encoders.read_encoders())
    # myDart.encoders.reset_both()
    time.sleep(1.0)
    print("Encoders :", myDart.encoders.read_encoders())
    print("Encoders (Front T-REX)) :", myDart.get_front_encoders())
    encs_front_0 = myDart.get_front_encoders()
    encs_rear_0 = myDart.encoders.read_encoders()
    print("Encoders (Front T-REX)) :", encs_front_0)
    print("Encoders :", encs_rear_0)
    """
    print (myDart.imu.read_mag_raw())
    """
    myDart.set_speed(0, 0)
    time.sleep(0.05)
    myDart.end()