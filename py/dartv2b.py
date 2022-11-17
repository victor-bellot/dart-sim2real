import drivers.dartv2b_basis
from tools import *
import sys
import time


class DartV2(drivers.dartv2b_basis.DartV2Basis):
    def __init__(self):
        # get init from parent class
        # drivers.dartv2b_basis.DartV2Basis.__init__(self)
        super().__init__()

        # place your new class variables here
        self.turn_count = None
        self.spdMin = 70
        self.nominalSpd = 200

        self.sonars.init_4_sonars()  # initialize cardinals sonars in synchronous mode

    # place your new functions here OK

    def calibration_compass(self):
        k = 0  # count the number of changes encounter
        near = True  # tell if this robot is near it's initial heading
        eps = math.pi / 10  # 1/20 revolution precision

        mag_x, mag_y, _ = self.imu.read_mag_raw()
        init_head = mag2heading(mag_x, mag_y)  # between -pi and +pi

        mag_xs, mag_ys = [], []
        self.set_speed(-75, 75)

        while k < 2:
            mag_x, mag_y, _ = self.imu.read_mag_raw()
            mag_xs.append(mag_x)
            mag_ys.append(mag_y)

            # a % b is in [0; b[
            head = mag2heading(mag_x, mag_y)
            new_near = abs((head - init_head) % pix2) < eps
            if near ^ new_near:
                k += 1
            near = new_near
            time.sleep(0.05)

        self.stop()

        mag_x_min, mag_x_max = min(mag_xs), max(mag_xs)
        mag_y_min, mag_y_max = min(mag_ys), max(mag_ys)

        self.imu.fast_heading_calibration(mag_x_min, mag_x_max, mag_y_min, mag_y_max)

    def wait(self, duration):
        print("Do nothing for", duration, "s")
        time.sleep(duration)

    def go_straight_to_obs_compass(self):
        """
        Go straight toward rb direction using the compass
        until an obstacle is encountered

        input parameters :
        None

        output parameters :
        None
        """

        print("Go forward, following a cap, until an obstacle is encounter.")

        spd_ask = self.nominalSpd  # nominal speed
        center_to_wall = 25  # distance in centimeters to wall to stop

        turn_dyn_const = 120 / 90  # proportional constant for cap regulation
        straight_const = 5  # proportional constant for forward speed regulation

        t_init = time.time()
        min_duration = 1  # minimum amount of time before leaving
        time_step = 0.1  # duration of each iteration
        direction = round_direction(self.imu.heading_deg())

        stage_in_progress = True
        self.set_speed(spd_ask, spd_ask)

        while stage_in_progress:
            t0 = time.time()

            dist_front, = self.get_sonars(['front'])
            dist_center = dist_front - center_to_wall

            current_angle = self.imu.heading_deg()
            delta = normalize_angle(current_angle - direction)
            delta_spd = turn_dyn_const * delta

            if dist_front <= center_to_wall and t0 - t_init > min_duration:
                stage_in_progress = False
            else:
                spd = max(min(spd_ask, straight_const * dist_center), self.spdMin)
                self.set_speed(spd - delta_spd, spd + delta_spd)

                delta_time = time.time() - t0
                sleep_time = time_step - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        self.stop()

    def get_free_direction(self):
        """
        Return the orientation of the free direction

        input parameters :
        None

        output parameters :
        direction : which way to go
        """

        dist_left, dist_right = self.get_sonars(['left', 'right'])

        if dist_left < 9990 and dist_right < 9990:
            if dist_left < dist_right:
                return 'right'
            else:
                return 'left'
        else:
            if dist_left < 9990:
                return 'right'
            elif dist_right < 9990:
                return 'left'
        return None

    def get_sonars(self, names=None):
        name2index = {'front': 0, 'left': 1, 'back': 2, 'right': 3}

        values = self.sonars.read_4_sonars()
        distances = [9990 if distance < 1 else distance for distance in values]

        if names:
            return [distances[name2index[name]] for name in names]
        else:
            return distances

    def half_turn(self):
        print("Make an half-turn")
        self.set_speed(100, -100)
        time.sleep(1.31)  # empirical
        self.stop()

    def turn_compass(self, cap):
        """
        Turn toward cap

        input parameters :
        cap : angle in degrees

        output parameters :
        None
        """
        print("Turn using compass toward: ", cap)
        eps = 3  # precision in degrees
        k = 1

        def sens_and_norm():
            # delta_heading is between -180° and +180°
            heading = self.imu.heading_deg()
            delta = (heading - cap) % 360
            delta = delta - 360 if delta > 180 else delta
            return sign_and_norm(delta)

        while sens_and_norm()[1] > eps:
            s, n = sens_and_norm()
            spd = s * (self.spdMin + n * k)
            self.set_speed(-spd, spd)
            time.sleep(0.05)

        self.stop()

    def turn_left(self):
        direction = round_direction(self.imu.heading_deg())
        self.turn_compass(direction - 90)

    def turn_right(self):
        direction = round_direction(self.imu.heading_deg())
        self.turn_compass(direction + 90)

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

    def set_sonar_0_to_99(self, d):
        if d == 0.0:
            d = 99.9
        return d

    def get_cardinal_sonars(self):
        df, dl, db, dr = self.sonars.read_4_sonars()
        # print ("df,dl,db,dr",df,dl,db,dr)
        df = self.set_sonar_0_to_99(df / 100.0)
        dl = self.set_sonar_0_to_99(dl / 100.0)
        db = self.set_sonar_0_to_99(db / 100.0)
        dr = self.set_sonar_0_to_99(dr / 100.0)
        self.update_cardinal_sonars(time.time(), df, dl, db, dr)
        return df, dl, db, dr

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
