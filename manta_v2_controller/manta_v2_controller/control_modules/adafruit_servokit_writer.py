from adafruit_servokit import ServoKit
import time

class ServoWriter(object):
    def __init__(self):
        self.servokit_0 = ServoKit(channels=16, address=0x40, frequency=50)
        self.servokit_1 = ServoKit(channels=16, address=0x41, frequency=50)

        self.default_angle = 90
        self.min_angle = 40
        self.max_angle = 140

    def servo_write_0(self, id, angle, angle_error, sleep_time = 0.0001):
        angle_0 = self.default_angle + angle + angle_error
        if angle_0 < self.min_angle + angle_error:
            angle_0 = self.min_angle + angle_error
        elif angle_0 > self.max_angle + angle_error:
            angle_0 = self.max_angle + angle_error

        # debag
        #print("servo angle raw: {}".format(angle_0))
        self.servokit_0.servo[id].angle = angle_0
        #time.sleep( sleep_time )

    def servo_write_1(self, id, angle, angle_error, sleep_time = 0.0001):
        angle_1 = self.default_angle + angle + angle_error
        if angle_1 < self.min_angle + angle_error:
            angle_1 = self.min_angle + angle_error
        elif angle_1 > self.max_angle + angle_error:
            angle_1 = self.max_angle + angle_error

        # debag
        #print("servo angle raw: {}".format(angle_1))
        self.servokit_1.servo[id].angle = angle_1
        #time.sleep( sleep_time )