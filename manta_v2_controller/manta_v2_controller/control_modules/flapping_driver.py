import math


class Flapping(object):
    def __init__(
        self, period=None, down_freq_e_fish=None, \
    ):
        self.period = period
        self.down_freq_e_fish = down_freq_e_fish

        self.freq = 1/self.period
        self.period_down = self.down_freq_e_fish*self.period
        self.period_up = self.period - self.period_down
        self.freq_down = 1/(2*self.period_down)
        self.freq_up = 1/(2*self.period_up)

    def create_flapping_angle(self, y_up_max, y_down_max, phase_diff):
        t = 0
        angle = []
        while t < self.period:
            if t < self.period_down:
                yt = round(y_up_max * math.sin(2*math.pi*self.freq_down*t + phase_diff), 2)
                if yt > 0:
                    angle.append(yt)
                else:
                    yt = round(y_down_max * math.sin(2*math.pi*self.freq_down*t + phase_diff), 2)
                    angle.append(yt)
            elif self.period_down <= t and t <= self.period:
                yt = round(-y_down_max * math.sin(2*math.pi*self.freq_up*(t-self.period_down) + phase_diff), 2)
                if yt < 0:
                    angle.append(yt)
                else:
                    yt = round(-y_up_max * math.sin(2*math.pi*self.freq_up*(t-self.period_down) + phase_diff), 2)
                    angle.append(yt)

            t += self.period/41
        # print(angle)
        # print("len:{}".format(len(angle)))

        return angle


    def average_generator(self, pre_array):
        for _ in range(0, 1):
            ave = []
            for i in range(0, len(pre_array)-1):
                ave.append((pre_array[i]+pre_array[i+1])/2)
            for j in range(0, len(pre_array)-1):
                pre_array.insert(2*j+1, round(ave[j],2))
        return pre_array


# if __name__ == '__main__':
#   flapping = Flapping(period=2.5, down_freq_e_fish=0.43)
#   y3, y5, chord1 = [], [], []
#   y3 = flapping.create_flapping_angle(
#       y_up_max=255, y_down_max=115, phase_diff=3*math.pi/4)
#   y5 = flapping.create_flapping_angle(
#       y_up_max=650, y_down_max=400, phase_diff=math.pi/2)

#   flapping.average_generator(y5)
#   print(len(y5))

#   chord1 = flapping.create_flapping_angle(
#       y_up_max=27, y_down_max=13, phase_diff=math.pi/4)