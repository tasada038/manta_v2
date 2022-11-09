import sympy
import math


class Polynominal(object):
    def __init__(
        self, period=None, half_period=None, \
        thi_j_s=None, thi_j_m=None, thi_j_g=None, \
        w_j_s=None, w_j_m=None, w_j_g=None,\
    ):
        self.period = period
        self.half_period = half_period

        self.thi_j_start_lr1 = thi_j_s
        self.thi_j_mid_lr1 = thi_j_m
        self.thi_j_goal_lr1 = thi_j_g
        self.w_j_start_lr1 = w_j_s
        self.w_j_mid_lr1 = w_j_m
        self.w_j_goal_lr1 = w_j_g

    def calculate_polynominal(self, name):
        a0 = self.thi_j_start_lr1
        a1 = self.w_j_start_lr1
        a2 = sympy.Symbol('a2')
        a3 = sympy.Symbol('a3')
        a4 = sympy.Symbol('a4')
        a5 = sympy.Symbol('a5')

        thi_j_mid = a0 + a1*(self.half_period) + a2*(self.half_period)**2 + a3*(self.half_period)**3 + a4*(self.half_period)**4 + a5*(self.half_period)**5 - (self.thi_j_mid_lr1)
        thi_j_goal = a0 + a1*(self.period) + a2*(self.period)**2 + a3*(self.period)**3 + a4*(self.period)**4 + a5*(self.period)**5 - (self.thi_j_goal_lr1)
        w_j_mid = a1 + 2*a2*(self.half_period) + 3*a3*(self.half_period)**2 + 4*a4*(self.half_period)**3 + 5*a5*(self.half_period)*4 - (self.w_j_mid_lr1)
        w_j_goal = a1 + 2*a2*(self.period) + 3*a3*(self.period)**2 + 4*a4*(self.period)**3 + 5*a5*(self.period)*4 - (self.w_j_goal_lr1)

        solution = sympy.solve([thi_j_mid, thi_j_goal, w_j_mid, w_j_goal])
        #print(solution)


        a2 = float(format(solution[a2],'.2f'))
        a3 = float(format(solution[a3],'.2f'))
        a4 = float(format(solution[a4],'.2f'))
        a5 = float(format(solution[a5],'.2f'))
        print("{} - a2:{}, a3:{}, a4:{}, a5:{}".format(name, a2, a3, a4, a5))

        angle_t = self.angle_list(self.period)
        angle_value = []

        for i in range(0, len(angle_t)):
            # Left r1 / use Right r1, thi * -1
            thi = a0 + a1*(angle_t[i]) + a2*(angle_t[i])**2 + a3*(angle_t[i])**3 + a4*(angle_t[i])**4 + a5*(angle_t[i])**5
            angle_value.append(round(thi,2))
            # print("{}: {}".format(angle_t[i], thi))

        #print("angle list: {}".format(angle_value))

        return angle_value

    def angle_list(self, period):
        angle_t = []
        for i in range(int(period*10) + 1):
            angle_t.append(round(i*0.1,1))
        return angle_t

    def add_init_angle(self, len, list, angle):
        for _ in range(0, len-1):
            list.append(angle)  

    def create_angle(self,\
        period, Lr1_init, Lr1_mid,\
        Ltop_init, Ltop_mid, Lmid_init, Lmid_mid, Lbtm_init, Lbtm_mid,\
        Rtop_init, Rtop_mid, Rmid_init, Rmid_mid, Rbtm_init, Rbtm_mid,\
    ):
        # angular velocity wj_middle
        w_j_m = 0.0

        # TODO r1 Init
        angle_period = period
        angle_Lr1_list, angle_Rr1_list = [], []
        Lr1_init_angle = Lr1_init
        Lr1_mid_angle = Lr1_mid
        wj_s = Lr1_mid_angle-Lr1_init_angle
        wj_g = Lr1_init_angle-Lr1_mid_angle

        angle_r1 = Polynominal(
            angle_period, angle_period/2,\
            Lr1_init_angle, Lr1_mid_angle, Lr1_init_angle,\
            wj_s,  w_j_m, wj_g,\
        )
        non_list = angle_r1.angle_list(angle_period)

        # Lr1
        angle_Lr1_list = angle_r1.calculate_polynominal("Lr1")
        angle_r1.add_init_angle(len(non_list), angle_Lr1_list, Lr1_init_angle)

        """
        # print("Left Roll1:{} \n".format(angle_Lr1_list))
        print("Left Roll1:")
        Lr1_n = 10
        for i in range(0, len(angle_Lr1_list), Lr1_n):
            print(angle_Lr1_list[i: i+Lr1_n])
        print("\n")
        """

        # Rr1
        angle_r1.add_init_angle(len(non_list), angle_Rr1_list, -1*Lr1_init_angle)
        Rr1_data = angle_r1.calculate_polynominal("Rr1")
        for i in range(0, len(Rr1_data)):
            angle_Rr1_list.append(-1*(Rr1_data[i]))

        """
        # print("Right Roll1:{} \n".format(angle_Rr1_list))
        print("Right Roll1:")
        Rr1_n = 10
        for i in range(0, len(angle_Rr1_list), Rr1_n):
            print(angle_Rr1_list[i: i+Rr1_n])
        print("\n")
        """

        ### L pitch
        # TODO Ltop_pitch Init
        angle_period_p = 4/3*angle_period
        angle_Ltop_p_list, angle_Lmid_p_list, angle_Lbtm_p_list = [], [], []
        Ltop_init_angle = Ltop_init
        Ltop_mid_angle = Ltop_mid
        wj_s = Ltop_mid_angle-Ltop_init_angle
        wj_g = Ltop_init_angle-Ltop_mid_angle

        # Calculate Ltop_pitch
        angle_Ltop_p = Polynominal(
            angle_period_p, angle_period_p/2, \
            Ltop_init_angle, Ltop_mid_angle, Ltop_init_angle, \
            wj_s,  w_j_m, wj_g,\
        )
        data_Ltop_p = angle_Ltop_p.calculate_polynominal("Ltop_p")
        non_pitch_list = angle_Ltop_p.angle_list(angle_period - angle_period_p/2)

        angle_Ltop_p.add_init_angle(len(non_pitch_list), angle_Ltop_p_list, Ltop_init_angle)
        for i in range(0, len(data_Ltop_p)):
            angle_Ltop_p_list.append(data_Ltop_p[i])
        angle_Ltop_p.add_init_angle(len(non_pitch_list), angle_Ltop_p_list, Ltop_init_angle)

        """
        # print("Left top Pitch:{} \n".format(angle_Ltop_p_list))
        print("Left top Pitch:")
        Ltop_p_n = 10
        for i in range(0, len(angle_Ltop_p_list), Ltop_p_n):
            print(angle_Ltop_p_list[i: i+Ltop_p_n])
        print("\n")
        """

        # TODO Lmid_pitch_init
        Lmid_init_angle = Lmid_init
        Lmid_mid_angle = Lmid_mid
        wj_s = Lmid_mid_angle - Lmid_init_angle
        wj_g = Lmid_init_angle - Lmid_mid_angle

        # Calculate Lmid_pitch
        angle_Lmid_p = Polynominal(
            angle_period_p, angle_period_p/2, \
            Lmid_init_angle, Lmid_mid_angle, Lmid_init_angle, \
            wj_s,  w_j_m, wj_g,\
        )
        data_Lmid_p = angle_Lmid_p.calculate_polynominal("Lmid_p")

        angle_Lmid_p.add_init_angle(len(non_pitch_list), angle_Lmid_p_list, Lmid_init_angle)
        for i in range(0, len(data_Lmid_p)):
            angle_Lmid_p_list.append(data_Lmid_p[i])
        angle_Lmid_p.add_init_angle(len(non_pitch_list), angle_Lmid_p_list, Lmid_init_angle)

        """
        # print("Left mid Pitch:{} \n".format(angle_Lmid_p_list))
        print("Left mid Pitch:")
        Lmid_p_n = 10
        for i in range(0, len(angle_Lmid_p_list), Lmid_p_n):
            print(angle_Lmid_p_list[i: i+Lmid_p_n])
        print("\n")
        """

        # TODO Lbtm_pitch_init
        Lbtm_init_angle = Lbtm_init
        Lbtm_mid_angle = Lbtm_mid
        wj_s = Lbtm_mid_angle - Lbtm_init_angle
        wj_g = Lbtm_init_angle - Lbtm_mid_angle

        # Calculate Lbtm_pitch
        angle_Lbtm_p = Polynominal(
            angle_period_p, angle_period_p/2, \
            Lbtm_init_angle, Lbtm_mid_angle, Lbtm_init_angle, \
            wj_s,  w_j_m, wj_g,\
        )
        data_Lbtm_p = angle_Lbtm_p.calculate_polynominal("Lbtm_p")

        angle_Lbtm_p.add_init_angle(len(non_pitch_list), angle_Lbtm_p_list, Lbtm_init_angle)
        for i in range(0, len(data_Lbtm_p)):
            angle_Lbtm_p_list.append(data_Lbtm_p[i])
        angle_Lbtm_p.add_init_angle(len(non_pitch_list), angle_Lbtm_p_list, Lbtm_init_angle)

        """"
        # print("Left btm Pitch:{} \n".format(angle_Lbtm_p_list))  
        print("Left btm Pitch:")
        Lbtm_p_n = 10
        for i in range(0, len(angle_Lbtm_p_list), Lbtm_p_n):
            print(angle_Lbtm_p_list[i: i+Lbtm_p_n])
        print("\n")
        """

        ### R pitch
        # TODO Rtop_pitch Init
        angle_period_p = 4/3*angle_period
        angle_Rtop_p_list, angle_Rmid_p_list, angle_Rbtm_p_list = [], [], []
        Rtop_init_angle = Rtop_init
        Rtop_mid_angle = Rtop_mid
        wj_s = Rtop_mid_angle - Rtop_init_angle
        wj_g = Rtop_init_angle - Rtop_mid_angle

        # Calculate Rtop_pitch
        angle_Rtop_p = Polynominal(
            angle_period_p, angle_period_p/2, \
            Rtop_init_angle, Rtop_mid_angle, Rtop_init_angle, \
            wj_s,  w_j_m, wj_g,\
        )
        data_Rtop_p = angle_Rtop_p.calculate_polynominal("Rtop_p")
        non_pitch_list = angle_Rtop_p.angle_list(2*angle_period - angle_period_p)
        Rtop_p_len_half = len(data_Rtop_p)/2

        for i in range(math.floor(Rtop_p_len_half), int(2*Rtop_p_len_half)):
            angle_Rtop_p_list.append(data_Rtop_p[i])
        angle_Rtop_p.add_init_angle(len(non_pitch_list), angle_Rtop_p_list, Rtop_init_angle)
        for i in range(0, math.floor(Rtop_p_len_half)):
            angle_Rtop_p_list.append(data_Rtop_p[i])

        """
        # print("Right top Pitch:{} \n".format(angle_Rtop_p_list))
        print("Right top Pitch:")
        Rtop_p_n = 10
        for i in range(0, len(angle_Rtop_p_list), Rtop_p_n):
            print(angle_Rtop_p_list[i: i+Rtop_p_n])
        print("\n")
        """

        # TODO Rmid_pitch Init
        Rmid_init_angle = Rmid_init
        Rmid_mid_angle = Rmid_mid
        wj_s = Rmid_mid_angle - Rmid_init_angle
        wj_g = Rmid_init_angle - Rmid_mid_angle

        # Calculate Rmid_pitch
        angle_Rmid_p = Polynominal(
            angle_period_p, angle_period_p/2, \
            Rmid_init_angle, Rmid_mid_angle, Rmid_init_angle, \
            wj_s,  w_j_m, wj_g,\
        )
        data_Rmid_p = angle_Rmid_p.calculate_polynominal("Rmid_p")
        non_pitch_list = angle_Rmid_p.angle_list(2*angle_period - angle_period_p)
        Rmid_p_len_half = len(data_Rmid_p)/2

        for i in range(math.floor(Rmid_p_len_half), int(2*Rmid_p_len_half)):
            angle_Rmid_p_list.append(data_Rmid_p[i])
        angle_Rmid_p.add_init_angle(len(non_pitch_list), angle_Rmid_p_list, Rmid_init_angle)
        for i in range(0, math.floor(Rmid_p_len_half)):
            angle_Rmid_p_list.append(data_Rmid_p[i])

        """
        # print("Right mid Pitch:{} \n".format(angle_Rmid_p_list))
        print("Right mid Pitch:")
        Rmid_p_n = 10
        for i in range(0, len(angle_Rmid_p_list), Rmid_p_n):
            print(angle_Rmid_p_list[i: i+Rmid_p_n])
        print("\n")
        """

        # TODO Rbtm_pitch Init
        Rbtm_init_angle = Rbtm_init
        Rbtm_mid_angle = Rbtm_mid
        wj_s = Rbtm_mid_angle - Rbtm_init_angle
        wj_g = Rbtm_init_angle - Rbtm_mid_angle

        # Calculate Rbtm_pitch
        angle_Rbtm_p = Polynominal(
            angle_period_p, angle_period_p/2, \
            Rbtm_init_angle, Rbtm_mid_angle, Rbtm_init_angle, \
            wj_s,  w_j_m, wj_g,\
        )
        data_Rbtm_p = angle_Rbtm_p.calculate_polynominal("Rbtm_p")
        non_pitch_list = angle_Rbtm_p.angle_list(2*angle_period - angle_period_p)
        Rbtm_p_len_half = len(data_Rbtm_p)/2

        for i in range(math.floor(Rbtm_p_len_half), int(2*Rbtm_p_len_half)):
            angle_Rbtm_p_list.append(data_Rbtm_p[i])
        angle_Rbtm_p.add_init_angle(len(non_pitch_list), angle_Rbtm_p_list, Rbtm_init_angle)
        for i in range(0, math.floor(Rbtm_p_len_half)):
            angle_Rbtm_p_list.append(data_Rbtm_p[i])

        """
        # print("Right btm Pitch:{} \n".format(angle_Rbtm_p_list))
        print("Right btm Pitch:")
        Rbtm_p_n = 10
        for i in range(0, len(angle_Rbtm_p_list), Rbtm_p_n):
            print(angle_Rbtm_p_list[i: i+Rbtm_p_n])
        print("\n")
        """

        return angle_Lr1_list, angle_Rr1_list, \
               angle_Ltop_p_list, angle_Lmid_p_list, angle_Lbtm_p_list, \
               angle_Rtop_p_list, angle_Rmid_p_list, angle_Rbtm_p_list