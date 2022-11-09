# https://automaticaddison.com/how-to-write-a-ros2-publisher-and-subscriber-python-foxy/

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

from .control_modules.tripod_driver import *
from .control_modules.servo_globals import *
import time

import numpy as np
from scipy.interpolate import make_interp_spline


class WalkingMotionParam():
    def __init__(self):
        self.wm = Polynominal()
        self.l_r1, self.r_r1,\
        self.ltop_p, self.lmid_p, self.lbtm_p,\
        self.rtop_p, self.rmid_p, self.rbtm_p,\
        = self.wm.create_angle(
            3.0, 30.0, 10.0, # period, Lr1_init, Lr1_mid
            0.0, -40.0, -5.0, 5.0, 40.0, 0.0, # L pitch top, mid, btm
            0.0, 40.0, 5.0, -5.0, -40.0, 0.0, # R pitch top, mid, btm
        )

        _, _, \
        self.ltop_p_rear, self.lmid_p_rear, self.lbtm_p_rear,\
        self.rtop_p_rear, self.rmid_p_rear, self.rbtm_p_rear,\
        = self.wm.create_angle(
            3.0, 30.0, 10.0, # period, Lr1_init, Lr1_mid
            -45.0, -15.0, 5.0, -5.0, 0.0, 40.0, # Lpitch Init,Mid - top, mid, btm
            45.0, 15.0, -5.0, 5.0, 0.0, -40.0, # Rpitch Init,Mid - top, mid, btm
        )

        """
            3.0, 30.0, 10.0, # period, Lr1_init, Lr1_mid
            -40.0, 0.0, 5.0, -5.0, 0.0, 40.0, # Lpitch Init,Mid - top, mid, btm
            40.0, 0.0, -5.0, 5.0, 0.0, -40.0, # Rpitch Init,Mid - top, mid, btm
        """

        self.r2 = 45

        # Servo Angle Power
        self.power_th1 = 1.2
        self.power_phi = 1.0
        self.power_th2 = 1.0

        # Rviz Duration Time
        self.pos_time = 0.03
        self.proportion_pos_time = 1.5

        # motion flag
        self.tripod_init_flag = True
        self.quadruped_init_flag = True


        # Spline motion
        spline_period = 3
        data_len = 30
        x_time = np.array([0.00, 0.10, 1.00, 2.50, spline_period])
        y_data = np.array([30.0, 21.0, 3.00, 20.0, 30.0])

        spline_model=make_interp_spline(x_time, y_data)

        xs_time=np.linspace(0,spline_period,data_len)
        ys_data=spline_model(xs_time)

        r1_data_v2 = []
        for i in range(0, len(ys_data)):
            r1_data_v2.append(round(ys_data[i], 2))

        r1_init_v2 = [30.0]*31
        self.l_r1_v2 = r1_data_v2 + r1_init_v2
        self.r_r1_v2 = [n*-1 for n in r1_init_v2 + r1_data_v2]
        


        for i in range(0, len(self.l_r1)):
            self.l_r1[i] *= self.power_th1
            self.r_r1[i] *= self.power_th1

            self.l_r1_v2[i] *= self.power_th1
            self.r_r1_v2[i] *= self.power_th1

            # tripod forward
            self.ltop_p[i] *= self.power_phi
            self.lmid_p[i] *= self.power_phi
            self.lbtm_p[i] *= self.power_phi
            self.rtop_p[i] *= self.power_phi
            self.rmid_p[i] *= self.power_phi
            self.rbtm_p[i] *= self.power_phi

            # tripod backward
            self.ltop_p_rear[i] *= self.power_phi
            self.lmid_p_rear[i] *= self.power_phi
            self.lbtm_p_rear[i] *= self.power_phi
            self.rtop_p_rear[i] *= self.power_phi
            self.rmid_p_rear[i] *= self.power_phi
            self.rbtm_p_rear[i] *= self.power_phi


class JointPubSub(Node):

    def __init__(self):
        self.motion = WalkingMotionParam()

        # Initiate the Node class's constructor and give it a name
        super().__init__('walking_motion_node')

        # Create subscriber(s)    
        self.sub_joy = self.create_subscription(
            Joy,
            'joy',
            self.pub_sub_callback,
            10)
        self.sub_joy  # prevent unused variable warning

        # Create publisher(s)  
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 30)
        
        self.joint_state_msg = JointState()
        self.joint_state_msg.header.frame_id = 'joint_tripod_frame'
        self.joint_state_msg.name = gv.jname
        self.joint_state_msg.position = [0.0]*19   

        # motion flag
        self.wingsup_flag = True
        self.wingsdown_flag = True
        self.aileron_L_flag = True
        self.aileron_R_flag = True

        # rviz param
        self.rviz_reversal = -1

    def pub_sub_callback(self, joy_msg):
            
        # self.get_logger().info(' joystick axes: "%s"\n' % joy_msg.axes)
        # self.get_logger().info(' joystick buttons: "%s"\n' % joy_msg.buttons)
        self.elevator = round(joy_msg.axes[3],2)
        self.aileron = round(joy_msg.axes[2],2)
        self.throttle = round(joy_msg.axes[1],2)
        self.rudder = round(joy_msg.axes[0],2)
        self.d_pad_x = joy_msg.axes[5]
        self.d_pad_y = joy_msg.axes[4]
        self.LB_button = joy_msg.buttons[4]
        self.RB_button = joy_msg.buttons[5]
        self.LT_button = joy_msg.buttons[6]
        self.RT_button = joy_msg.buttons[7]
        self.BACK_button = joy_msg.buttons[8]
        self.START_button = joy_msg.buttons[9]

        """
        self.get_logger().info('throttle: {}, rudder: {}'.format(
            self.throttle, self.rudder))
        self.get_logger().info('dpad_x: {}, dpad_y: {}'.format(self.d_pad_x, self.d_pad_y))
        self.get_logger().info('LB: {}, RB: {}, LT: {}, RT: {}'.format(
            self.LB_button, self.RB_button, self.LT_button, self.RT_button
        ))
        self.get_logger().info('BACK: {}, START: {}'.format(
            self.BACK_button, self.START_button)+"\n")
        """

        # TODO Tripod Motion
        # Quadruped torotto control           
        if (abs(self.d_pad_x) > 0.5) and (self.LT_button == 0) and (self.LB_button == 0):
            reverse_value = 1
            if self.motion.quadruped_init_flag:
                self.legged_quadruped_init(self.throttle, reverse_value)
                self.motion.quadruped_init_flag = False

            self.get_logger().info('quadruped torotto')
            self.legged_quadruped_torotto(self.throttle, self.d_pad_x, self.BACK_button)

        elif (abs(self.d_pad_y) > 0.5) and (self.LT_button == 0) and (self.LB_button == 0):
            reverse_value = 1
            if self.motion.quadruped_init_flag:
                self.legged_quadruped_init(self.throttle, reverse_value)
                self.motion.quadruped_init_flag = False

            self.get_logger().info('quadruped rotate torotto')
            self.legged_quadruped_torotto_rotate(self.throttle, self.d_pad_y, self.BACK_button)   

        # Quadruped Init Pos
        elif (abs(self.d_pad_x) < 0.5) and (self.LT_button == 0) and (self.LB_button == 0) and (self.BACK_button == 1):
            reverse_value = 1
            if self.motion.quadruped_init_flag:
                self.legged_quadruped_init(self.throttle, reverse_value)
                self.motion.quadruped_init_flag = False
            
            self.get_logger().info('bending stretching motion')
            self.quadruped_bending_stretching(self.elevator, self.aileron, self.throttle)

        # Quadruped v2 Control
        elif (abs(self.d_pad_x) > 0.5) and (self.LT_button == 0) and (self.LB_button == 1):
            reverse_value = 1
            if self.motion.quadruped_init_flag:
                self.legged_quadruped_init(self.throttle, reverse_value)
                self.motion.quadruped_init_flag = False

            self.get_logger().info('quadruped torotto v2')
            self.quadruped_torotto_v2(self.throttle, self.d_pad_x, self.BACK_button)

        elif (abs(self.d_pad_y) > 0.5) and (self.LT_button == 0) and (self.LB_button == 1):
            reverse_value = 1
            if self.motion.quadruped_init_flag:
                self.legged_quadruped_init(self.throttle, reverse_value)
                self.motion.quadruped_init_flag = False

            self.get_logger().info('quadruped torotto rotate v2')
            self.quadruped_torotto_rotate_v2(self.throttle, self.d_pad_y, self.BACK_button)  


        # Tripod Control
        elif (abs(self.d_pad_x) > 0.5) and (self.LT_button == 1) and (self.LB_button == 0):
            reverse_value = 1
            if self.motion.tripod_init_flag:
                self.legged_init(self.throttle, reverse_value)
                self.motion.tripod_init_flag = False

            self.get_logger().info('tripod walking')
            self.legged_walking(self.throttle, self.d_pad_x, self.BACK_button)

        # Tripod Init Pos
        elif (abs(self.d_pad_x) < 0.5) and (self.LT_button == 1) and (self.LB_button == 0) and (self.BACK_button == 1):
            reverse_value = 1
            if self.motion.tripod_init_flag:
                self.legged_init(self.throttle, reverse_value)
                self.motion.tripod_init_flag = False
        else:
            if self.motion.tripod_init_flag == False:
                reverse_value = -1
                self.legged_init(self.throttle, reverse_value)
                self.motion.tripod_init_flag = True

            elif self.motion.quadruped_init_flag == False:
                reverse_value = -1
                self.legged_quadruped_init(self.throttle, reverse_value)
                self.motion.quadruped_init_flag = True


    def legged_init(self, throttle, reverse_value):
        self.legged_init_angle = 30
        self.proportion_r1 = 1.2
        self.proportion_p = 1.3
        self.proportion_r2 = 1.4
    
            # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        for i in range(0, self.legged_init_angle)[::reverse_value]:
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = i*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = i*self.proportion_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = i*self.proportion_r2
            self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = i*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = 0
            self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = i*self.proportion_r2
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = i*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = -i*self.proportion_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = i*self.proportion_r2

            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -i*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -i*self.proportion_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -i*self.proportion_r2
            self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -i*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = 0
            self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -i*self.proportion_r2
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -i*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = i*self.proportion_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -i*self.proportion_r2
            time.sleep(self.motion.pos_time)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)

    def legged_walking(self, throttle, d_pad_x, back_button):
        loop_count = len(self.motion.l_r1)

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        for i in range(0, loop_count):
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal
            if d_pad_x > 0.5:
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.l_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = -self.motion.ltop_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = -self.motion.r_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.lmid_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.motion.l_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = -self.motion.lbtm_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.r2

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = self.motion.r_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.rtop_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.l_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.rmid_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = self.motion.r_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = -self.motion.rbtm_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.r2
            else:
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.l_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = -self.motion.ltop_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = -self.motion.r_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = -self.motion.lmid_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.motion.l_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = -self.motion.lbtm_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.r2

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = self.motion.r_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.rtop_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.l_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.rmid_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = self.motion.r_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = -self.motion.rbtm_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.r2
            # Motion Period Parameter
            if back_button > 0.5:
                time.sleep(self.motion.pos_time*self.motion.proportion_pos_time)
            else:
                time.sleep(self.motion.pos_time)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)    

    def legged_quadruped_init(self, throttle, reverse_value):
        self.legged_init_angle = 30
        self.proportion_r1 = 1.2
        self.proportion_p = 1.3
        self.proportion_r2 = 1.4

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        for i in range(0, self.legged_init_angle)[::reverse_value]:
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = i*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = i*self.proportion_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = i*self.proportion_r2
            for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lmid_roll2")):
                self.joint_state_msg.position[j] = 0
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = i*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = -i*self.proportion_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = i*self.proportion_r2

            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -i*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -i*self.proportion_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -i*self.proportion_r2
            for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rmid_roll2")):
                self.joint_state_msg.position[j] = 0
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -i*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = i*self.proportion_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -i*self.proportion_r2
            time.sleep(self.motion.pos_time)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)

    def legged_quadruped_torotto(self, throttle, d_pad_x, back_button):
        loop_count = len(self.motion.l_r1)

        ajust_r1 = 9.0

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        for i in range(0, loop_count):
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal
            
            for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lmid_roll2")):
                self.joint_state_msg.position[j] = 0
            for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rmid_roll2")):
                self.joint_state_msg.position[j] = 0

            if d_pad_x > 0.5:
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.l_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = -self.motion.ltop_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = -self.motion.r_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.rbtm_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.r2

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = self.motion.r_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.rtop_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.l_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = self.motion.lbtm_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.r2
            else:
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = (self.motion.l_r1[i]+ajust_r1)*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = -self.motion.ltop_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = -self.motion.r_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.rbtm_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.r2

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = (self.motion.r_r1[i]-ajust_r1)*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.rtop_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.l_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = self.motion.lbtm_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.r2
            # Motion Period Parameter
            if back_button > 0.5:
                time.sleep(self.motion.pos_time*self.motion.proportion_pos_time)
            else:
                time.sleep(self.motion.pos_time)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)

    def legged_quadruped_torotto_rotate(self, throttle, d_pad_y, back_button):
        loop_count = len(self.motion.l_r1)

        ajust_r1 = 9.0

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        for i in range(0, loop_count):
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal
            
            for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lmid_roll2")):
                self.joint_state_msg.position[j] = 0
            for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rmid_roll2")):
                self.joint_state_msg.position[j] = 0
            
            if d_pad_y > 0.5:
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = (self.motion.l_r1[i]+ajust_r1)*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = -self.motion.ltop_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = -self.motion.r_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.rbtm_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.r2

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = (self.motion.r_r1[i]-ajust_r1)*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.rtop_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.l_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = self.motion.lbtm_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.r2
            else:
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = (self.motion.l_r1[i]+ajust_r1)*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = -self.motion.ltop_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = -self.motion.r_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.rbtm_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.r2

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = (self.motion.r_r1[i]-ajust_r1)*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.rtop_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.l_r1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = self.motion.lbtm_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.r2
            # Motion Period Parameter
            if back_button > 0.5:
                time.sleep(self.motion.pos_time*self.motion.proportion_pos_time)
            else:
                time.sleep(self.motion.pos_time)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)

    def quadruped_torotto_v2(self, throttle, d_pad_x, back_button):
        loop_count = len(self.motion.l_r1_v2)

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        for i in range(0, loop_count):
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal
            
            for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lmid_roll2")):
                self.joint_state_msg.position[j] = 0
            for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rmid_roll2")):
                self.joint_state_msg.position[j] = 0

            if d_pad_x > 0.5:
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.l_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = -self.motion.ltop_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = -self.motion.r_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.rbtm_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.r2

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = self.motion.r_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.rtop_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.l_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = self.motion.lbtm_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.r2
            else:
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.l_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = -self.motion.ltop_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = -self.motion.r_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.rbtm_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.r2

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = self.motion.r_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.rtop_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.l_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = self.motion.lbtm_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.r2
            # Motion Period Parameter
            if back_button > 0.5:
                time.sleep(self.motion.pos_time*self.motion.proportion_pos_time)
            else:
                time.sleep(self.motion.pos_time)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)

    def quadruped_torotto_rotate_v2(self, throttle, d_pad_y, back_button):
        loop_count = len(self.motion.l_r1_v2)

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        for i in range(0, loop_count):
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal
            
            for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lmid_roll2")):
                self.joint_state_msg.position[j] = 0
            for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rmid_roll2")):
                self.joint_state_msg.position[j] = 0
            
            if d_pad_y > 0.5:
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.l_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = -self.motion.ltop_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = 40*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = -20
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = 40
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = self.motion.r_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.rtop_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.l_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = self.motion.lbtm_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.r2
            else:
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.l_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = -self.motion.ltop_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = -self.motion.r_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.rbtm_p[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.r2

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = self.motion.r_r1_v2[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.rtop_p_rear[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.r2
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -40*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = 20
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -40

            # Motion Period Parameter
            if back_button > 0.5:
                time.sleep(self.motion.pos_time*self.motion.proportion_pos_time)
            else:
                time.sleep(self.motion.pos_time)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)

    def quadruped_bending_stretching(self, elevator, aileron, throttle):
        self.legged_init_angle = 30
        self.r1_stretching = 1.2
        self.r1_bending = 0.7

        self.proportion_r1 = 1.0
        self.proportion_p = 1.3
        self.proportion_r2 = 1.4

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        if elevator > 0.5:
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.legged_init_angle*self.r1_stretching*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.legged_init_angle*self.r1_bending*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.legged_init_angle*self.r1_stretching*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.legged_init_angle*self.r1_bending*self.rviz_reversal
        elif elevator < -0.5:
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.legged_init_angle*self.r1_bending*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.legged_init_angle*self.r1_stretching*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.legged_init_angle*self.r1_bending*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.legged_init_angle*self.r1_stretching*self.rviz_reversal

        elif aileron > 0.5:
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.legged_init_angle*self.r1_bending*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.legged_init_angle*self.r1_bending*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.legged_init_angle*self.r1_stretching*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.legged_init_angle*self.r1_stretching*self.rviz_reversal            

        elif aileron < -0.5:
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.legged_init_angle*self.r1_stretching*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.legged_init_angle*self.r1_stretching*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.legged_init_angle*self.r1_bending*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.legged_init_angle*self.r1_bending*self.rviz_reversal               

        else:
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.legged_init_angle*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.legged_init_angle*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.legged_init_angle*self.proportion_r1*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.legged_init_angle*self.proportion_r1*self.rviz_reversal
        self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.legged_init_angle*self.proportion_p
        
        for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lmid_roll2")):
            self.joint_state_msg.position[j] = 0
        self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal
        self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = -self.legged_init_angle*self.proportion_p
        self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.legged_init_angle*self.proportion_r2
        self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.legged_init_angle*self.proportion_p
        self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.legged_init_angle*self.proportion_r2
        for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rmid_roll2")):
            self.joint_state_msg.position[j] = 0
        self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = self.legged_init_angle*self.proportion_p
        self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.legged_init_angle*self.proportion_r2
        self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.legged_init_angle*self.proportion_p
        self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.legged_init_angle*self.proportion_r2
        # convert RViz parameter
        self.convert_rviz_param(self.joint_state_msg)
        self.pub_joint_state.publish(self.joint_state_msg)

    def convert_rviz_param(self, joint_msg):
            for i in range(0, len(joint_msg.name)):
                if joint_msg.position[i] == gv.jname.index("rev_link_head"):
                    rviz_param = 7/400
                    joint_msg.position[i] *= rviz_param
                elif joint_msg.position[i] == range(1, 18, 3):
                    rviz_param = 1/80
                    joint_msg.position[i] *= rviz_param
                else:
                    rviz_param = 1/60
                    joint_msg.position[i] *= rviz_param


def main(args=None):
 
  # Initialize the rclpy library
  rclpy.init(args=args)
  # Create the node
  publishing_subscriber = JointPubSub()
 
  rclpy.spin(publishing_subscriber)
 
  publishing_subscriber.destroy_node()
 
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()