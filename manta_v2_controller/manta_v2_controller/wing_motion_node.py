# https://automaticaddison.com/how-to-write-a-ros2-publisher-and-subscriber-python-foxy/

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from .control_modules.flapping_driver import *
from .control_modules.servo_globals import *
import time
import math
# sudo apt install ros-foxy-tf-transformations
# sudo pip3 install transforms3d
import tf_transformations


class WingMotionParam():
    def __init__(self):    

        L13 = 169.4
        L35 = 214.7

        # Create the motion of the flapping wing
        flapping = Flapping(period=2.5, down_freq_e_fish=0.43)
        p3y, p5y, self.mid_phi_chord1, self.mid_phi_chord2 = [], [], [], []
        p3y = flapping.create_flapping_angle(
            y_up_max=255, y_down_max=115, phase_diff=3*math.pi/4)
        p5y = flapping.create_flapping_angle(
            y_up_max=650, y_down_max=400, phase_diff=math.pi/2)
        self.mid_phi_chord1 = flapping.create_flapping_angle(
            y_up_max=27, y_down_max=13, phase_diff=math.pi/4)
        self.mid_phi_chord2 = flapping.create_flapping_angle(
            y_up_max=54.5, y_down_max=41.5, phase_diff=math.pi/4)

        # Calculate th1, th2 angle from arc-sin
        self.mid_th1, self.mid_th2 = [], []
        for i in range(len(p3y)):
            angle_th1 = round( math.degrees( math.asin( (p3y[i]/3) / L13) ) ,2)
            self.mid_th1.append(angle_th1)

            angle_th2 = round( math.degrees( math.asin( ((p5y[i]-p3y[i])/3) / L35) ) ,2)
            self.mid_th2.append(angle_th2)
        # debag
        # print("mid th1 angle : {} \n".format(self.mid_th1))
        # print("mid th2 angle : {} \n".format(self.mid_th2))

        # Servo Angle Setup
        self.mid_th1 = self.average_generator(self.mid_th1)
        self.mid_phi_chord1 = self.average_generator(self.mid_phi_chord1)
        self.mid_phi_chord2 = self.average_generator(self.mid_phi_chord2)
        self.mid_th2 = self.average_generator(self.mid_th2)

        # Servo Angle Offset Power
        self.power_th1 = 1.2
        self.power_phi_chord1 = 1.3
        self.power_phi_chord2 = 0.8
        self.power_th2 = 1.0
        self.shift = 4

        self.wing_period = 0.015

        for i in range(0, len(self.mid_th1)):
            self.mid_th1[i] = self.mid_th1[i] * self.power_th1 * -1
            self.mid_phi_chord1[i] = self.mid_phi_chord1[i] * self.power_phi_chord1 * -1
            self.mid_phi_chord2[i] = self.mid_phi_chord2[i] * self.power_phi_chord2 * -1
            self.mid_th2[i] = self.mid_th2[i] * self.power_th2 * -1

    # arithmetic mean append
    def average_generator(self, pre_array):
        for _ in range(0, 1):
            ave = []
            for i in range(0, len(pre_array)-1):
                ave.append((pre_array[i]+pre_array[i+1])/2)
            for j in range(0, len(pre_array)-1):
                pre_array.insert(2*j+1, round(ave[j],2))
        return pre_array


class PidController(object):
    def __init__(self):

        self.m, self.m1 = 0.0, 0.0
        self.KP = 0.1
        self.KI = 0.1
        self.KD = 0.1
        self.delta_t = 0.01
        self.integral = 0.0
        self.e1 = 0.0
        self.e = 0.0

    def pid_controller(self, sensor_val, target_val):

        self.m1 = self.m
        self.e = self.e1
        self.e1 = sensor_val - target_val
        self.integral += (self.e1+self.e) /2*self.delta_t

        p = self.KP*self.e1
        i = self.KI*self.integral
        d = self.KD*(self.e1-self.e)/self.delta_t

        self.m = self.m1 + p + i +d

        return self.m

    def min_max(l):
        l_min = min(l)
        l_max = max(l)
        return [(i - l_min) / (l_max - l_min) for i in l]

class JointPubSub(Node):

    def __init__(self):
        # WingMotionParam Instance Parameter
        self.motion = WingMotionParam()
        # Pid Controller
        self.pid = PidController()

        # Initiate the Node class's constructor and give it a name
        super().__init__('wing_motion_node')

        # Create subscriber(s)

        self.sub_imu = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.sub_imu_callback,
            10,
        )
        self.sub_imu

        self.sub_depth = self.create_subscription(
            Float32,
            '/bar30/depth',
            self.sub_depth_callback,
            10,
        )

        #  Initialize Euler angle & depth data
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.roll_err, self.pitch_err, self.yaw_err = 0.0, 0.0, 180.0
        self.rpy_err_flag = True
        self.depth_data = 0.0

        self.sub_joy = self.create_subscription(
            Joy,
            'joy',
            self.pub_sub_callback,
            10)
        self.sub_joy  # prevent unused variable warning

        # Create publisher(s)  
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 30)
        
        self.joint_state_msg = JointState()
        self.joint_state_msg.header.frame_id = 'joint_wing_frame'
        self.joint_state_msg.name = gv.jname
        self.joint_state_msg.position = [0.0]*19   

        # Create ROS2 Bag Publisher(s)
        self.pub_yaw_state = self.create_publisher(Float32, 'yaw_actual_data', 10)
        self.yaw_msg = Float32()

        # motion flag
        self.wingsup_flag = True
        self.wingsdown_flag = True
        self.aileron_L_flag = True
        self.aileron_R_flag = True

        # rviz param
        self.rviz_reversal = -1


    # Callback 
    def sub_imu_callback(self, imu_msg):
        quat_msg = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z,  imu_msg.orientation.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quat_msg)
        self.roll = round(roll*180/math.pi - self.roll_err, 2)
        self.pitch = round(pitch*180/math.pi - self.pitch_err, 2)
        self.yaw = round(yaw*180/math.pi - self.yaw_err, 2)

        if self.yaw > 180:
            self.yaw -= 360
        elif self.yaw < -180:
            self.yaw += 360
        

        if self.rpy_err_flag:
            self.roll_err += self.roll
            self.pitch_err += self.pitch
            self.yaw_err += self.yaw
            self.get_logger().info('Error roll: {}, pitch: {}, yaw: {}'.format(
                self.roll_err, self.pitch_err, self.yaw_err
            ))
            self.rpy_err_flag = False

    def sub_depth_callback(self, depth_msg):
        self.depth_data = round(depth_msg.data*1000, 3)


    def pub_sub_callback(self, joy_msg):
            
        # self.get_logger().info(' joystick axes: "%s"\n' % joy_msg.axes)
        # self.get_logger().info(' joystick buttons: "%s"\n' % joy_msg.buttons)
        self.elevator = round(joy_msg.axes[3],2)
        self.aileron = round(joy_msg.axes[2],2)
        self.throttle = round(joy_msg.axes[1],2)
        self.rudder = round(joy_msg.axes[0],2)
        self.d_pad_x = joy_msg.axes[5]
        self.d_pad_y = joy_msg.axes[4]
        self.X_button = joy_msg.buttons[0]
        self.A_button = joy_msg.buttons[1]
        self.B_button = joy_msg.buttons[2]
        self.Y_button = joy_msg.buttons[3]
        self.LB_button = joy_msg.buttons[4]
        self.RB_button = joy_msg.buttons[5]
        self.LT_button = joy_msg.buttons[6]
        self.RT_button = joy_msg.buttons[7]
        self.BACK_button = joy_msg.buttons[8]
        self.START_button = joy_msg.buttons[9]
        self.Ljoy_button = joy_msg.buttons[10]
        self.Rjoy_button = joy_msg.buttons[11]

        # debag
        # self.get_logger().info('elevator: {}, aileron: {}, throttle: {}, rudder: {}'.format(
        #     self.elevator, self.aileron, self.throttle, self.rudder))
        # self.get_logger().info('dpad_x: {}, dpad_y: {}'.format(self.d_pad_x, self.d_pad_y))
        # self.get_logger().info('X: {}, A: {}, B:{}, Y:{}'.format(
        #     self.X_button, self.A_button, self.B_button, self.Y_button))
        # self.get_logger().info('LB: {}, RB: {}, LT: {}, RT: {}'.format(
        #     self.LB_button, self.RB_button, self.LT_button, self.RT_button
        # ))
        # self.get_logger().info('BACK: {}, START: {}, Ljoy: {}, Rjoy: {}'.format(
        #     self.BACK_button, self.START_button, self.Ljoy_button, self.Rjoy_button
        # )+"\n")


        self.get_logger().info('roll: {}, pitch: {}, yaw: {}'.format(self.roll, self.pitch, self.yaw))
        self.get_logger().info('depth: {}'.format(self.depth_data))
        self.yaw_msg.data = self.yaw
        self.pub_yaw_state.publish(self.yaw_msg)


        if self.Rjoy_button == 1:
            self.wing_stopping()

        # TODO Wing Motion Main
        # right stick up(elevator) ↑
        if (self.elevator>0.5) and (abs(self.aileron)<0.5) and (abs(self.rudder)<0.5) and (self.BACK_button==0) and (self.Ljoy_button==1):
            self.motion_init()
            self.get_logger().info('oscillate fin')
            self.oscillate_fin(self.throttle, self.RT_button, self.RB_button)

        elif (self.elevator>0.5) and (abs(self.aileron)<0.5) and (abs(self.rudder)<0.5) and (self.BACK_button==0) and (self.Ljoy_button==0):
            self.motion_init()
            self.get_logger().info('oscillate pid controller')
            self.oscillate_pid(self.throttle, self.RT_button, self.RB_button)        

        elif (self.elevator<-0.5) and (abs(self.aileron)<0.5) and (abs(self.rudder)<0.5) and (self.BACK_button==0):
            self.motion_init()
            self.get_logger().info('oscillate fin')
            self.oscillate_back_fin(self.throttle, self.RT_button, self.RB_button)

        # left stick (rudder) ← →
        elif abs(self.rudder) > 0.5 and (abs(self.elevator) < 0.5) and (self.BACK_button==0):
            self.motion_init()
            self.get_logger().info('lateral move')
            self.lateral_move(self.rudder, self.throttle, self.RT_button, self.RB_button)

        #right stick ← →
        # LB, LT = 0, 0
        elif abs(self.aileron) > 0.5 and (abs(self.elevator) < 0.5) and (self.LB_button < 0.5) and (self.LT_button < 0.5) and (self.BACK_button==0):
            self.motion_init()
            self.get_logger().info('superstrength turn')
            self.superstrength_turn(self.aileron, self.throttle, self.RT_button, self.RB_button)
        # LB = 1, LT = 0
        elif abs(self.aileron) > 0.5 and (abs(self.elevator) < 0.5) and (self.LB_button > 0.5) and (self.LT_button < 0.5) and (self.BACK_button==0):
            self.motion_init()
            self.get_logger().info('spin turn')
            self.spin_turn(self.aileron, self.throttle, self.RT_button, self.RB_button)
        # LB = 0, LT = 1
        elif abs(self.aileron) > 0.5 and (abs(self.elevator) < 0.5) and (self.LB_button < 0.5) and (self.LT_button > 0.5) and (self.BACK_button==0):
            self.motion_down_init(self.aileron)
            self.get_logger().info('move Left and right')
            self.original_turn(self.aileron, self.throttle, self.RT_button, self.RB_button)

        else:
            if self.wingsup_flag == False:
                reverse_value = -1
                self.wingsup_init(reverse_value)
                self.wingsup_flag = True

            elif self.wingsdown_flag == False:
                reverse_value = -1
                if self.aileron_L_flag:
                    self.wingsdown_init(reverse_value, 1)
                    self.aileron_L_flag = False
                elif self.aileron_R_flag:
                    self.wingsdown_init(reverse_value, -1)
                    self.aileron_R_flag = False                   

                self.wingsdown_flag = True

    def motion_init(self):
        reverse_value = 1
        if self.wingsup_flag:
            self.wingsup_init(reverse_value)
            self.wingsup_flag = False

    def motion_down_init(self, aileron):
        reverse_value = 1
        if self.wingsdown_flag:
            self.wingsdown_init(reverse_value, aileron)
            self.wingsdown_flag = False

    def wingsup_init(self, reverse_value):
        self.wingsup_r = 30
        self.wingsup_p = 0
        
        for i in range(0, self.wingsup_r)[::reverse_value]:

            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = 0

            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = -i*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.wingsup_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = -i
            self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = -i*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.wingsup_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = -i
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = -i*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.wingsup_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = -i

            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = i*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = self.wingsup_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = i
            self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = i*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = self.wingsup_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = i
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = i*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = self.wingsup_p
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = i
            time.sleep(self.motion.wing_period*1.5)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)

    def wingsdown_init(self, reverse_value, aileron):
        self.wingsup_r = 15
        self.wingsup_p = 0
        
        for i in range(0, self.wingsup_r)[::reverse_value]:

            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = 0

            if aileron > 0.5:
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = -self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = i
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = -self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = i
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = -self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = i

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = i
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = i
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = i

                self.aileron_L_flag = True

            else:
                for j in range(gv.jname.index("rev_link_Ltop_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                    self.joint_state_msg.position[j] = 0

                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = -i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = -i
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = -i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = -i
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = -i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = -i

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -i
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -i
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -i*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = -self.wingsup_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -i

                self.aileron_R_flag = True

            time.sleep(self.motion.wing_period)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)

    def oscillate_fin(self, throttle, RT_button, RB_button):
        loop_count = len(self.motion.mid_th1)

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        for i in range(0, loop_count):
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            if RT_button == 1 and RB_button == 0: # top fin only
                # L middle & bottom fin stopping
                for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                    self.joint_state_msg.position[j] = 0
                # R middle & bottom fin stopping
                for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                    self.joint_state_msg.position[j] = 0

            elif RT_button == 0 and RB_button == 1: # top and middle fin
                # middle fin oscillate
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]
                # bottom fin stopping
                for j in range(gv.jname.index("rev_link_Lbtm_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                    self.joint_state_msg.position[j] = 0                
                for j in range(gv.jname.index("rev_link_Rbtm_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                    self.joint_state_msg.position[j] = 0  

            elif RT_button == 0 and RB_button == 0: # all fin oscillate
                # middle fin oscillate
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]
                # bottom fin oscillate                
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift*2]
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.mid_th2[i-self.motion.shift*2]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift*2]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.mid_th2[i-self.motion.shift*2]

            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.mid_th1[i]*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.motion.mid_phi_chord1[i]
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.mid_th2[i]
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.motion.mid_th1[i]*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.mid_phi_chord1[i]
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.mid_th2[i]

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

            time.sleep(self.motion.wing_period)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)

    def oscillate_pid(self, throttle, RT_button, RB_button):
        loop_count = len(self.motion.mid_th1)

        manipulated_value = self.pid.pid_controller(self.yaw, 0.0)/450
        # Limit manipulated_value
        if abs(self.yaw) <= 5:
            manipulated_value = 0
        elif abs(self.yaw) > 45:
            manipulated_value = 1
        elif abs(self.yaw) < -45:
            manipulated_value = -1
        self.get_logger().info("manipulated value:{}".format(manipulated_value))

        # PID control by manipulated variable
        power_left, power_right = 1.0, 1.0
        if manipulated_value > 0.0:
            power_right = 1 - manipulated_value
        elif manipulated_value < 0.0:
            power_left = 1 + manipulated_value

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        for i in range(0, loop_count):
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            if RT_button == 1 and RB_button == 0: # top fin only
                # L middle & bottom fin stopping
                for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                    self.joint_state_msg.position[j] = 0
                # R middle & bottom fin stopping
                for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                    self.joint_state_msg.position[j] = 0

            elif RT_button == 0 and RB_button == 1: # top and middle fin
                # middle fin oscillate
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal*power_left
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]*power_left
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]*power_left
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal*power_right
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]*power_right
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]*power_right
                # bottom fin stopping
                for j in range(gv.jname.index("rev_link_Lbtm_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                    self.joint_state_msg.position[j] = 0                
                for j in range(gv.jname.index("rev_link_Rbtm_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                    self.joint_state_msg.position[j] = 0  

            elif RT_button == 0 and RB_button == 0: # all fin oscillate
                # middle fin oscillate
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal*power_left
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]*power_left
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]*power_left
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal*power_right
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]*power_right
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]*power_right
                # bottom fin oscillate                
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal*power_left
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift*2]*power_left
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.mid_th2[i-self.motion.shift*2]*power_left
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal*power_right
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift*2]*power_right
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.mid_th2[i-self.motion.shift*2]*power_right

            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.mid_th1[i]*self.rviz_reversal*power_left
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.motion.mid_phi_chord1[i]*power_left
            self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.mid_th2[i]*power_left
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.motion.mid_th1[i]*self.rviz_reversal*power_right
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.mid_phi_chord1[i]*power_right
            self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.mid_th2[i]*power_right

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

            time.sleep(self.motion.wing_period)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)

    def oscillate_back_fin(self, throttle, RT_button, RB_button):
        loop_count = len(self.motion.mid_th1)
        back_reversal = -1

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        for i in range(0, loop_count):
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            if RT_button == 1 and RB_button == 0: # top fin only
                # L middle & bottom fin stopping
                for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                    self.joint_state_msg.position[j] = 0
                # R middle & bottom fin stopping
                for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                    self.joint_state_msg.position[j] = 0

            elif RT_button == 0 and RB_button == 1: # top and middle fin
                # middle fin oscillate
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]*back_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]*back_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]
                # bottom fin stopping
                for j in range(gv.jname.index("rev_link_Lbtm_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                    self.joint_state_msg.position[j] = 0                
                for j in range(gv.jname.index("rev_link_Rbtm_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                    self.joint_state_msg.position[j] = 0  

            elif RT_button == 0 and RB_button == 0: # all fin oscillate
                # middle fin oscillate
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]*back_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]*back_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]

                # top fin oscillate
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift*2]*back_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.mid_th2[i-self.motion.shift*2]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift*2]*back_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.mid_th2[i-self.motion.shift*2]

            # bottom fin oscillate                
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.motion.mid_th1[i]*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.mid_phi_chord1[i]*back_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.mid_th2[i]
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.mid_th1[i]*self.rviz_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = -self.motion.mid_phi_chord1[i]*back_reversal
            self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.mid_th2[i]

            self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

            time.sleep(self.motion.wing_period)

            # convert RViz parameter
            self.convert_rviz_param(self.joint_state_msg)

            #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
            self.pub_joint_state.publish(self.joint_state_msg)

    def spin_turn(self, aileron, throttle, RT_button, RB_button):
        loop_count = len(self.motion.mid_th1)

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        if aileron > 0.5:
            for i in range(0, loop_count):
                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

                self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

                if RT_button == 1 and RB_button == 0: # top fin only
                    # R middle & bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                        self.joint_state_msg.position[j] = 0

                elif RT_button == 0 and RB_button == 1: # top and middle fin
                    # middle fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]
                    # R bttom fin stopping
                    for j in range(gv.jname.index("rev_link_Rbtm_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                        self.joint_state_msg.position[j] = 0  

                elif RT_button == 0 and RB_button == 0: # all fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift*2]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.mid_th2[i-self.motion.shift*2]

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.motion.mid_th1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.mid_phi_chord1[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.mid_th2[i]

                # Left fin stopping
                for j in range(gv.jname.index("rev_link_Ltop_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                    self.joint_state_msg.position[j] = 0


                time.sleep(self.motion.wing_period)
                # convert RViz parameter
                self.convert_rviz_param(self.joint_state_msg)

                #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
                self.pub_joint_state.publish(self.joint_state_msg)



        else: # aileron < -0.5
            for i in range(0, loop_count):
                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

                self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

                if RT_button == 1 and RB_button == 0: # top fin only
                    # L middle & bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                        self.joint_state_msg.position[j] = 0

                elif RT_button == 0 and RB_button == 1: # top and middle fin
                    # middle fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                    # L bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Lbtm_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                        self.joint_state_msg.position[j] = 0  

                elif RT_button == 0 and RB_button == 0: # all fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift*2]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.mid_th2[i-self.motion.shift*2]

                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.mid_th1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.motion.mid_phi_chord1[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.mid_th2[i]

                # R middle & bottom fin stopping
                for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                    self.joint_state_msg.position[j] = 0

                time.sleep(self.motion.wing_period)
                # convert RViz parameter
                self.convert_rviz_param(self.joint_state_msg)

                #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
                self.pub_joint_state.publish(self.joint_state_msg)

    # Push the wings up 45° 
    def original_turn(self, aileron, throttle, RT_button, RB_button):
        loop_count = len(self.motion.mid_th1)
        self.turn_r = 30.0
        self.turn_p = 0.0

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        if aileron > 0.5:
            for i in range(0, loop_count):
                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

                self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

                if RT_button == 1 and RB_button == 0: # top fin only
                    # R middle & bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                        self.joint_state_msg.position[j] = 0

                elif RT_button == 0 and RB_button == 1: # top and middle fin
                    # middle fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]
                    # R bttom fin stopping
                    for j in range(gv.jname.index("rev_link_Rbtm_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                        self.joint_state_msg.position[j] = 0  

                elif RT_button == 0 and RB_button == 0: # all fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift*2]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.mid_th2[i-self.motion.shift*2]

                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.turn_r*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.turn_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.turn_r
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.turn_r*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.turn_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.turn_r
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.turn_r*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.turn_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.turn_r

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.motion.mid_th1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.mid_phi_chord1[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.mid_th2[i]

                time.sleep(self.motion.wing_period)

                # convert RViz parameter
                self.convert_rviz_param(self.joint_state_msg)

                #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
                self.pub_joint_state.publish(self.joint_state_msg)

        else: # aileron < -0.5
            for i in range(0, loop_count):
                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

                self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

                if RT_button == 1 and RB_button == 0: # top fin only
                    # L middle & bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                        self.joint_state_msg.position[j] = 0

                elif RT_button == 0 and RB_button == 1: # top and middle fin
                    # middle fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                    # L bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Lbtm_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                        self.joint_state_msg.position[j] = 0  

                elif RT_button == 0 and RB_button == 0: # all fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift*2]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.mid_th2[i-self.motion.shift*2]

                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.mid_th1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.motion.mid_phi_chord1[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.mid_th2[i]

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.turn_r*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.turn_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.turn_r
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.turn_r*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.turn_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.turn_r
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.turn_r*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = -self.turn_p
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.turn_r

                time.sleep(self.motion.wing_period)

                # convert RViz parameter
                self.convert_rviz_param(self.joint_state_msg)

                #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
                self.pub_joint_state.publish(self.joint_state_msg)

    def superstrength_turn(self, aileron, throttle, RT_button, RB_button):
        loop_count = len(self.motion.mid_th1)
        back_reversal = -1

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        if aileron > 0.5:
            for i in range(0, loop_count):
                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

                self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

                if RT_button == 1 and RB_button == 0: # top fin only
                    # R middle & bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                        self.joint_state_msg.position[j] = 0

                elif RT_button == 0 and RB_button == 1: # top and middle fin
                    # middle fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]*back_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]            
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]
                    # R bttom fin stopping
                    for j in range(gv.jname.index("rev_link_Rbtm_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                        self.joint_state_msg.position[j] = 0  

                elif RT_button == 0 and RB_button == 0: # all fin oscillate
                    # middle fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]*back_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                    # top fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift*2]*back_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.mid_th2[i-self.motion.shift*2]

                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift*2]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.mid_th2[i-self.motion.shift*2]

                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.motion.mid_th1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.mid_phi_chord1[i]*back_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.mid_th2[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.motion.mid_th1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.mid_phi_chord1[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.mid_th2[i]

                time.sleep(self.motion.wing_period)
                # convert RViz parameter
                self.convert_rviz_param(self.joint_state_msg)

                #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
                self.pub_joint_state.publish(self.joint_state_msg)

        else: # aileron < -0.5
            for i in range(0, loop_count):
                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

                self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

                if RT_button == 1 and RB_button == 0: # top fin only
                    # L middle & bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                        self.joint_state_msg.position[j] = 0

                elif RT_button == 0 and RB_button == 1: # top and middle fin
                    # middle fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                    # middle fin back oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]*back_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]
                    # L bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Lbtm_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                        self.joint_state_msg.position[j] = 0

                elif RT_button == 0 and RB_button == 0: # all fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = self.motion.mid_phi_chord1[i-self.motion.shift*2]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.mid_th2[i-self.motion.shift*2]

                    # middle & top fin back oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift]*back_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = -self.motion.mid_phi_chord1[i-self.motion.shift*2]*back_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.mid_th2[i-self.motion.shift*2]

                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.mid_th1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = self.motion.mid_phi_chord1[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.mid_th2[i]
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.mid_th1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = -self.motion.mid_phi_chord1[i]*back_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.mid_th2[i]


                time.sleep(self.motion.wing_period)
                # convert RViz parameter
                self.convert_rviz_param(self.joint_state_msg)

                #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
                self.pub_joint_state.publish(self.joint_state_msg)

    def lateral_move(self, rudder, throttle, RT_button, RB_button):
        loop_count = len(self.motion.mid_th1)

        # head parameter
        if throttle > 0.5:  # head is up, and floating
            df_value = 20.0
        elif throttle < -0.5: # head is down, and diving
            df_value = -20.0
        else:
            df_value = 0.0

        if rudder > 0.5:
            for i in range(0, loop_count):
                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

                self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

                if RT_button == 1 and RB_button == 0: # top fin only
                    # R middle & bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                        self.joint_state_msg.position[j] = 0

                elif RT_button == 0 and RB_button == 1: # top and middle fin
                    # middle fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = 0
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i]
                    # R bttom fin stopping
                    for j in range(gv.jname.index("rev_link_Rbtm_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                        self.joint_state_msg.position[j] = 0  

                elif RT_button == 0 and RB_button == 0: # all fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll1")] = -self.motion.mid_th1[i]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_pitch")] = 0
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rmid_roll2")] = -self.motion.mid_th2[i]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll1")] = -self.motion.mid_th1[i]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_pitch")] = 0
                    self.joint_state_msg.position[gv.jname.index("rev_link_Rbtm_roll2")] = -self.motion.mid_th2[i]

                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll1")] = -self.motion.mid_th1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_pitch")] = 0
                self.joint_state_msg.position[gv.jname.index("rev_link_Rtop_roll2")] = -self.motion.mid_th2[i]

                # Left fin stopping
                for j in range(gv.jname.index("rev_link_Ltop_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                    self.joint_state_msg.position[j] = 0


                time.sleep(self.motion.wing_period)
                # convert RViz parameter
                self.convert_rviz_param(self.joint_state_msg)

                #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
                self.pub_joint_state.publish(self.joint_state_msg)

        else: # rudder < -0.5
            for i in range(0, loop_count):
                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

                self.joint_state_msg.position[gv.jname.index("rev_link_head")] = df_value*self.rviz_reversal

                if RT_button == 1 and RB_button == 0: # top fin only
                    # L middle & bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Lmid_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                        self.joint_state_msg.position[j] = 0

                elif RT_button == 0 and RB_button == 1: # top and middle fin
                    # middle fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = 0
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                    # L bottom fin stopping
                    for j in range(gv.jname.index("rev_link_Lbtm_roll1"), gv.jname.index("rev_link_Lbtm_roll2")):
                        self.joint_state_msg.position[j] = 0  

                elif RT_button == 0 and RB_button == 0: # all fin oscillate
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll1")] = self.motion.mid_th1[i-self.motion.shift]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_pitch")] = 0
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lmid_roll2")] = self.motion.mid_th2[i-self.motion.shift]
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll1")] = self.motion.mid_th1[i-self.motion.shift*2]*self.rviz_reversal
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_pitch")] = 0
                    self.joint_state_msg.position[gv.jname.index("rev_link_Lbtm_roll2")] = self.motion.mid_th2[i-self.motion.shift*2]

                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll1")] = self.motion.mid_th1[i]*self.rviz_reversal
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_pitch")] = 0
                self.joint_state_msg.position[gv.jname.index("rev_link_Ltop_roll2")] = self.motion.mid_th2[i]

                # R middle & bottom fin stopping
                for j in range(gv.jname.index("rev_link_Rmid_roll1"), gv.jname.index("rev_link_Rbtm_roll2")):
                    self.joint_state_msg.position[j] = 0

                time.sleep(self.motion.wing_period)
                # convert RViz parameter
                self.convert_rviz_param(self.joint_state_msg)

                #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
                self.pub_joint_state.publish(self.joint_state_msg)

    def wing_stopping(self):
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(0, len(self.joint_state_msg.position)):
            self.joint_state_msg.position[i] = 0
        time.sleep(self.motion.wing_period)

        # convert RViz parameter
        self.convert_rviz_param(self.joint_state_msg)

        #self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
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