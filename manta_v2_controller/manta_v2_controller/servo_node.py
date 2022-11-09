import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from .control_modules.servo_globals import *
from adafruit_servokit import ServoKit
import time

class ServoWriter(object):
    def __init__(self):
        self.servokit_0 = ServoKit(channels=16, address=0x40, frequency=50)
        self.servokit_1 = ServoKit(channels=16, address=0x41, frequency=50)

        self.default_angle = 90

    def servo_write_0(self, id, angle, angle_error, sleep_time = 0.0001):
        angle_0 = self.default_angle + angle + angle_error
        if angle_0 < 45 + angle_error:
            angle_0 = 45 + angle_error
        elif angle_0 > 135 + angle_error:
            angle_0 = 135 + angle_error

        # debag
        #print("servo angle raw: {}".format(angle_0))
        self.servokit_0.servo[id].angle = angle_0
        #time.sleep( sleep_time )

    def servo_write_1(self, id, angle, angle_error, sleep_time = 0.0001):
        angle_1 = self.default_angle + angle + angle_error
        if angle_1 < 45 + angle_error:
            angle_1 = 45 + angle_error
        elif angle_1 > 135 + angle_error:
            angle_1 = 135 + angle_error

        # debag
        #print("servo angle raw: {}".format(angle_1))
        self.servokit_1.servo[id].angle = angle_1
        #time.sleep( sleep_time )


class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('sub_joint_state')
        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.sub_joint_states  # prevent unused variable warning

        self.get_logger().info("Preparing servo kit")
        self.servo = ServoWriter()
        self.get_logger().info("Servo kit ready!")

        self.hw_param_head = 400/7
        self.hw_param_roll1 = 80
        self.hw_param = 60
        self.hw_reversal = -1

    def listener_callback(self, joint_msg):
        # self.get_logger().info(' joystick axes: "%s"\n' % joy_msg.axes)
        # self.get_logger().info(' joystick buttons: "%s"\n' % joy_msg.buttons)
        if len(joint_msg.position) > 0:

            self.get_logger().info('{}: {}'.format(joint_msg.name[0], joint_msg.position[0]*60))

            self.servo.servo_write_1(gv.head_p_id, joint_msg.position[joint_msg.name.index("rev_link_head")]*self.hw_param_head*self.hw_reversal, gv.head_p_err)

            self.servo.servo_write_1(gv.ltop_r1_id, joint_msg.position[joint_msg.name.index("rev_link_Ltop_roll1")]*self.hw_param_roll1*self.hw_reversal, gv.ltop_r1_err)
            self.servo.servo_write_1(gv.ltop_p_id, joint_msg.position[joint_msg.name.index("rev_link_Ltop_pitch")]*self.hw_param, gv.ltop_p_err)
            self.servo.servo_write_1(gv.ltop_r2_id, joint_msg.position[joint_msg.name.index("rev_link_Ltop_roll2")]*self.hw_param, gv.ltop_r2_err)
            self.servo.servo_write_0(gv.lmid_r1_id, joint_msg.position[joint_msg.name.index("rev_link_Lmid_roll1")]*self.hw_param_roll1*self.hw_reversal, gv.lmid_r1_err)
            self.servo.servo_write_0(gv.lmid_p_id, joint_msg.position[joint_msg.name.index("rev_link_Lmid_pitch")]*self.hw_param, gv.lmid_p_err)
            self.servo.servo_write_0(gv.lmid_r2_id, joint_msg.position[joint_msg.name.index("rev_link_Lmid_roll2")]*self.hw_param, gv.lmid_r2_err)
            self.servo.servo_write_0(gv.lbtm_r1_id, joint_msg.position[joint_msg.name.index("rev_link_Lbtm_roll1")]*self.hw_param_roll1*self.hw_reversal, gv.lbtm_r1_err)
            self.servo.servo_write_0(gv.lbtm_p_id, joint_msg.position[joint_msg.name.index("rev_link_Lbtm_pitch")]*self.hw_param, gv.lbtm_p_err)
            self.servo.servo_write_0(gv.lbtm_r2_id, joint_msg.position[joint_msg.name.index("rev_link_Lbtm_roll2")]*self.hw_param, gv.lbtm_r2_err)

            self.servo.servo_write_1(gv.rtop_r1_id, joint_msg.position[joint_msg.name.index("rev_link_Rtop_roll1")]*self.hw_param_roll1*self.hw_reversal, gv.rtop_r1_err)
            self.servo.servo_write_1(gv.rtop_p_id, joint_msg.position[joint_msg.name.index("rev_link_Rtop_pitch")]*self.hw_param, gv.rtop_p_err)
            self.servo.servo_write_1(gv.rtop_r2_id, joint_msg.position[joint_msg.name.index("rev_link_Rtop_roll2")]*self.hw_param, gv.rtop_r2_err)
            self.servo.servo_write_0(gv.rmid_r1_id, joint_msg.position[joint_msg.name.index("rev_link_Rmid_roll1")]*self.hw_param_roll1*self.hw_reversal, gv.rmid_r1_err)
            self.servo.servo_write_0(gv.rmid_p_id, joint_msg.position[joint_msg.name.index("rev_link_Rmid_pitch")]*self.hw_param, gv.rmid_p_err)
            self.servo.servo_write_0(gv.rmid_r2_id, joint_msg.position[joint_msg.name.index("rev_link_Rmid_roll2")]*self.hw_param, gv.rmid_r2_err)
            self.servo.servo_write_0(gv.rbtm_r1_id, joint_msg.position[joint_msg.name.index("rev_link_Rbtm_roll1")]*self.hw_param_roll1*self.hw_reversal, gv.rbtm_r1_err)
            self.servo.servo_write_0(gv.rbtm_p_id, joint_msg.position[joint_msg.name.index("rev_link_Rbtm_pitch")]*self.hw_param, gv.rbtm_p_err)
            self.servo.servo_write_1(gv.rbtm_r2_id, joint_msg.position[joint_msg.name.index("rev_link_Rbtm_roll2")]*self.hw_param, gv.rbtm_r2_err)

def main(args=None):
    rclpy.init(args=args)

    joint_states_sub = JointStateSubscriber()

    rclpy.spin(joint_states_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_states_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
