class GlobalValues(object):
    def __init__(self):

        # ServoAngle error
        self.head_p_err = 15
        self.ltop_r1_err, self.ltop_p_err, self.ltop_r2_err = 10, 0, 0
        self.lmid_r1_err, self.lmid_p_err, self.lmid_r2_err = 10, 20, 10
        self.lbtm_r1_err, self.lbtm_p_err, self.lbtm_r2_err = 8, 0, 15

        self.rtop_r1_err, self.rtop_p_err, self.rtop_r2_err = 15, 0, 20
        self.rmid_r1_err, self.rmid_p_err, self.rmid_r2_err = 20, 20, 27
        self.rbtm_r1_err, self.rbtm_p_err, self.rbtm_r2_err = 20, 7, 5

        # Servo ID
        # adafruit_servokit_1, I2C address = 0x41
        self.ltop_r1_id, self.ltop_p_id, self.ltop_r2_id = 0, 1, 2
        self.head_p_id = 3
        self.rtop_r1_id, self.rtop_p_id, self.rtop_r2_id = 4, 5, 6
        self.rbtm_r2_id = 7

        # adafruit_servokit_0, I2C address = 0x40
        self.lbtm_r1_id, self.lbtm_p_id, self.lbtm_r2_id = 0, 1, 2
        self.lmid_r1_id, self.lmid_p_id, self.lmid_r2_id = 4, 5, 6

        self.rmid_r1_id, self.rmid_p_id, self.rmid_r2_id = 8, 9, 10
        self.rbtm_r1_id, self.rbtm_p_id = 12, 13

        # Servo link list
        self.servoname_list = [
            self.head_p_id, \
            self.ltop_r1_id, self.ltop_p_id, self.ltop_r2_id, \
            self.lmid_r1_id, self.lmid_p_id, self.lmid_r2_id, \
            self.lbtm_r1_id, self.lbtm_p_id, self.lbtm_r2_id, \
            self.rtop_r1_id, self.rtop_p_id, self.rtop_r2_id, \
            self.rmid_r1_id, self.rmid_p_id, self.rmid_r2_id, \
            self.rbtm_r1_id, self.rbtm_p_id, self.rbtm_r2_id, \
        ]

        self.jname = [
            "rev_link_head",     
            "rev_link_Ltop_roll1", "rev_link_Ltop_pitch", "rev_link_Ltop_roll2",
            "rev_link_Lmid_roll1", "rev_link_Lmid_pitch", "rev_link_Lmid_roll2",
            "rev_link_Lbtm_roll1", "rev_link_Lbtm_pitch", "rev_link_Lbtm_roll2",
            "rev_link_Rtop_roll1", "rev_link_Rtop_pitch", "rev_link_Rtop_roll2",
            "rev_link_Rmid_roll1", "rev_link_Rmid_pitch", "rev_link_Rmid_roll2",
            "rev_link_Rbtm_roll1", "rev_link_Rbtm_pitch", "rev_link_Rbtm_roll2",   
        ]

gv = GlobalValues() # Create Instance