#!/usr/bin/env python
import rospy
from lib_robotis import *
from servo import *
from wrist_node.msg import RawPosition, Position

class WristNode:
    def __init__(self):
        rospy.init_node('wrist_node')

        # get parameters
        self.w1_addr = rospy.get_param('w1_addr', 1)
        self.w2_addr = rospy.get_param('w2_addr', 2)

        self.serialport = rospy.get_param('dynamixel_port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('baud_rate', 1000000)

        rotate_Kp = rospy.get_param('rotate_Kp', 1)
        rotate_Ki = rospy.get_param('rotate_Ki', 0)
        rotate_Kd = rospy.get_param('rotate_Kd', 0)

        angle_Kp = rospy.get_param('angle_Kp', 1)
        angle_Ki = rospy.get_param('angle_Ki', 0)
        angle_Kd = rospy.get_param('angle_Kd', 0)

        # connect to dynamixels
        self.dyn = USB2Dynamixel_Device(self.serialport, self.baud_rate)
        self.wrist = Wrist(dyn, self.w1_addr, self.w2_addr)

        # set PID
        self.wrist.setAnglePID(angle_Kp, angle_Ki, angle_Kd)
        self.wrist.setRotatePID(rotate_Kp, rotate_Ki, rotate_Kd)

        # publishers
        self.cur_pos_pub = rospy.Publisher('wrist_diff/cur_pos', Position)

        # subscribers
        self.des_pos_sub = rospy.Subscriber('wrist_diff/desired_pos', Position, self.newpos_cb)
        self.raw_pos_sub = rospy.Subscriber('wrist_diff/raw_pos', RawPosition, self.rawpos_cb)

        # initialize variables
        self.rotate_sp = 0
        self.angle_sp = 0
        self.w1_pos = 0
        self.w2_pos = 0

    def run(self):
        while not rospy.is_shutdown():
            # set setpoints
            self.wrist.setAngle(self.angle_sp)
            self.wrist.setRotate(self.rotate_sp)

            # run PID control
            self.wrist.process(self.w1_pos, self.w2_pos, 0.01)

            # publish current position
            self.cur_pos_pub.publish(time=rospy.get_rostime(), angle=self.wrist.angle, rotate=self.wrist.rotate)

            rospy.sleep(0.010)

    def newpos_cb(self, data):
        rospy.loginfo(rospy.get_name() + ': desired pos: %.02f %02f' % (data.rotate, data.angle))
        self.rotate_sp = data.rotate
        self.angle_sp = data.angle

    def rawpos_cb(self, data):
        rospy.loginfo(rospy.get_name() + ': rawpos: %d %d' % (data.w1pos, data.w2pos))
        self.w1_pos = data.w1pos
        self.w2_pos = data.w2pos

if __name__ == '__main__':
    try:
        wrist = WristNode()    
        wrist.run()
    except rospy.ROSInterruptException:
        pass
