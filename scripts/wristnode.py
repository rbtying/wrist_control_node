#!/usr/bin/env python
import rospy
from lib_robotis import *
from std_msgs.msg import *
from servo import *
from wrist_node.msg import RawPosition, Data

class WristNode:
    def __init__(self):
        rospy.init_node('wrist_node')

        # get parameters
        self.w1_addr = rospy.get_param('~w1_addr', 1)
        self.w2_addr = rospy.get_param('~w2_addr', 2)

        self.serialport = rospy.get_param('~dynamixel_port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud_rate', 1000000)

        rotate_Kp = rospy.get_param('~rotate_Kp', 5)
        rotate_Ki = rospy.get_param('~rotate_Ki', 0)
        rotate_Kd = rospy.get_param('~rotate_Kd', 0.5)

        angle_Kp = rospy.get_param('~angle_Kp', 5)
        angle_Ki = rospy.get_param('~angle_Ki', 0)
        angle_Kd = rospy.get_param('~angle_Kd', 0.5)

        # connect to dynamixels
        self.dyn = USB2Dynamixel_Device(self.serialport, self.baud_rate)
        self.wrist = Differential(self.dyn, self.w1_addr, self.w2_addr)

        # set PID
        self.wrist.setAnglePID(angle_Kp, angle_Ki, angle_Kd)
        self.wrist.setRotatePID(rotate_Kp, rotate_Ki, rotate_Kd)

        # publishers
        self.data_pub = rospy.Publisher('wrist_diff/data', Data)

        # subscribers
        self.des_ang_sub = rospy.Subscriber('wrist_diff/desired_angle', Float32, self.angle_cb)
        self.des_rot_sub = rospy.Subscriber('wrist_diff/desired_rotation', Float32, self.rotate_cb)
        self.raw_pos_sub = rospy.Subscriber('wrist_diff/raw_pos', RawPosition, self.rawpos_cb)

        # initialize variables
        self.rotate_sp = 0
        self.angle_sp = 0
        self.angle = 0
        self.rotate = 0
        self.recvddata = False

    def run(self):
        dt = 0.02
        self.rate = rospy.Rate(1/dt)

        while not rospy.is_shutdown():
            # set setpoints
            self.wrist.setAngle(self.angle_sp)
            self.wrist.setRotate(self.rotate_sp)
            # rospy.loginfo(rospy.get_name() + ': setpoints: %f %f' % (self.rotate_sp, self.angle_sp))
            if self.recvddata:
                oldang = self.angle
                oldrot = self.rotate

                # run PID control
                self.wrist.process(self.angle, self.rotate, dt)

                # publish current data
                self.data_pub.publish(timestamp=rospy.get_rostime(), 
                        angle=self.wrist.angle, 
                        rotate=self.wrist.rotate,
                        anglespd=(self.wrist.angle-oldang)/dt,
                        rotatespd=(self.wrist.rotate-oldrot)/dt,
                        angle_output=self.wrist.angle_pid.output,
                        angle_error=self.wrist.angle_pid.error,
                        rotate_output=self.wrist.rotate_pid.output,
                        rotate_error=self.wrist.rotate_pid.error,
                        )

            self.rate.sleep()

    def rotate_cb(self, data):
        pi = 3.1415926535
        self.rotate_sp = max(min(data.data, pi), -pi)

    def angle_cb(self, data):
        pi = 3.1415926535
        self.angle_sp = max(min(data.data, pi/2), -pi/2)

    def rawpos_cb(self, data):
        pi = 3.1415926535
        self.angle = data.angle
        self.rotate = data.rotate
        self.recvddata = True
        # rospy.loginfo(rospy.get_name() + ': rawpos: %.02f %.02f' % (self.angle, self.rotate))

if __name__ == '__main__':
    try:
        wrist = WristNode()    
        wrist.run()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        wrist.wrist.set_speed(0, 0)
        pass
