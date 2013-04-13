#!/usr/bin/env python
import rospy
import serial
from wrist_node.msg import RawPosition

class SensorNode:
    def __init__(self):
        rospy.init_node('sensor_node')

        # get parameters
        self.w1_addr = rospy.get_param('w1_addr', 0)
        self.w2_addr = rospy.get_param('w2_addr', 1)

        self.serialport = rospy.get_param('dynamixel_port', '/dev/ttyUSB1')
        self.baud_rate = rospy.get_param('baud_rate', 1000000)

        # open serial port
        self.ser = serial.Serial(self.serialport, self.baud_rate, timeout=0.3)        

        # publishers
        self.cur_pos_pub = rospy.Publisher('wrist_diff/raw_pos', RawPosition)

    def run(self):
        while not rospy.is_shutdown():
            # publish current position
            self.ser.flushInput()
            self.ser.write('get\n')
            self.ser.flush()
            data = self.ser.readline()
            rospy.loginfo('data recv: %s' % data)

            vals = data.split(',')

            self.w1pos = vals[self.w1_addr]
            self.w2pos = vals[self.w2_addr]
            
            self.cur_pos_pub.publish(time=rospy.get_rostime(), w1pos=self.w1pos, w2pos=self.w2pos)

            rospy.sleep(0.010)

if __name__ == '__main__':
    try:
        sensor = SensorNode()    
        sensor.run()
    except rospy.ROSInterruptException:
        pass
