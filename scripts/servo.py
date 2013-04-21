from lib_robotis import *
from pid_controller import *
from datetime import datetime
import rospy

class Servo(Robotis_Servo):
    def __init__(self, dyn, addr, flip):
        super(Servo, self).__init__(dyn, addr, 'MX')
        self.pos = 0
        self.load = 0
        self.spd = 0
        self.enc = 0
        self.p_enc = 0
        self.ang = 0
        self.voltage = 0
        self.flip = flip
        self.stopped = False

    def set_spd(self, spd):
        if self.flip:
            self.spd = -spd
        else:
            self.spd = spd

        if self.spd > 21:
            self.spd = 21
        elif self.spd < -21:
            self.spd = -21

        try:
            if self.stopped:
                self.set_angvel(0.0)
            else:
                self.set_angvel(self.spd)
        except:
            self.set_angvel(0.0)

    def reset(self):
        self.update()
        self.p_enc = self.enc
        self.ang = 0

class Differential(object):
    def __init__(self, dyn, s1, s2):
        self.w1 = Servo(dyn, s1, True)
        self.w2 = Servo(dyn, s2, False)

        self.w1.init_cont_turn()
        self.w2.init_cont_turn()

        self.angle_pid = PID(1, 0, 0, 100, 0, 'angle')
        self.rotate_pid = PID(1, 0, 0, 100, 0, 'rotate')

        self.angle = 0
        self.rotate = 0

    def setAnglePID(self, kP, kI, kD):
        self.angle_pid.setParams(kP, kI, kD)

    def setRotatePID(self, kP, kI, kD):
        self.rotate_pid.setParams(kP, kI, kD)

    def setAngle(self, angle):
        self.angle_pid.setSetpoint(angle)

    def setRotate(self, rotate):
        self.rotate_pid.setSetpoint(rotate)

    def process(self, angle, rotate, dt):
        pi = 3.1415926535

        self.angle = angle
        self.rotate = rotate
        
        self.angle_pid.update(self.angle, dt)
        self.rotate_pid.update(self.rotate, dt)

        self.set_speed(self.angle_pid.getOutput(), self.rotate_pid.getOutput())

    def set_speed(self, spd, turn=0):
        maxspd = 10
        # limit angle velocity
        w1spd = min(max(spd + turn, -maxspd), maxspd)
        w2spd = min(max(spd - turn, -maxspd), maxspd)
        rospy.loginfo(rospy.get_name() + ' %.02f %.02f [%.02f %.02f]' % (w1spd, w2spd, spd, turn))
        self.w1.set_spd(w1spd)
        self.w2.set_spd(w2spd)
