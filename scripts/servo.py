from lib_robotis import *
from pid_controller import *
from datetime import datetime

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

    def update(self):
        self.enc = self.read_encoder()
        denc = self.enc - self.p_enc
        if self.flip:
            denc = -1 * denc

        if abs(denc) < 3000: 
            self.ang += denc

        self.p_enc = self.enc

        self.load = self.read_load()
        self.voltage = self.read_voltage()

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

        self.angle_pid = PID(0.1, 0, 0, 100, 0)
        self.rotate_pid = PID(0.1, 0, 0, 100, 0)

        self.angle = 0
        self.rotate = 0
        self.w1pos = 0
        self.w2pos = 0

    def setAnglePID(self, kP, kI, kD):
        self.angle_pid.setParams(kP, kI, kD)

    def setRotatePID(self, kP, kI, kD):
        self.rotate_pid.setParams(kP, kI, kD)

    def setAngle(self, angle):
        self.angle_pid.setSetpoint(angle)

    def setRotate(self, rotate):
        self.rotate_pid.setSetpoint(rotate)

    def process(self, w1pos, w2pos, dt):
        self.w1.update()
        self.w2.update()

        self.w1pos = w1pos
        self.w2pos = w2pos

        self.angle = (w1pos + w2pos) / 2
        self.rotate = (w1pos - w2pos) / 2
        
        self.angle_pid.update(self.angle, dt)
        self.rotate_pid.update(self.rotate, dt)

        self.set_speed(self.angle_pid.getOutput(), self.rotate_pid.getOutput())

    def set_speed(self, spd, turn=0):
        self.w1.set_spd(spd + turn)
        self.w2.set_spd(spd - turn)

