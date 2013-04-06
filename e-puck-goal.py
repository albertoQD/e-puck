"""
    E-Puck robot going to a goal avoiding obstacles

    Alberto QUINTERO DELGADO
    05/MAR/2013
"""

from controller import *
from numpy import arange
import math

TIME_STEP = 1
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052
ENCODER_RESOLUTION = 159.23
RANGE = 512

F_D = 100  # Frontal distance
D_D = 150  # diagonal distance
L_D = 300  # lateral distance
D_R = 5    # distance range

GX = -0.1  # X Coordinate of the Goal
GZ = -1.2  # Z Coordinate of the Goal
GA =  0.0  # Angel of the Goal
EX =  0.1  # X Coordinate of the E-Puck
EZ =  0.0  # Z Coordinate of the E-Puck

class E_Puck (DifferentialWheels):
    def reset(self):
        self.speed = [1000, 1000]
        self.sensorToCheck = -1
        self.wallInSensor = -1
        self.turning = False

    def __init__(self):
        DifferentialWheels.__init__(self)
        self.reset()
        self.prevX = EX
        self.prevZ = EZ
        self.prevD = 0.0
        self.prevR = 0.0
        self.prevL = 0.0

    def updatePosition(self):
        le = (self.getLeftEncoder() / ENCODER_RESOLUTION) * WHEEL_RADIUS
        re = (self.getRightEncoder() / ENCODER_RESOLUTION) * WHEEL_RADIUS
        dl = le - self.prevL
        dr = re - self.prevR
        da = (dr - dl) / AXLE_LENGTH # Angle
        ds = (dr + dl)/2.0

        if (self.prevD + da > math.pi or \
            self.prevD + da < -math.pi):
            self.prevD *= -1

        self.prevD += da

        dz = ds * math.cos(self.prevD + da/2.0)
        dx = ds * math.sin(self.prevD + da/2.0)

        self.prevZ -= dz
        self.prevX -= dx
        self.prevL = le
        self.prevR = re

    def turn(self, _speed, _toCheck, _wallIn):
        self.turning = True
        self.speed = _speed
        self.sensorToCheck = _toCheck
        self.wallInSensor = _wallIn

    def goLeft(self):
        self.turn([-100, 900], 2, 1)

    def goRight(self):
        self.turn([900, -100], 5, 6)

    def goStraight(self):
        self.turning = False 
        self.speed = [1000, 1000]

    def run(self):

        #enable devices
        self.enableEncoders(TIME_STEP*4)
        #distance sensors (Enabling)
        sensors = map(lambda x: 'ps'+str(x), xrange(0,8))
        map(lambda x: self.getDistanceSensor(x).enable(TIME_STEP*4), sensors)

        #main loop
        while (self.step(TIME_STEP) != -1):
            front1 = self.getDistanceSensor(sensors[7]).getValue()
            front2 = self.getDistanceSensor(sensors[0]).getValue()
            diag1 = self.getDistanceSensor(sensors[6]).getValue()
            diag2 = self.getDistanceSensor(sensors[1]).getValue()

            self.updatePosition()
            zeq = (round(self.prevZ,2) - EZ)
            xeq = (6*round(self.prevX,2))-0.6 # Equation of M-Line - UPDATE TO USE YOURS
            xeqR = map(lambda x: round(x, 2), arange(xeq-0.11, xeq+0.11, 0.01)) # set a range to search

            if ( zeq in xeqR \
                 and self.sensorToCheck != -1 \
                 and front1 < F_D \
                 and front2 < F_D):
                # belongs to M-Line
                self.reset()
                if (diag2 < diag1):
                    self.speed = [900, -100]
                else:
                    self.speed = [-100, 900]

                self.setSpeed(self.speed[0], self.speed[1])
                while (self.step(TIME_STEP) != -1):
                    self.updatePosition()
                    if(round(self.prevD, 1) in [GA-0.1, GA, GA+0.1]): break

                self.reset()

            if (self.sensorToCheck != -1):
                c_lateral = self.getDistanceSensor(sensors[self.sensorToCheck]).getValue()
                c_diag = self.getDistanceSensor(sensors[self.wallInSensor]).getValue()

                # Wall in front
                if ( front1 > F_D or front2 > F_D):
                    if (diag2 < diag1):
                        self.goRight()
                    else:
                        self.goLeft()

                # Wall founded, follow it
                elif ( c_diag < D_D-D_R and c_lateral < L_D-D_R):
                    # Get closer
                    if (self.sensorToCheck == 2):
                        self.goRight()
                        self.sensorToCheck = 2
                        self.wallInSensor = 1
                    else:
                        self.goLeft()
                        self.sensorToCheck = 5
                        self.wallInSensor = 6

                elif (c_diag > L_D+D_R and c_lateral > L_D+D_R):
                    # Get away
                    if (self.sensorToCheck == 5):
                        self.goRight()
                    else:
                        self.goLeft()
                else:
                    self.goStraight()
                    
            else:
                # Wall not founded yet
                if (front1 > F_D or front2 > F_D or diag1 > D_D-D_R or diag2 > D_D-D_R):
                    # Wall founded start turning
                    # if (diag2 < diag1):
                    self.goRight() # ALWAIS TURN TO THE RIGHT
                    # else:
                    #     self.goLeft()
                else: 
                    self.goStraight()

            #set speed
            self.setSpeed(self.speed[0], self.speed[1])

epuck = E_Puck()
epuck.run()