"""
    E-Puck robot following the wall

    Alberto QUINTERO DELGADO
    05/MAR/2013
"""

from controller import *

TIME_STEP = 8
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052
ENCODER_RESOLUTION = 159.23
RANGE = 512

F_D = 100 # Frontal distance
D_D = 150 # diagonal distance
L_D = 300 # lateral distance
D_R = 5   # distance range

class E_Puck (DifferentialWheels):
    def __init__(self):
        DifferentialWheels.__init__(self)
        self.speed = [1000, 1000]
        self.sensorToCheck = -1
        self.wallInSensor = -1
        self.turning = False

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
                    if (diag2 < diag1):
                        self.goRight()
                    else:
                        self.goLeft()
                else: 
                    self.goStraight()

            #set speed
            self.setSpeed(self.speed[0], self.speed[1])

epuck = E_Puck()
epuck.run()