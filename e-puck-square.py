"""
    E-Puck robot moving in a Square

    Alberto QUINTERO DELGADO
    05/MAR/2013
"""

from controller import *

TIME_STEP = 8
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052
ENCODER_RESOLUTION = 159.23
RANGE = 512

class E_Puck (DifferentialWheels):
    def orientationChange(self):
        dl = self.getLeftEncoder() / ENCODER_RESOLUTION * WHEEL_RADIUS
        dr = self.getRightEncoder() / ENCODER_RESOLUTION * WHEEL_RADIUS
        return (dr - dl) / AXLE_LENGTH

    def meanDistance(self):
        dl = self.getLeftEncoder() / ENCODER_RESOLUTION * WHEEL_RADIUS
        dr = self.getRightEncoder() / ENCODER_RESOLUTION * WHEEL_RADIUS
        return ((dl + dr)/2)*100

    def run(self):
        #vars
        speed = [1000, 1000] #Maximum speed
        coef = [ [150, -35], [100, -15], [80, -10], [-10, -10], [-10, -10], [-10, 80], [-30, 100], [-20, 150] ]
        turn = False

        #enable devices
        self.enableEncoders(TIME_STEP*4)
        #distance sensors (Enabling)
        sensors = map(lambda x: 'ps'+str(x), xrange(0,8))
        map(lambda x: self.getDistanceSensor(x).enable(TIME_STEP*4), sensors)

        #main loop
        while (self.step(TIME_STEP) != -1):
            dist = self.meanDistance()
            print 'Distance: ', dist
            if ((round(dist) > 0) and (round(dist) % 25 == 0) and (not turn)):
                speed = [-100, 450] #coef[7]
                turn = True
            elif (turn and (self.orientationChange() >= 1.8)):
                self.setEncoders(0, 0) # reset the encoders
                speed = [1000, 1000]
                turn = False

            #set speed
            self.setSpeed(speed[0], speed[1])

epuck = E_Puck()
epuck.run()