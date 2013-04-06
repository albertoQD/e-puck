"""
    E-Puck robot stopping before hitting the wall
    
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
    def computeOdometry(self):
        dl = self.getLeftEncoder() / ENCODER_RESOLUTION * WHEEL_RADIUS
        dr = self.getRightEncoder() / ENCODER_RESOLUTION * WHEEL_RADIUS
        da = (dr - dl) / AXLE_LENGTH
        print 'Estimated distance covered by left wheel: ', dl*100, 'cm'
        print 'Estimated distance covered by right wheel: ',dr*100, 'cm'
        print 'Estimated change of orientation: ', da, 'rad'

    def run(self):
        #vars
        speed = [1000, 1000] #Maximum speed
        coef = [ [150, -35], [100, -15], [80, -10], [-10, -10], [-10, -10], [-10, 80], [-30, 100], [-20, 150] ]

        #enable devices
        self.getAccelerometer('accelerometer').enable(TIME_STEP*4)
        self.enableEncoders(TIME_STEP*4)

        #distance sensors (Enabling)
        sensors = map(lambda x: 'ps'+str(x), xrange(0,8))
        map(lambda x: self.getDistanceSensor(x).enable(TIME_STEP*4), sensors)

        #main loop
        while (self.step(TIME_STEP) != -1):
            self.computeOdometry()

            # === FOR STOPING THE CAR AT 2 cms ==== #
            # Checking just for IR7 and IR0
            if (self.getDistanceSensor(sensors[7]).getValue() > 150 or \
                self.getDistanceSensor(sensors[0]).getValue() > 150):
                speed = [0, 0] #Stop

            #set speed
            self.setSpeed(speed[0], speed[1])

epuck = E_Puck()
epuck.run()
