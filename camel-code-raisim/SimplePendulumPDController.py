import sys
libPath = 'C:/Users/Jaehoon/raisimLib/CAMEL-code/lib'
sys.path.append(libPath)

import PDController
import numpy as np

class SimplePendulumPDController(PDController.PDController):

    def __init__(self, robot):
        super().__init__(robot)
        self.setPDGain(PGain=100.0, DGain=10.0)
        self.setTrajectory(desiredPosition=-1.57, desiredVelocity=0.0)
        self.setTorqueLimit(50)

    # override
    def doControl(self):
        self.updateState()
        self.computeControlInput()
        self.setControlInput()

    # override
    def setTrajectory(self, desiredPosition, desiredVelocity):
        self.desiredPosition = desiredPosition
        self.desiredVelocity = desiredVelocity

    # override
    def updateState(self):
        self.position = self.robot.getQ()
        self.velocity = self.robot.getQD()           

    # override
    def computeControlInput(self):
        self.positionError = self.desiredPosition - self.position
        self.differentialError = self.desiredVelocity - self.velocity
        self.torque = self.PGain * self.positionError + self.DGain * self.differentialError
        
    # override
    def setControlInput(self):
        if (self.torqueLimit < self.torque):
            self.robot.setGeneralizedForce(np.array([self.torqueLimit]))
        elif(-self.torqueLimit > self.torque):
            self.robot.setGeneralizedForce(np.array([-self.torqueLimit]))
        else:
            self.robot.setGeneralizedForce(np.array([self.torque]))        

    def setTorqueLimit(self, torqueLimit):
        self.torqueLimit = torqueLimit