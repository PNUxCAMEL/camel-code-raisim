import sys
libPath = 'C:/Users/Jaehoon/raisimLib/CAMEL-code/lib'
sys.path.append(libPath)

import PDController
import numpy as np

class SimplePendulumPDController(PDController.PDController):

    def __init__(self, robot):
        super().__init__(robot)
        self.setPDGain(PGain=250.0, DGain=20.0)
        self.setTrajectory(desiredPosition=1.57, desiredVelocity=0.0)
        self.iteration = 0

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
        self.torque = self.PGain * (self.desiredPosition - self.position) + self.DGain * (self.desiredVelocity - self.velocity)
        
    # override
    def setControlInput(self):
        self.robot.setGeneralizedForce(np.array([self.torque]))        
