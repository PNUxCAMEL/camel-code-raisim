import PIDController
import numpy as np

class SimplePendulumPIDController(PIDController.PIDController):
    def __init__(self, robot):
        super().__init__(robot)
        self.setPIDGain(150, 1, 15)
        self.setTrajectory(desiredPosition=-1.57, desiredVelocity=0.0)
        
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
        self.cumulativeError += self.positionError
        self.differentialError = self.desiredVelocity - self.velocity
        self.torque = self.PGain * self.positionError + self.IGain * self.cumulativeError + self.DGain * self.differentialError 
        
    # override
    def setControlInput(self):
        self.robot.setGeneralizedForce(np.array([self.torque]))
