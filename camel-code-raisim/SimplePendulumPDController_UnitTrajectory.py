from CAMELController import PDController
import numpy as np
import math

class SimplePendulumPDController_UnitTrajectory(PDController):

    def __init__(self, robot):
        super().__init__(robot)
        self.setPDGain(PGain=100.0, DGain=10.0)
        self.updateState()
        self.setTorqueLimit(5)

    # override
    def doControl(self):
        self.updateState()
        self.setTrajectory(desiredPosition=math.pi * 1.0, desiredVelocity=0.0)
        self.computeControlInput()
        self.setControlInput()

    # override
    def setTrajectory(self, desiredPosition, desiredVelocity):
        return super().setTrajectory(desiredPosition, desiredVelocity)

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
            self.inputTorque = self.torqueLimit
        elif(-self.torqueLimit > self.torque):
            self.robot.setGeneralizedForce(np.array([-self.torqueLimit]))
            self.inputTorque = -self.torqueLimit
        else:
            self.robot.setGeneralizedForce(np.array([self.torque]))  
            self.inputTorque = self.torque      

    def setTorqueLimit(self, torqueLimit):
        self.torqueLimit = torqueLimit

    def getPosition(self):
        return self.position

    def getVelocity(self):
        return self.velocity

    def getDesiredPosition(self):
        return self.desiredPosition
    
    def getDesiredVelocity(self):
        return self.desiredVelocity

    def getInputTorque(self):
        return self.inputTorque