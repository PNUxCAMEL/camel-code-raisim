from CAMELController import PDController
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectory1D
import numpy as np
import math

class SimplePendulumPDController(PDController):

    def __init__(self, robot):
        super().__init__(robot)
        self.trajectoryDuration = 6.0
        self.setPDGain(PGain=100.0, DGain=10.0)
        self.updateState()
        self.trajectoryGenerator = ThirdOrderPolynomialTrajectory1D()
        self.trajectoryGenerator.updateTrajectory(self.position, math.pi * 0.25, self.robot.getTime(), self.trajectoryDuration)
        self.setTorqueLimit(20000)

    # override
    def doControl(self):
        self.updateState()
        self.setTrajectory(desiredPosition=self.trajectoryGenerator.getPostionTrajectory(self.robot.getTime()), desiredVelocity=self.trajectoryGenerator.getVelocityTrajectory(self.robot.getTime()))
        self.computeControlInput()
        self.setControlInput()

    # override
    def setTrajectory(self, desiredPosition, desiredVelocity):
        if self.robot.getTime() > self.trajectoryDuration :
            desiredPosition = math.pi * 0.25
            desiredVelocity = 0.0
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
        elif(-self.torqueLimit > self.torque):
            self.robot.setGeneralizedForce(np.array([-self.torqueLimit]))
        else:
            self.robot.setGeneralizedForce(np.array([self.torque]))        

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
