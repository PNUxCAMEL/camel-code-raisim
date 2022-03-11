import math
import numpy as np
from CAMELController import InverseDynamicsController

class SimplePendulumIDController(InverseDynamicsController):
    def __init__(self, robot):
        super().__init__(robot)
        self.setPDGain(PGain = 200.0, DGain= 25.0)
        self.setTrajectory(desiredPosition=1.57, desiredVelocity=0.0, desiredAcceleration=0.0)
        self.setTorqueLimit(50)

    def setPDGain(self, PGain, DGain):
        return super().setPDGain(PGain, DGain)
    
    def doControl(self):
        self.updateState()
        self.computeControlInput()
        self.setControlInput()
    
    def setTrajectory(self, desiredPosition, desiredVelocity, desiredAcceleration):
        return super().setTrajectory(desiredPosition, desiredVelocity, desiredAcceleration)

    def updateState(self):
        self.position = self.robot.getQ()
        self.velocity = self.robot.getQD()        
        self.updateMassMatrix()
        self.updateGravityTerm()
    
    def updateMassMatrix(self):
        self.massMatrix = self.robot.getMass() * self.robot.getWireLength()**2
    
    def updateGravityTerm(self):
        self.gravityTerm = self.robot.getMass() * -9.81 * self.robot.getWireLength() * math.sin(self.position)
    
    def computeControlInput(self):
        self.positionError = self.desiredPosition - self.position
        self.velocityError = self.desiredVelocity - self.velocity
        self.torque = self.massMatrix * (self.desiredAcceleration + self.PGain * self.positionError + self.DGain * self.velocityError) + self.gravityTerm

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
    