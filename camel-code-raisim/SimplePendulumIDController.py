import math
import numpy as np
from CAMELController import InverseDynamicsController
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectory1D

class SimplePendulumIDController(InverseDynamicsController):
    def __init__(self, robot):
        super().__init__(robot)
        self.trajectoryDuration = 5.0
        self.setPDGain(PGain = 200.0, DGain= 20.0)
        self.trajectoryGenerator = ThirdOrderPolynomialTrajectory1D()
        self.trajectoryGenerator.updateTrajectory(currentPosition=self.robot.getQ(), goalPosition= math.pi * 1.0, currentTime= self.robot.getTime(), timeDuration=self.trajectoryDuration)
        self.setTorqueLimit(50)

    def setPDGain(self, PGain, DGain):
        return super().setPDGain(PGain, DGain)
    
    # override
    def doControl(self):
        self.updateState()
        self.setTrajectory(desiredPosition=self.trajectoryGenerator.getPostionTrajectory(self.robot.getTime()), desiredVelocity=self.trajectoryGenerator.getVelocityTrajectory(self.robot.getTime()), desiredAcceleration=self.trajectoryGenerator.getAccelerationTrajectory(self.robot.getTime()))
        self.computeControlInput()
        self.setControlInput()
    
    def setTrajectory(self, desiredPosition, desiredVelocity, desiredAcceleration):
        if self.robot.getTime() > self.trajectoryDuration :
            desiredPosition = math.pi * 1.0
            desiredVelocity = 0.0
            desiredAcceleration = 0.0
        return super().setTrajectory(desiredPosition, desiredVelocity, desiredAcceleration)

    # override
    def updateState(self):
        self.position = self.robot.getQ()
        self.velocity = self.robot.getQD()        
        self.updateMassMatrix()
        self.updateGravityTerm()
    
    # override
    def updateMassMatrix(self):
        self.massMatrix = self.robot.getMass() * self.robot.getWireLength()**2
    
    def updateGravityTerm(self):
        self.gravityTerm = self.robot.getMass() * -9.81 * self.robot.getWireLength() * math.sin(self.position)
    
    # override
    def computeControlInput(self):
        self.positionError = self.desiredPosition - self.position
        self.velocityError = self.desiredVelocity - self.velocity

        # self.torque = self.massMatrix * (self.desiredAcceleration + self.PGain * self.positionError + self.DGain * self.velocityError) - self.gravityTerm
        self.torque = self.massMatrix * self.desiredAcceleration - self.gravityTerm
        # print("torque : ", self.torque)
        

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