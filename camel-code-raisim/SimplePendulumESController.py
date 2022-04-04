import math
import numpy as np
from CAMELController import EnergyShapingController
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectory1D

class SimplePendulumESController(EnergyShapingController):
    def __init__(self, robot):
        super().__init__(robot)
        self.setPGain(PGain = 2.0)
        self.desiredPosition = 1.0 * math.pi
        self.setTorqueLimit(5)
        print(self.robot.getBodyIdx("wire"))

        self.iteration = 0

    def setPGain(self, PGain):
        return super().setPGain(PGain)
    
    # override
    def doControl(self):
        self.updateState()
        self.setTrajectory(self.desiredPosition)
        self.computeControlInput()
        self.setControlInput()
        if(self.iteration == 10):
            self.robot.setExternalForce(1,np.array([0.0, 0.0, -0.575]),np.array([10.0, 10.0, 10.0]))
        else:
            self.robot.setExternalForce(1,np.array([0.0, 0.0, -0.575]),np.array([0.0, 0.0, 0.0]))
        
        self.iteration += 1
    
    def setTrajectory(self, desiredPosition):
        desiredEnergy = -1 * self.robot.getMass() * 9.8 * self.robot.getWireLength() * math.cos(desiredPosition)
        return super().setTrajectory(desiredEnergy)

    # override
    def updateState(self):
        self.position = self.robot.getQ()
        self.velocity = self.robot.getQD()        
        self.updateEnergy()
    
    # override
    def updateEnergy(self):
        self.energy = self.robot.getMass() * self.robot.getWireLength()**2 * self.velocity**2 / 2 - self.robot.getMass() * 9.8 * self.robot.getWireLength() * math.cos(self.position)
    
    # override
    def computeControlInput(self):
        self.energyError = self.energy - self.desiredEnergy
        self.torque = -1 * self.PGain * self.velocity * self.energyError + 3.0 * self.velocity
        print("torque : ", self.torque)
        

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
    
    # #for plot
    def getEnergyError(self):
        return self.energyError

    def getDEnergyError(self):
        if (self.torqueLimit < self.torque):
            self.DEnergyError = self.torqueLimit * self.velocity
        elif(-self.torqueLimit > self.torque):
            self.DEnergyError = -1 * self.torqueLimit * self.velocity
        else:
            self.DEnergyError = self.torque * self.velocity
        return self.DEnergyError