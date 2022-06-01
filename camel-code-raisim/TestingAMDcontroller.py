import numpy as np
from CAMELRaisimLib import Controller

class TestingAMDconroller(Controller):
    def __init__(self, robot):
        super().__init__(robot)
        self.DGain = -0.01
        self.kp = 0.1
        self.kd = 0.04
        self.observeVelocity = self.robot.getQD()
        self.motorVelocity = self.robot.getMotorQD()
        self.direction = self.robot.getQ()
        self.setTorqueLimit(3.5)

    def doControl(self):
        self.updateState()
        self.computControlInput()
        self.setControlInput()

    def updateState(self):
        temp = self.observeVelocity
        tempM = self.motorVelocity
        self.observeVelocity = self.robot.getQD()
        self.direction = self.robot.getQ()
        self.observeAcc = (self.observeVelocity - temp)/0.005
        self.motorVelocity = self.robot.getMotorQD()
        self.motorAcc = (self.motorVelocity - tempM)/0.005
    
    def computControlInput(self):
        if (self.observeVelocity * self.observeAcc > 0):
            self.torque = self.DGain * self.observeVelocity
            state = 0
        else:
            self.torque = -self.kp * self.motorVelocity - self.kd * self.motorAcc
            state = 1

        # print(self.observeVelocity)
        print(state)

    def setTorqueLimit(self, torqueLimit):
        self.torqueLimit = torqueLimit
    
    def setControlInput(self):
        if(self.torqueLimit < self.torque):
            self.robot.setGeneralizedForce(np.array([0, self.torqueLimit]))
        elif( -self.torqueLimit > self.torque):
            self.robot.setGeneralizedForce(np.array([0, -self.torqueLimit]))
        else:
            self.robot.setGeneralizedForce(np.array([0, self.torque]))