import math
import numpy as np
from CAMELFinateStateMachine import TwoState

from SimplePendulumESController import SimplePendulumESController
from SimplePendulumPDController_UnitTrajectory import SimplePendulumPDController_UnitTrajectory


class SimplePendulumSW(TwoState):
    def __init__(self, robot):
        super().__init__(robot)
        super().setControllers(SimplePendulumESController(self.robot), SimplePendulumPDController_UnitTrajectory(self.robot))
        self.controller2.setPDGain(200,50)

    def checkState(self):
        if((self.robot.getQ() > 0.9 * math.pi or self.robot.getQ() < -0.9 * math.pi) and self.state == 0):
            self.state = 1
        elif((self.robot.getQ() > 0.1 * math.pi or self.robot.getQ() < -0.1 * math.pi) and self.state == 1):
            self.state = 0

    def doControl(self):
        self.checkState()
        if(self.state == 0):
            self.controller1.doControl()
        else:
            self.controller2.doControl()

    def getPosition(self):
        return self.robot.getQ()

    def getVelocity(self):
        return self.robot.getQD()

    def getDesiredPosition(self):
        return 1.0*math.pi
    
    def getDesiredVelocity(self):
        return 0.0