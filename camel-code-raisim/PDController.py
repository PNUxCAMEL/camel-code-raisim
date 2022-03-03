from abc import abstractmethod

import Controller

class PDController(Controller.Controller):

    def __init__(self, robot):
        super().__init__(robot)
        self.positionError = 0
        self.differentialError = 0
        
    def setPDGain(self, PGain, DGain):
        self.PGain = PGain
        self.DGain = DGain
    
    def doControl(self):
        return super().doControl()

    def setTrajectory(self):
        return super().setTrajectory()

    def updateState(self):
        return super().updateState()

    def computeControlInput(self):
        return super().computeControlInput()

    def setControlInput(self):
        return super().setControlInput()
