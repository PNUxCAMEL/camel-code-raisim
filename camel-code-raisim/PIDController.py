import PDController

class PIDController(PDController.PDController):
    def __init__(self, robot):
        super().__init__(robot)
        self.cumulativeError = 0
    
    def setPDGain(self, PGain, DGain):
        return super().setPDGain(PGain, DGain)
    
    def setPIDGain(self, PGain, IGain, DGain):
        self.setPDGain(PGain, DGain)
        self.IGain = IGain

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
    