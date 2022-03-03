import Controller

class MPCController(Controller.Controller):
    def __init__(self, robot):
        super().__init__(robot)
        
    def doControl(self):
        return super().doControl()
    
    def setTrajectory(self, trajectorySequence):
        self.trajectorySequence = trajectorySequence
    
    def updateState(self):
        return super().updateState()
    
    def computeControlInput(self):
        return super().computeControlInput()

    def setControlInput(self):
        return super().setControlInput()
    
    def setMPCHorizon(self, MPCHorizon):
        self.MPCHorizon = MPCHorizon

    def setSolver(self, solver):
        self.solver = solver
