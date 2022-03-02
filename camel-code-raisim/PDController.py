from abc import abstractmethod


class PDController:

    def __init__(self, robot):
        self.robot = robot
        
    def setPDGain(self, PGain, DGain):
        self.PGain = PGain
        self.DGain = DGain
    
    @abstractmethod
    def setTrajectory(self):
        pass

    @abstractmethod
    def updateState(self):
        pass

    @abstractmethod
    def computeControlInput(self):
        pass

    @abstractmethod
    def setControlInput(self):
        pass
    
