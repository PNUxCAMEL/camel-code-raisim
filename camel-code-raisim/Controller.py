from abc import abstractmethod


class Controller:

    def __init__(self, robot):
        self.robot = robot
    
    
    @abstractmethod
    def doControl(self):
        pass

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
    
