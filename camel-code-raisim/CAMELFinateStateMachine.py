from abc import abstractmethod

class TwoState:
    def __init__(self, robot):
        self.robot = robot
        self.state = 0

    def setState(self, state):
        self.state = state

    def setControllers(self, controller1, controller2):
        self.controller1 = controller1
        self.controller2 = controller2

    @abstractmethod
    def checkState(self):
        pass

    @abstractmethod
    def doControl(self):
        pass