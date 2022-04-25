import sys
from CAMELRaisimLib import Simulation, CAMELThread
from PySide6.QtWidgets import QApplication
import math

from SimplePendulumRobot import SimplePendulumRobot
from SimplePendulumPDController_UnitTrajectory import SimplePendulumPDController_UnitTrajectory
from SimplePendulumPDController import SimplePendulumPDController
from SimplePendulumPIDController import SimplePendulumPIDController
from SimplePendulumIDController import SimplePendulumIDController
from SimplePendulumESController import SimplePendulumESController
from SimplePendulumSW import SimplePendulumSW
from SimplePendulumPlot import SimplePendulumPlot

class SimplePendulumSimulation(Simulation):
    def __init__(self):
        super().__init__()
        self.setDT(0.005)
        self.setSimulationDuration(duration = 6.0)
        self.setFastSimulation(False)
        self.setDataPlot(True)
        self.initializeServer()

        # set robot class
        self.robot = SimplePendulumRobot(self)
        
        # set controller 
        self.PDcontroller_U = SimplePendulumPDController_UnitTrajectory(self.robot)
        self.PDcontroller_U.setPDGain(700,200)
        self.PDcontroller = SimplePendulumPDController(self.robot)
        self.PDcontroller.setPDGain(700,200)
        self.PIDcontroller = SimplePendulumPIDController(self.robot)
        self.PIDcontroller.setPIDGain(200,1,20)
        self.IDcontroller = SimplePendulumIDController(self.robot)
        self.EScontroller = SimplePendulumESController(self.robot)
        self.SWcontroller = SimplePendulumSW(self.robot)

        self.setController(self.SWcontroller)
        
        # set plot
        self.plot = SimplePendulumPlot(self)
        self.setPlot(self.plot)

        # set thread
        self.thread = CAMELThread(self)

    def run(self):
        self.thread.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    SimplePendulumSimulation().run()
    app.exec()
