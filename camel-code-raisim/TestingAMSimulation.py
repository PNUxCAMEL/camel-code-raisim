import sys
from CAMELRaisimLib import Simulation, CAMELThread
from PySide6.QtWidgets import QApplication

from TestingAMRobot import TestingAMRobot
from TestingAMDcontroller import TestingAMDconroller

class TestingAMSimulation(Simulation):
    def __init__(self):
        super().__init__()
        self.setDT(0.005)
        self.setSimulationDuration(duration = 5.0)
        self.setFastSimulation(False)
        self.setDataPlot(False)
        self.initializeServer()

        #set robot class
        self.robot = TestingAMRobot(self)

        #set controller
        self.Dcontroller = TestingAMDconroller(self.robot)

        self.setController(self.Dcontroller)
        #set plot

        #set thread
        self.thread = CAMELThread(self)

    def run(self):
        self.thread.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    TestingAMSimulation().run()
    app.exec()
    