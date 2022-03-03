import sys
libPath = 'C:/Users/Jaehoon/raisimLib/CAMEL-code/lib'
sys.path.append(libPath)

import Simulation
import CAMELThread
import SimplePendulumRobot
import SimplePendulumPDController
import SimplePendulumPIDController
from PySide6.QtWidgets import QApplication
"""
dT       : Discrete time of your system
"""

## new Simulation class
app = QApplication(sys.argv)

sim = Simulation.Simulation()
sim.setDT(0.005)
sim.setSimulationDuration(duration = 100.0)
sim.setFastSimulation(True)
# sim.setFastSimulation(False)
sim.initializeServer()

## new Robot class
robot = SimplePendulumRobot.SimplePendulumRobot(sim)

## new Controller class
controller = SimplePendulumPDController.SimplePendulumPDController(robot)
# controller = SimplePendulumPIDController.SimplePendulumPIDController(robot)
sim.setController(controller)

## new CAMELThread class
simulationThread = CAMELThread.CAMELThread(sim)

# run simulation
simulationThread.start()

app.exec()