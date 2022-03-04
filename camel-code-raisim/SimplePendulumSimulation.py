import sys
libPath = 'C:/Users/Jaehoon/raisimLib/CAMEL-code/lib'
sys.path.append(libPath)

import Simulation
import CAMELThread
import SimplePendulumRobot
import SimplePendulumPDController
import SimplePendulumPIDController
import SimplePendulumPlot
from PySide6.QtWidgets import QApplication
"""
dT       : Discrete time of your system
"""

app = QApplication(sys.argv)

## new Simulation class
sim = Simulation.Simulation()
sim.setDT(0.005)
sim.setSimulationDuration(duration = 1.0)
sim.setFastSimulation(False)
sim.setDataPlot(True)
sim.initializeServer()

## set Robot class
robot = SimplePendulumRobot.SimplePendulumRobot(sim)

## set Controller class
# controller = SimplePendulumPDController.SimplePendulumPDController(robot)
controller = SimplePendulumPIDController.SimplePendulumPIDController(robot)
sim.setController(controller)

## set Plot class
plot = SimplePendulumPlot.SimplePendulumPlot(sim)
sim.setPlot(plot)

## new CAMELThread class
simulationThread = CAMELThread.CAMELThread(sim)

# run simulation
simulationThread.start()

app.exec()