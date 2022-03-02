import sys
libPath = 'C:/Users/Jaehoon/raisimLib/CAMEL-code/lib'
sys.path.append(libPath)

import Simulation
import CAMELThread
import SimplePendulumRobot
import SimplePendulumPDController

"""
dT       : Discrete time of your system
"""

## new Simulation class
sim = Simulation.Simulation(dT = 0.005)
sim.setSimulationDuration(duration = 10.0)
# sim.setFastSimulation(True)
sim.setFastSimulation(False)

## new Robot class
robot = SimplePendulumRobot.SimplePendulumRobot(sim)

## new Controller class
controller = SimplePendulumPDController.SimplePendulumPDController(robot)
sim.setController(controller)

## new CAMELThread class
simulationThread = CAMELThread.CAMELThread(sim)

# run simulation
simulationThread.start()