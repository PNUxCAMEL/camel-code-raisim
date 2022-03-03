import sys

# TODO : make UI and play button which works with simulaitonDuration

"""
path of bin and license should be set by user. 
"""

binPath = 'C:/Users/Jaehoon/raisimLib/install/bin'
licensePath = "C:/Users/Jaehoon/.raisim/raisim.activation"
sys.path.append(binPath)
import raisimpy as raisim
raisim.World.setLicenseFile(licensePath)

class Simulation:

    def __init__(self):
        self.world = raisim.World()
        self.server = raisim.RaisimServer(self.world)
        self.ground = self.world.addGround()

    def integrate(self):
        self.world.integrate()

    def integration1(self):
        self.world.integrate1()

    def integration2(self):
        self.world.integrate2()

    def initializeServer(self):
        self.server.launchServer(8080)

    def killServer(self):
        self.server.killServer()

    def setRobot(self, robot):
        self.robot = robot

    def setController(self, controller):
        self.controller = controller

    def setPlot(self, plot):
        self.plot = plot

    def setDT(self, dT):
        self.world.setTimeStep(dT)

    def setSimulationDuration(self, duration):
        self.simulationDuration = duration

    def setFastSimulation(self, booleanValue):
        self.isFastSimulation = booleanValue

    def setDataPlot(self, booleanValue):
        self.isDataPlot = booleanValue

    def getRobot(self):
        return self.robot

    def getController(self):
        return self.controller

    def getPlot(self):
        return self.plot

    def getDT(self):
        return self.world.getTimeStep()

    def getSimulationDuration(self):
        return self.simulationDuration

    def getTime(self):
        return self.world.getWorldTime()
