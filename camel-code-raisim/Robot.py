from abc import abstractmethod
import numpy as np

class Robot:

    def __init__(self, sim, urdfPath, name):
        self.sim = sim
        self.robot = sim.world.addArticulatedSystem(urdfPath)
        self.robot.setName(name)

    @abstractmethod
    def initialize(self):
        pass

    def setState(self, position, velocity):
        self.robot.setState(position, velocity)

    def getGeneralizedCoordinate(self):
        return self.robot.getGeneralizedCoordinate()

    def getGeneralizedVelocity(self):
        return self.robot.getGeneralizedVelocity()        

    def getContacts(self):
        return self.robot.getContacts()

    def getFrameIdx(self, frameName):
        return self.robot.getFrameIdxByName(frameName)
    
    def getFramePosition(self, frameName):
        return self.robot.getFramePosition(self.getFrameIdx(frameName))

    def getFrameOrientation(self, frameName):
        return self.robot.getFrameOrientation(self.getFrameIdx(frameName))

    def getFrameAngularVelocity(self, frameName):
        return self.robot.getFrameAngularVelocity(self.getFrameIdx(frameName))

    def getBodyNames(self):
        return self.robot.getBodyNames()

    def getFrames(self):
        return self.robot.getFrames()

    def setGeneralizedForce(self, force):
        return self.robot.setGeneralizedForce(force)