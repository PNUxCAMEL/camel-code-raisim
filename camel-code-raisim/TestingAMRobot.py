from CAMELRaisimLib import Robot
import numpy as np
import math
import os

"""
path of the robot's urdf file and name of the robot should be set by user.
urdfPath    : Path and name of 'robot.urdf' file
name        : Name of robot

[Generalized Cordinate] 0: "observ_baring"  1: "dumbball_motor"
"""
class TestingAMRobot(Robot):
    def __init__(self, sim):
        urdfPath = os.path.dirname(os.path.realpath(__file__)) + "/rsc/camel_testingAM.urdf"
        name = 'testingAM'
        super().__init__(sim, urdfPath, name)
        print(self.getGeneralizedCoordinate())
        self.initialize()
        print(self.getGeneralizedCoordinate())

    #override
    def initialize(self):
        initialPosition = np.array([math.pi * 0.5, math.pi * 0.0])
        initialVelocity = np.array([0.0, 0.0])
        self.setState(initialPosition, initialVelocity)

    def getQ(self):
        return self.getGeneralizedCoordinate()[0]
    
    def getQD(self):
        return self.getGeneralizedVelocity()[0]

    def getMotorQD(self):
        return self.getGeneralizedVelocity()[1]