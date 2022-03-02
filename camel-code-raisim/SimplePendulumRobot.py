import sys
libPath = 'C:/Users/Jaehoon/raisimLib/CAMEL-code/lib'
sys.path.append(libPath)

import Robot
import numpy as np
import math

"""
path of the robot's urdf file and name of the robot should be set by user.
urdfPath : Path and name of 'robot.urdf' file.
name     : Name of robot

[ frames]    0: "fixed" 1: "top_pitch"
[ bodies]    0: "base" 1: "wire"
[Generalized Coordinate]  0: "linear_guide" 1: "hip_pitch" 2: "knee_pitch"
"""
class SimplePendulumRobot(Robot.Robot):

    def __init__(self, sim):
        urdfPath = "C:/Users/Jaehoon/raisimLib/CAMEL-code/rsc/camel_simple_pendulum.urdf"
        name = 'cutePendulum'
        super().__init__(sim, urdfPath, name)
        self.initialize()

    # override
    def initialize(self):
        initialPosition = np.array([math.pi * 0.25])
        initialVelocity = np.array([0.0])
        self.setState(initialPosition, initialVelocity)
        
    def getQ(self):
        return self.getGeneralizedCoordinate()[0]   # hip joint angular position , 0: top_pitch

    def getQD(self):
        return self.getGeneralizedVelocity()[0]   # knee joint angular position