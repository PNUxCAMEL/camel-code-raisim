import math
import os
import numpy as np
import sys
sys.path.append('C:/Users/camelcha/raisimLib/install/bin') # path to the raisimpy
import raisimpy as raisim
import time
raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
### basic lines for initializing raisim

# make raisim world
world = raisim.World()
world.setTimeStep(0.001)
server = raisim.RaisimServer(world)
ground = world.addGround()

# load robot.urdf file
planarElbow_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/rsc/test_planar_elbow_2dof.urdf"
planarElbow = world.addArticulatedSystem(planarElbow_urdf_file)   # robot class
planarElbow.setName("planarElbow")
planarElbow.setGeneralizedCoordinate(np.array([0, 0]))

# lauch server
server.launchServer(8080)

torque = np.zeros(2)
desiredPosition = np.array([math.pi/2,0])
desiredVelocity = np.array([0,0])
Kp = 1000.0
Kd = 50.0

obj = world.addCylinder(0.2, 0.3, 9999)
obj.setPosition(0.0, 0.4, 0.2)

time.sleep(2)
#integrate() = integrate1() + integrate2()
world.integrate1()

def delay(delayTime):   # should be changed later.
      startTime = time.time_ns()
      while(True):
         currentTime = time.time_ns()
         if((currentTime - startTime)*1e-9 > delayTime):
            break
i=0
while(True):
    delay(0.01)

    position = planarElbow.getGeneralizedCoordinate()
    velocity = planarElbow.getGeneralizedVelocity()
    tempTorque = Kp * (desiredPosition - position) + Kd * (desiredVelocity - velocity)
    torque = tempTorque
    planarElbow.setGeneralizedForce(torque)

    
    # print("position :", position)
    # print("velocity :", velocity)
    world.integrate()

