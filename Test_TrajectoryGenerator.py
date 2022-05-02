import numpy as np
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectory1D

polynomialTrajectoryGen = ThirdOrderPolynomialTrajectory1D()
currentPosition = 0.0
currentTime = 0.0
for i in range(0,100):
    currentTime += 0.04
    if(i%50 == 0):
        goalPosition = i/10.0  + 1.5
        polynomialTrajectoryGen.updateTrajectory(currentPosition=currentPosition,goalPosition=goalPosition, currentTime=currentTime, timeDuration=2.0)
    currentPosition = polynomialTrajectoryGen.getPostionTrajectory(currentTime)

    print(currentPosition)
    