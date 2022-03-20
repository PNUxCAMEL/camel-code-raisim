import math
import numpy as np
from CAMELOptimizer import GradientDescentSolver
from pytictoc import TicToc

"""
Difference btw/ existed MPC solver (analytic ver.) and this MPC solver (numerical ver.)

1) I divided calculated gradient by dT^2 in existed MPC(because of the small gradient).
 => So, I have to multiply dT^2 in forcePenalizeWeighting
    Also, Step size should be divided by dT^2 

"""
class functionClass:
    def __init__(self, MPChorizon, initialPos, initialVel, desiredTrajectorySequence):
        self.MPChorizon = MPChorizon
        self.initialPos = initialPos
        self.initialVel = initialVel
        self.trajectory = desiredTrajectorySequence
        self.dT = 0.005
        self.m = 5.0
        self.g = 9.81
        self.forceLimit = 200.0
        
        self.A = np.array([[1.0, self.dT],[0, 1.0]])
        self.b = np.array([[0],[self.dT/self.m]])
        self.c = np.zeros((2,self.MPChorizon))
        for i in range(0,self.MPChorizon):
            self.c[1,i] = -self.dT/self.g

        self.weightingMat = np.array([[1.5, 0.0],[0.0, 0.001]])
        self.forcePenalizeWeighting = 1e-3 * self.dT * self.dT
        self.next_x = np.zeros((2,self.MPChorizon))
        self.next_x_temp = np.zeros((2,self.MPChorizon))
        self.force = np.ones(self.MPChorizon)*self.m * self.g
        
        self.stepSize = 0.08 / self.dT / self.dT
        self.maximumIteration = 200
        self.terminateCondition = 1e-6
        self.delta = 1e-3
        self.iteration = 0
        self.terminateFlag = False
        self.gradient = np.zeros(self.MPChorizon)
        self.solver = GradientDescentSolver()

    def solve(self):
        for i in range(0, self.maximumIteration):
            self.iteration += 1
            self.updateState()
            self.computeGradient()
            self.updateVariables()
            self.checkTerminateCondition()
            if(self.terminateFlag):
                print(self.terminateReason)
                break

    def updateStateTemp(self, force):
        for i in range(0, self.MPChorizon):
            temp = 0.0
            if(i==0):
                self.next_x_temp[0,i] = self.initialPos + self.dT * self.initialVel
                self.next_x_temp[1,i] = self.initialVel + self.dT * (force[i] / self.m - self.g)
            elif(i==1):
                self.next_x_temp[0,i] = self.next_x_temp[0, i-1] + self.dT * self.dT * (force[i-1] / self.m - self.g) + self.dT * self.initialVel
                self.next_x_temp[1,i] = self.next_x_temp[1, i-1] + self.dT * (force[i] / self.m - self.g)
            else:
                self.next_x_temp[0,i] = self.next_x_temp[0, i-1] + self.dT * self.dT * (force[i-1] / self.m - self.g) + self.dT * self.next_x_temp[1, i-2]
                self.next_x_temp[1,i] = self.next_x_temp[1, i-1] + self.dT * (force[i] / self.m - self.g)

    def updateState(self):
        for i in range(0, self.MPChorizon):
            if(i==0):
                self.next_x[0,i] = self.initialPos + self.dT * self.initialVel
                self.next_x[1,i] = self.initialVel + self.dT * (self.force[i] / self.m - self.g)
            elif(i==1):
                self.next_x[0,i] = self.next_x[0, i-1] + self.dT * self.dT * (self.force[i-1] / self.m - self.g) + self.dT * self.initialVel
                self.next_x[1,i] = self.next_x[1, i-1] + self.dT * (self.force[i] / self.m - self.g)
            else:
                self.next_x[0,i] = self.next_x[0, i-1] + self.dT * self.dT * (self.force[i-1] / self.m - self.g) + self.dT * self.next_x[1, i-2]
                self.next_x[1,i] = self.next_x[1, i-1] + self.dT * (self.force[i] / self.m - self.g)

    def objectiveFunction(self, force):
        self.updateStateTemp(force)
        ret = 0.0
        for i in range(0, self.MPChorizon):
            ret += (self.next_x_temp[:,i] - self.trajectory[:,i]).T.dot(self.weightingMat).dot(self.next_x_temp[:,i] - self.trajectory[:,i]) + self.forcePenalizeWeighting*force[i]*force[i]
        return ret

    def computeGradient(self):
        # TODO: Errors are in gradient calculator
        functionValue = self.objectiveFunction(self.force)
        for i in range(self.MPChorizon):
            temp_force = self.force.copy()
            temp_force[i] += self.delta 
            self.gradient[i] = (self.objectiveFunction(temp_force) - functionValue) / self.delta
        self.RMSgradient = (self.gradient.dot(self.gradient) / self.MPChorizon) ** (1/2)

    def updateVariables(self):
        for i in range(0, self.MPChorizon):
            tempForce = self.force[i] - self.stepSize * self.gradient[i]
            if(tempForce > self.forceLimit):
                self.force[i] = self.forceLimit
            elif(tempForce < -self.forceLimit):
                self.force[i] = -self.forceLimit
            else:
                self.force[i] = tempForce
    
    def checkTerminateCondition(self):
        if(self.iteration == self.maximumIteration):
            self.terminateReason = "maximum iteration"
            self.terminateFlag = True
        elif(self.RMSgradient < self.terminateCondition):
            self.terminateReason = "terminate conditon"
            self.terminateFlag = True

desiredTrajectory = np.zeros((2,1000))
positionTrajectory = np.arange(start=0,stop=2,step=0.002)
velocityTrajectory = np.ones(np.size(positionTrajectory))*0.4
desiredTrajectory[0,:] = positionTrajectory
desiredTrajectory[1,:] = velocityTrajectory
tictoc = TicToc()

currentPos = 0.0
currentVel = 0.0
MPChorizon = 10

for i in range(0, 900):
    desiredTrajectorySequence = desiredTrajectory[:,i+1:i+MPChorizon+1]
    MPCsolver = functionClass(MPChorizon, initialPos=currentPos, initialVel=currentVel, desiredTrajectorySequence=desiredTrajectorySequence)
    tictoc.tic()
    MPCsolver.solve()
    tictoc.toc()
    
    print(MPCsolver.iteration)
    print(MPCsolver.RMSgradient)
    print(MPCsolver.next_x)
    print(desiredTrajectorySequence)
    print(MPCsolver.force)
    currentPos = MPCsolver.next_x[0,0]
    currentVel = MPCsolver.next_x[1,0]