import math
import numpy as np

class functionClass:
    def __init__(self, MPChorizon, initialPos, initialVel, desiredTrajectorySequence):
        self.trajectory = desiredTrajectorySequence
        self.dT = 0.005
        self.m = 5.0
        self.l = 0.575
        self.I = self.m * self.l**2
        self.g = -9.81
        self.weightingMat = np.array([[15.0, 0.0],[0.0, 1.0]])
        
        self.MPChorizon = MPChorizon
        self.x = np.zeros((2,self.MPChorizon))
        self.x[0,0] = initialPos
        self.x[1,0] = initialVel
        self.tau = np.ones((1,self.MPChorizon))*5.0
        
        self.A = np.array([[1.0, self.dT],[0, 1.0]])
        self.b = np.array([[0],[self.dT/self.I]])
        self.c = np.zeros((2,self.MPChorizon))
        for i in range(0,self.MPChorizon):
            self.c[1,i] = self.dT/self.I*self.m*self.l*self.g*math.sin(self.x[0,i])
        
        self.stepSize = 1.0
        self.maximumIteration = 200
        self.terminateCondition = 1e-3
        self.delta = 1e-8
        self.iteration = 0
        self.terminateFlag = False
        self.gradient = np.zeros(self.MPChorizon)



    def objectiveFunction(self, tau):
        tempFunctionValue = 0.0
        for i in range(0, self.MPChorizon):
            tempA = (self.A.dot(self.x[:,i]) + self.b.dot(tau[:,i]) + self.c[:,i] - self.trajectory[:,i])
            tempFunctionValue += tempA.T.dot(self.weightingMat).dot(tempA)
        return tempFunctionValue

    def updateState(self):
        self.x1 = self.A.dot(self.x.copy()) + self.b.dot(self.tau) + self.c
        for i in range(0,self.MPChorizon-1):
            self.x[:,i+1] = self.x1[:,i]
        for i in range(0,self.MPChorizon):
            self.c[1,i] = self.dT/self.I*self.m*self.l*self.g*math.sin(self.x[0,i])

    def computeGradient(self): 
               
        for i in range(self.MPChorizon):
            temp_tau = self.tau.copy()
            temp_tau[:,i] += self.delta 
            self.gradient[i] = (self.objectiveFunction(temp_tau) - self.objectiveFunction(self.tau)) / self.delta
        self.RMSgradient = (self.gradient.dot(self.gradient) / self.MPChorizon) ** (1/2)

    def updateVariables(self):
        self.tau = self.tau.copy() - self.stepSize * self.gradient

    def checkTerminateCondition(self):
        if(self.iteration == self.maximumIteration):
            self.terminateReason = "maximum iteration"
            self.terminateFlag = True
        elif(self.RMSgradient < self.terminateCondition):
            self.terminateReason = "terminate conditon"
            self.terminateFlag = True

    def solve(self):
        for i in range(self.maximumIteration):
            self.iteration += 1
            self.computeGradient()
            self.updateVariables()
            self.updateState()
            self.checkTerminateCondition()
            if(self.terminateFlag):
                print(self.terminateReason)
                break



desiredTrajectory = np.zeros((2,1000))
temp1 = np.arange(start=0,stop=2,step=0.002)
temp2 = np.ones(np.size(temp1))*0.4
desiredTrajectory[0,:] = temp1
desiredTrajectory[1,:] = temp2


currentPos = 0.0
currentVel = 0.0
MPChorizon = 10
for i in range(0, 900):
    desiredTrajectorySequence = desiredTrajectory[:,i+1:i+MPChorizon+1]
    MPCsolver = functionClass(MPChorizon, initialPos=currentPos, initialVel=currentVel, desiredTrajectorySequence=desiredTrajectorySequence)
    MPCsolver.solve()
    print(MPCsolver.iteration)
    print(MPCsolver.x)
    print(desiredTrajectorySequence)
    print(MPCsolver.tau)
    currentPos = MPCsolver.x[0,1]
    currentVel = MPCsolver.x[1,1]