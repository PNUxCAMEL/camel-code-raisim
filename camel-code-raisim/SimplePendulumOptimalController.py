import math
import numpy as np
from CAMELController import OptimalController
from CAMELOptimizer import GradientDescentSolver
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectory1D

class SimplePendulumOptimalController(OptimalController):
    def __init__(self, robot):
        super().__init__(robot)
        self.setTorqueLimit(50)
        self.desiredPosition = 1.0 * math.pi
        self.desiredVelocity = 0.0

        self.GDSolver = GradientDescentSolver()
        self.GDSolver.setMaximumIteration(1000)
        self.GDSolver.setStepSize(130.0)
        self.GDSolver.setTerminateCondition(1e-5)

        self.trajecotryGenerator = ThirdOrderPolynomialTrajectory1D()
        self.trajecotryGenerator.updateTrajectory(0.0, math.pi, 0.0, 3.1)
        self.desiredNextPosition = self.trajecotryGenerator.getPostionTrajectory(self.robot.getTime() + 0.005)
        self.desiredNextVelocity = self.trajecotryGenerator.getVelocityTrajectory(self.robot.getTime() + 0.005)
        self.initialPoint = np.array([0.0])
        self.PGain = 1
        self.DGain = 2.5
    
    # override
    def doControl(self):
        self.updateState()
        self.updateNextTrajectory()
        self.computeControlInput()
        self.setControlInput()

    # override
    def updateState(self):
        self.position = self.robot.getQ()
        self.velocity = self.robot.getQD()        
    
    def updateNextTrajectory(self):
        self.desiredNextPosition = self.trajecotryGenerator.getPostionTrajectory(self.robot.getTime() + 0.005)
        self.desiredNextVelocity = self.trajecotryGenerator.getVelocityTrajectory(self.robot.getTime() + 0.005)
       
    # def calNextState(self, u):
    #     self.nextPosition = self.position + self.delta * self.velocity
    #     self.nextVelocity = self.velocity + self.delta * (-9.8 * math.sin(self.position)/self.robot.getWireLength + u / (self.robot.getMass * self.robot.getWireLength**2))

    # override
    def computeControlInput(self):
        
        self.GDSolver.setObjectiveFunction(self.objectiveFunction)
        self.GDSolver.setInitialPoint(self.initialPoint)
        self.GDSolver.reset()
        self.GDSolver.solve()

        self.positionError = self.desiredPosition - self.position
        self.velocityError = self.desiredVelocity - self.velocity

        self.torqueff = self.GDSolver.x
        self.torquefb = self.PGain * self.positionError + self.DGain * self.velocityError

        self.torque = self.torqueff
        #self.torque = self.torqueff + self.torquefb
        
        #print("torque : ", self.torque)
        
    def objectiveFunction(self, x):
        # self.calNextState(u)
        self.nextPosition = self.position + 0.005 * self.velocity
        self.nextVelocity = self.velocity + 0.005 * (-9.8 * math.sin(self.position)/self.robot.getWireLength() + x / (self.robot.getMass() * self.robot.getWireLength()**2))

        self.nextPositionError = self.desiredNextPosition - self.nextPosition
        self.nextVelocityError = self.desiredNextVelocity - self.nextVelocity

        return self.nextPositionError**2 + self.nextVelocityError**2

    # override
    def setControlInput(self):
        if (self.torqueLimit < self.torque):
            self.robot.setGeneralizedForce(np.array([self.torqueLimit]))
        elif(-self.torqueLimit > self.torque):
            self.robot.setGeneralizedForce(np.array([-self.torqueLimit]))
        else:
            self.robot.setGeneralizedForce(np.array([self.torque]))        

    def setTorqueLimit(self, torqueLimit):
        self.torqueLimit = torqueLimit

    def getPosition(self):
        return self.position

    def getVelocity(self):
        return self.velocity

    def getDesiredPosition(self):
        return self.desiredNextPosition
    
    def getDesiredVelocity(self):
        return self.desiredNextVelocity
    