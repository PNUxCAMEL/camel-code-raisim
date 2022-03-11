import numpy as np

class ThirdOrderPolynomialTrajectory1D:
    """
    3rd-order polynomial trajectory generator in 1dimensional space. (Time normalized in [0, 1])
    """
    def __init__(self):
        self.A = np.array([[2.0,-2.0,1.0,1.0],[-3.0,3.0,-2.0,-1.0],[0.0,0.0,1.0,0.0],[1.0,0.0,0.0,0.0]])
        self.coefficient = np.zeros(4)
        self.referenceTime = 0.0
        self.timeDuration = 1.0

    def updateTrajectory(self, currentPosition, goalPosition, currentTime, timeDuration):
        self.functionValue = np.array([currentPosition, goalPosition, 0.0, 0.0])
        self.referenceTime = currentTime
        self.timeDuration = timeDuration
        self.calculateCoefficient()

    def calculateCoefficient(self):
        self.coefficient = self.A.dot(self.functionValue)

    def getPostionTrajectory(self, currentTime):
        normalizedTime = (currentTime - self.referenceTime) / self.timeDuration
        return self.coefficient[0] * normalizedTime**3 + self.coefficient[1] * normalizedTime**2 + self.coefficient[2] * normalizedTime + self.coefficient[3]

    def getVelocityTrajectory(self, currentTime):
        normalizedTime = (currentTime - self.referenceTime) / self.timeDuration
        return 3 * self.coefficient[0] * normalizedTime**2 + 2 * self.coefficient[1] * normalizedTime + self.coefficient[2]

    def getAccelerationTrajectory(self, currentTime):
        normalizedTime = (currentTime - self.referenceTime) / self.timeDuration
        return 6 * self.coefficient[0] * normalizedTime + 2 * self.coefficient[1]
