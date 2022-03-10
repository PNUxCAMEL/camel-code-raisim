import numpy as np
from CAMELOptimizer import GradientDescent

def myFunction(A):
    return (A[0] - 2.0)**2 + A[1]**2
    


GDSolver = GradientDescent()

GDSolver.setObjectiveFunction(objectiveFunction=myFunction)
initialPoint = np.array([0.0, 0.0])
GDSolver.setInitialPoint(initialPoint)
GDSolver.solve()
print(GDSolver.x)