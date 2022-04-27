from CAMELOptimizer import GradientDescentSolver
import numpy as np


c = 0
def objectiveFunction(x):
    return (x[0]-5.0)**2 + (x[1]-2.0)**2 + c


initialPoint = np.array([0.0, 0.0])
print(objectiveFunction(initialPoint))

c = 10
print(objectiveFunction(initialPoint))
# GDSolver = GradientDescentSolver()
# GDSolver.setObjectiveFunction(objectiveFunction)
# GDSolver.setInitialPoint(initialPoint)
# GDSolver.solve()

# print(GDSolver.x)
