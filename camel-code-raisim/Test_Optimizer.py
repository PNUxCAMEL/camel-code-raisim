from CAMELOptimizer import GradientDescentSolver
import numpy as np

def objectiveFunction(x):
    return (x[0]-5.0)**2 + (x[1]-2.0)**2


initialPoint = np.array([0.0, 0.0])
GDSolver = GradientDescentSolver()
GDSolver.setObjectiveFunction(objectiveFunction)
GDSolver.setInitialPoint(initialPoint)
GDSolver.solve()

print(GDSolver.x)
