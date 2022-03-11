import numpy as np
from CAMELOptimizer import GradientDescent

def myFunction1(x):
    return (x[0] - 2.0)**2 + x[1]**2    # f(x) = (x1 - 2)^2 + x2^2
    
def myFunction2(x):
    return (x[0] - 2.5)**2 + (x[1] - 5.2)**2


GDSolver = GradientDescent()

GDSolver.setObjectiveFunction(objectiveFunction=myFunction2)
initialPoint = np.array([0.0, 0.0])
GDSolver.setInitialPoint(initialPoint)
GDSolver.solve()
print(GDSolver.iteration)
print(GDSolver.x)