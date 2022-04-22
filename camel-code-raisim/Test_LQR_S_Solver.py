from CAMELOptimizer import GradientDescentSolver
import numpy as np

def costFunction(x):
    g= 9.8
    l = 0.085
    lb = 0.075
    mb = 0.419
    mw = 0.204
    Ib = 3.34 * 10**(-3)
    Iw = 0.57 * 10**(-3)
    Cb = 1.02 * 10**(-3)
    Cw = 0.05 * 10**(-3)

    A = np.array([[0., 1.0, 0.0], [(mb*lb + mw*l)*g/(Ib + mw*l**2), -Cb/(Ib + mw*l**2), -(mb*lb + mw*l)*g/(Ib+mw*l**2)], [-(mb*lb + mw*l)*g/(Ib+mw*l**2), Cb/(Ib + mw*l**2), -Cw*(Ib+Iw+mw*l**2)/Iw*(Ib+mw*l**2)]])
    B = np.array([[0],[-1/(Ib + mw*l**2)],[(Ib+Iw+mw*l**2)/(Iw*(Ib+mw*l**2))]])

    Q = np.array([[g/lb, 0, 0], [0, 1, 0], [0, 0, 1]])
    R = 1
    S = np.array([[x[0], x[1], x[2]], [x[3], x[4], x[5]], [x[6], x[7], x[8]]])

    term1 = S.dot(B)*R
    term1 = term1.dot(B.T)
    term1 = term1.dot(S)
    Eqn = Q - term1 + S.dot(A) + A.T.dot(S)
    Eqn_ = Eqn.dot(Eqn.T)
    cost = Eqn_[0,0] + Eqn_[1,1] + Eqn_[2,2]

    # cost = (- B[1,0]**2*S[0,1]**2 - 2*B[1,0]*B[2,0]*S[0,1]*S[0,2] - B[2,0]**2*S[0,2]**2 + 2*A[1,0]*R*S[0,1] + 2*A[2,1]*R*S[0,2] + Q[0,0]*R)**2/R**2 + \
    #     (- B[1,0]**2*S[1,2]**2 - 2*B[1,0]*B[2,0]*S[1,2]*S[2,2] - B[2,0]**2*S[2,2]**2 + 2*A[1,2]*R*S[1,2] + 2*A[2,2]*R*S[2,2] + Q[2,2]*R)**2/R**2 + \
    #         (2*(B[1,0]**2*S[0,1]*S[1,2] + B[2,0]**2*S[0,2]*S[2,2] - A[1,2]*R*S[0,1] - A[1,0]*R*S[1,2] - A[2,2]*R*S[0,2] - A[2,1]*R*S[2,2] + B[1,0]*B[2,0]*S[0,2]*S[1,2] + B[1,0]*B[2,0]*S[0,1]*S[2,2])**2)/R**2 + \
    #             (- B[1,0]**2*S[1,1]**2 - 2*B[1,0]*B[2,0]*S[1,1]*S[1,2] - B[2,0]**2*S[1,2]**2 + 2*A[1,1]*R*S[1,1] + 2*A[2,1]*R*S[1,2] + Q[1,1]*R + 2*R*S[0,1])**2/R**2 + \
    #                 (2*(R*S[0,2] - B[1,0]**2*S[1,1]*S[1,2] - B[2,0]**2*S[1,2]*S[2,2] + A[1,1]*R*S[1,2] + A[1,2]*R*S[1,1] + A[2,2]*R*S[1,2] + A[2,1]*R*S[2,2] - B[1,0]*B[2,0]*S[1,2]**2 - B[1,0]*B[2,0]*S[1,1]*S[2,2])**2)/R**2 + \
    #                     (2*(R*S[0,0] - B[1,0]**2*S[0,1]*S[1,1] - B[2,0]**2*S[0,2]*S[1,2] + A[1,1]*R*S[0,1] + A[1,0]*R*S[1,1] + A[2,1]*R*S[0,2] + A[2,1]*R*S[1,2] - B[1,0]*B[2,0]*S[0,1]*S[1,2] - B[1,0]*B[2,0]*S[0,2]*S[1,1])**2)/R**2
    print(cost)
    return cost


initialPoint = np.array([1., 1., 1., 1., 1., 1., 1., 1., 1.])
GDSolver = GradientDescentSolver()
GDSolver.setObjectiveFunction(costFunction)
GDSolver.setInitialPoint(initialPoint)
GDSolver.solve()

print(GDSolver.x)
