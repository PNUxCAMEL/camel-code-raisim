from CAMELRaisimLib import Plot
import numpy as np

class SimplePendulumPlot(Plot):
    def __init__(self, simulation):
        super().__init__(simulation)
        self.totalDataNum = 4
        self.subPlotNum = 2
        self.data1 = np.array([])
        self.data2 = np.array([])
        self.data3 = np.array([])
        self.data4 = np.array([])
        self.t = np.array([])

    # override
    def setData(self):
        self.t = np.append(self.t, [self.sim.getTime()])
        self.data1 = np.append(self.data1, [self.controller.getDesiredPosition()])
        self.data2 = np.append(self.data2, [self.controller.getPosition()])
        self.data3 = np.append(self.data3, [self.controller.getDesiredVelocity()])
        self.data4 = np.append(self.data4, [self.controller.getVelocity()])
