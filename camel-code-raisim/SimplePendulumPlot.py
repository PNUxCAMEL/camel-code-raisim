from CAMELRaisimLib import Plot
import numpy as np

class SimplePendulumPlot(Plot):
    def __init__(self, simulation):
        super().__init__(simulation)
        self.totalDataNum = 2

        self.t = np.array([])
        self.data1 = np.array([])
        self.data2 = np.array([])
        
    # override
    def setData(self):
        self.t = np.append(self.t, [self.controller.getEnergyError()])
        # self.data1 = np.append(self.data1, [self.controller.getDesiredPosition()])
        # self.data2 = np.append(self.data2, [self.controller.getPosition()])

        # self.data1 = np.append(self.data1, [self.controller.getDesiredVelocity()])
        # self.data2 = np.append(self.data2, [self.controller.getVelocity()])

        #plot data for energy shaping controller
        self.data1 = np.append(self.data1, [self.controller.getDEnergyError()])
        self.data2 = np.append(self.data2, [self.controller.getDEnergyError()])
