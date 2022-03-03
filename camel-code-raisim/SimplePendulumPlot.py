import Plot
import numpy as np
from matplotlib import pyplot as plt

class SimplePendulumPlot(Plot.Plot):
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
    def show(self):
        plt.subplot(2,1,1)
        plt.plot(self.t,self.data1,self.t,self.data2)
        plt.title('position of pendulum')
        plt.xlabel('time')
        plt.xlabel('rad')

        plt.subplot(2,1,2)
        plt.plot(self.t,self.data3,self.t,self.data4)
        plt.title('velocity of pendulum')
        plt.xlabel('time')
        plt.xlabel('rad / sec')

        plt.show()
        
        

    #override
    def getData(self):
        
        self.t = np.append(self.t, [self.sim.getTime()])
        self.data1 = np.append(self.data1, [self.controller.getDesiredPosition()])
        self.data2 = np.append(self.data2, [self.controller.getPosition()])
        self.data3 = np.append(self.data3, [self.controller.getDesiredVelocity()])
        self.data4 = np.append(self.data4, [self.controller.getVelocity()])


