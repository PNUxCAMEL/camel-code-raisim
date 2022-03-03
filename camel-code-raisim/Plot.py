from abc import abstractmethod



class Plot:
    def __init__(self, simulation):
        self.sim = simulation
        self.controller =simulation.getController()
        self.totalPlotTime = self.sim.getSimulationDuration()

    @abstractmethod
    def show(self):
        pass

    @abstractmethod
    def getData(self):
        pass