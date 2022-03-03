import threading
import time
import UI

# TODO : time.sleep is not working well in windows 10. (kernel problem.)

class CAMELThread (threading.Thread):
   def __init__(self, sim):
      threading.Thread.__init__(self)
      self.sim = sim
      self.ui = UI.UI()
      self.iteration = 0
      self.simDurationTime = 0
      self.ui.show()

   def run(self):
      while(True):
         if(self.ui.getIsButtonPressed()):
            if(self.sim.getSimulationDuration() > self.simDurationTime):
               # print("simulation time : ", self.sim.getTime())
               self.simDurationTime += self.sim.getDT()
               self.iteration += 1
               if(self.sim.isFastSimulation):
                  self.fastSimulation()
               else:
                  self.realTimeSimulation()
                  
               if(self.sim.isDataPlot):   
                  self.sim.getPlot().getData()
               
            else:
               if(self.sim.isDataPlot):
                  self.sim.getPlot().show()
               
               print("simulation time : ", self.sim.getTime())
               self.ui.setIsButtonPressed(False)
               self.simDurationTime = 0
         
      
   def delay(self, delayTime):   # should be changed later.
      startTime = time.time_ns()
      while(True):
         currentTime = time.time_ns()
         if((currentTime - startTime)*1e-9 > delayTime):
            break

   def fastSimulation(self):
      # print("current time :", self.sim.getTime())
      self.sim.controller.doControl()
      self.sim.integrate()

   def realTimeSimulation(self):
      self.delay(self.sim.getDT() / 2)
      # print("current time :", self.sim.getTime())
      self.sim.controller.doControl()
      self.sim.integrate()