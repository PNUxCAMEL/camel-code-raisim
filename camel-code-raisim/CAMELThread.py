import threading
import time

# TODO : time.sleep is not working well in windows 10. (kernel problem.)

class CAMELThread (threading.Thread):
   def __init__(self, sim):
      threading.Thread.__init__(self)
      self.sim = sim

   def run(self):
      if(self.sim.isFastSimulation):
         self.fastSimulation()
      else:
         self.realTimeSimulation()
      
   def delay(self, delayTime):   # should be changed later.
      startTime = time.time_ns()
      while(True):
         currentTime = time.time_ns()
         if((currentTime - startTime)*1e-9 > delayTime):
            break

   def fastSimulation(self):
      while(True):
         # print("current time :", self.sim.getTime())
         self.sim.controller.doControl()
         self.sim.integrate()

   def realTimeSimulation(self):
      while(True):
         self.delay(0.0025)
         # print("current time :", self.sim.getTime())
         self.sim.controller.doControl()
         self.sim.integrate()