from PySide6.QtWidgets import *
from PySide6.QtCore import Slot
import pyqtgraph as pg




class UI(QMainWindow):
    def __init__(self): 
        super().__init__()
        self.setWindowTitle("Simulation Controller")
        self.initializeGraph()
        self.initializeButton()
        self.initializeWindow()
        self.show()

    def initializeButton(self):
        self.button = QPushButton(text = "Run", parent=self)
        self.button.clicked.connect(self.buttonPressPlay)
        self.button.move(400,0)
        self.isButtonPressed = False

    def initializeGraph(self):
        self.graphWidget = pg.PlotWidget(parent=self)
        self.graphWidget.setBackground('w')
        self.setCentralWidget(self.graphWidget)

    def initializeWindow(self):
        self.setMinimumHeight(300)
        self.setMinimumWidth(500)
        styles = {'color':'b', 'font-size':'10px'}
        self.graphWidget.setLabel('left', 'rad', **styles)
        self.graphWidget.setLabel('bottom', 'time [s]', **styles)
        
    @Slot()
    def buttonPressPlay(self):
        self.isButtonPressed = True
        print("Play button is pressed")
    
    def getIsButtonPressed(self):
        return self.isButtonPressed
    
    def setIsButtonPressed(self, booleanValue):
        self.isButtonPressed = booleanValue

    def plot(self,x,y):
        pen = pg.mkPen(color=(255, 0, 0))
        self.graphWidget.plot(x, y, pen=pen)

    def plot(self,x1,y1,x2,y2):
        pen1 = pg.mkPen(color=(0, 0, 255))
        self.graphWidget.plot(x1, y1, pen=pen1)        
        pen2 = pg.mkPen(color=(255, 0, 0))
        self.graphWidget.plot(x2, y2, pen=pen2)        

    def setPlotTitle(self, title):
        self.graphWidget.setTitle(title)

    # def setPlotLegend(self):
    #     pen1 = pg.mkPen(color=(0, 0, 255))
    #     self.graphWidget.plot([0 , 0], [0, 0], pen=pen1)        
    #     pen2 = pg.mkPen(color=(255, 0, 0))
    #     self.graphWidget.plot([0, 0], [0, 0], pen=pen2)        
        