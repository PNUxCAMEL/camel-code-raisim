import sys
from tkinter import Button
from PySide6.QtWidgets import *
from PySide6.QtCore import Slot
import numpy as np




class UI(QMainWindow):
    def __init__(self): 
        super().__init__()
        self.setWindowTitle("Simulation Controller")
        self.button = QPushButton(text = "Run", parent=self)
        self.button.clicked.connect(self.buttonPressPlay)
        self.setCentralWidget(self.button)
        self.isButtonPressed = False
        self.button.show()

    @Slot()
    def buttonPressPlay(self):
        self.isButtonPressed = True
        print("Play button is pressed")
    
    def getIsButtonPressed(self):
        return self.isButtonPressed
    
    def setIsButtonPressed(self, booleanValue):
        self.isButtonPressed = booleanValue
