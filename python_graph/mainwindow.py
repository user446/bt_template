import sys
from PyQt5 import QtCore, QtWidgets, QtGui


class MainWindow(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)

        self.centralWidget = QtGui.QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.Layout = QtGui.QGridLayout(self)

        self.centralWidget.setLayout(self.Layout)

    def SetSize(self, x, y):
        self.setMinimumSize(QtGui.QSize(x, y))

    def SetTitle(self, title):
        self.setWindowTitle(title)

    def SetLayout(self, layout):
        self.Layout = layout
        self.centralWidget.setLayout(layout)

    def AddNewVidget(self, vidget, gridpos_x=0, gridpos_y=0):
        self.Layout.addWidget(vidget, gridpos_x, gridpos_y)
