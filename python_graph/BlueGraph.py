import logging
import struct
import serial
import threading
import socket
import numpy as np
from PyQt5 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg
import datahandle
from QRS import ECG_QRS_detect

# https://kushaldas.in/posts/pyqt5-thread-example.html


class BlueCardioGraph(pg.GraphicsWindow):
    def __init__(self, communication, qrs, showlen, logger, parent=None):
        pg.GraphicsWindow.__init__(self)

        self.comm = communication
        self.logger = logger
        self.showlen = showlen
        self.x_data = np.array([])
        self.y_data = np.array([])
        self.max_y = 0
        self.min_y = 0
        self.time_data = 0
        self.data_switch = True
        self.qrs_compute = qrs
        self.counter = 0

        self.show_timer = QtCore.QTimer(self)
        self.show_timer.setInterval(100)  # in milliseconds
        self.show_timer.start()
        self.show_timer.timeout.connect(self.onFixView)

        if self.qrs_compute:
            self.qrs_timer = QtCore.QTimer(self)
            self.qrs_timer.setInterval(2000)
            self.qrs_timer.start()
            self.qrs_timer.timeout.connect(self.OnQRSCompute)

        self.plotItem = self.addPlot(title="BlueCardio Output")

        self.plotDataItem = self.plotItem.plot([], pen=pg.mkPen('b', width=1),
                                               symbolBrush=(255, 0, 0), symbolSize=1, symbolPen=None)

        self.plotDataQ = self.plotItem.plot(
            [], pen=None, symbol='o', symbolBrush='c', symbolSize=5)
        self.plotDataR = self.plotItem.plot(
            [], pen=None, symbol='o', symbolBrush='r', symbolSize=5)
        self.plotDataS = self.plotItem.plot(
            [], pen=None, symbol='o', symbolBrush='y', symbolSize=5)
        if(showlen >= 4):
            self.plotItem.setXRange(0, showlen)

        self.comm.signal.connect(self.OnNewData)
        if self.comm.start() is not None: #проверка - стоит ли запускать тред, или он уже запущен (случай с pygatt)
            self.comm.start()

    def DataUpdSwitch(self):
        #pen = pg.mkPen('g', width=2)
        if self.data_switch:
            #pen.drawLine(pg.Point(self.x_data[-1], self.min_y - 10),pg.Point(self.x_data[-1],self.max_y + 10))
            self.data_timer.stop()
            self.show_timer.stop()
            self.data_switch = False
        else:
            #pen.drawLine(pg.Point(self.x_data[-1], self.min_y - 10),pg.Point(self.x_data[-1],self.max_y + 10))
            self.data_timer.start()
            self.show_timer.start()
            self.data_switch = True

    def OnQRSCompute(self):
        if self.qrs_compute is True:
            R_peaks, S_point, Q_point = ECG_QRS_detect(self.y_data, 360)
            if R_peaks.any():
                self.setQRS(self.x_data, Q_point, R_peaks, S_point)

    def setQRS(self, x, Q, R, S):
        self.plotDataQ.setData(x[Q], self.y_data[Q])
        self.plotDataR.setData(x[R], self.y_data[R])
        self.plotDataS.setData(x[S], self.y_data[S])

    def setData(self, x, y):
        self.plotDataItem.setData(x, y)

    def onFixView(self):
        # подправляем вид графика, чтобы не выходил за границы видимости
        if self.y_data.any():
            self.max_y = np.amax(self.y_data)
            self.min_y = np.amin(self.y_data)
            self.plotItem.setYRange(
                self.min_y - 10, self.max_y + 10)

    def OnNewData(self, result):
        try:
            counter, data = result
        except:
            return None
        t_list = [0]*4
        i = 0
        while i < 4:
            t_list[i] = self.time_data
            self.time_data += 4
            i += 1
        try:
            self.x_data = np.append(
                self.x_data, np.array(t_list).astype(np.int))
        except ValueError:
            return None
        # преобразуем лист в float и записываем его в данные оси y
        self.y_data = np.append(
            self.y_data, data)
        if self.x_data[-1] > self.showlen and self.showlen >= 4:
            self.x_data = np.delete(self.x_data, [0, 1, 2, 3])
            self.y_data = np.delete(self.y_data, [0, 1, 2, 3])
            self.plotItem.setXRange(
                self.x_data[-1] - self.showlen, self.x_data[-1])
        self.setData(self.x_data, self.y_data)
