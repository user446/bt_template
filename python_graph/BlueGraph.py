import logging
import struct
import serial
import threading
import socket
import numpy as np
from PyQt5 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg
import datahandle
import mne
from detection import detect

# https://kushaldas.in/posts/pyqt5-thread-example.html


class BlueCardioGraph(pg.GraphicsWindow):
    def __init__(self, communication, mark, showlen, logger, parent=None):
        pg.GraphicsWindow.__init__(self)

        self.comm = communication
        self.logger = logger
        self.showlen = showlen
        self.x_data = np.array([])
        self.y_data = np.array([])
        self.R_peak_data = np.array([])
        self.R_peak_time = np.array([])
        self.SR_peak_data = np.array([])
        self.SR_peak_time = np.array([])
        self.Window_markers = np.array([])
        self.Window_time = np.array([])
        self.max_y = 0
        self.min_y = 0
        self.time_data = 0
        self.data_switch = True
        if mark == 'internal':
            self.markers = mark
        elif isinstance(mark, str) or not isinstance(mark, bool):
            raise ValueError('threshold value must be "internal" or a bool')
        else:
            self.markers = mark
        self.counter = 0

        self.show_timer = QtCore.QTimer(self)
        self.show_timer.setInterval(100)  # in milliseconds
        self.show_timer.start()
        self.show_timer.timeout.connect(self.onFixView)
        
        self.error_timer = QtCore.QTimer(self)
        self.error_timer.setInterval(5000)  # in milliseconds
        self.error_timer.start()
        self.error_timer.timeout.connect(self.onPrintError)

        if self.markers != 'internal' and self.markers:
            self.mark_timer = QtCore.QTimer(self)
            self.mark_timer.setInterval(500)
            self.mark_timer.start()
            self.mark_timer.timeout.connect(self.OnQRSCompute)

        self.plotItem = self.addPlot(title="BlueCardio Output")

        self.plotDataItem = self.plotItem.plot([], pen=pg.mkPen('b', width=1),
                                               symbolBrush=(255, 0, 0), symbolSize=3, symbolPen=None)

        self.plotDataQ = self.plotItem.plot(
            [], pen=None, symbol='x', symbolBrush='b', symbolSize=14)
        self.plotDataR = self.plotItem.plot( 
            [], pen=None, symbol='o', symbolBrush='r', symbolSize=10)
        self.plotDataS = self.plotItem.plot(
            [], pen=None, symbol='s', symbolBrush='g', symbolSize=12)
        if(showlen >= 4):
            self.plotItem.setXRange(0, showlen)

        try: #проверка - стоит ли запускать тред, или он уже запущен (случай с pygatt)
            self.comm.start()
        except AttributeError:
            self.logger.warning("Ok, this is pygatt, no need to start()")
        self.comm.signal.connect(self.OnNewData)

    def DataUpdSwitch(self):
        #pen = pg.mkPen('g', width=2)
        if self.data_switch:
            #pen.drawLine(pg.Point(self.x_data[-1], self.min_y - 10),pg.Point(self.x_data[-1],self.max_y + 10))
            #self.data_timer.stop()
            self.comm.signal.disconnect()
            self.show_timer.stop()
            if self.markers != 'internal' and self.markers:
                self.mark_timer.stop()
            self.data_switch = False
        else:
            #pen.drawLine(pg.Point(self.x_data[-1], self.min_y - 10),pg.Point(self.x_data[-1],self.max_y + 10))
            #self.data_timer.start()
            self.comm.signal.connect(self.OnNewData)
            self.show_timer.start()
            if self.markers != 'internal' and self.markers:
                self.mark_timer.start()
            self.data_switch = True

    def OnQRSCompute(self):
        if self.markers is True and self.x_data[-1] > self.showlen:
            R_peaks0 = mne.preprocessing.ecg.qrs_detector(256, self.y_data, thresh_value = 'auto')
            R_peaks1 = np.array(detect(self.y_data, 256)).astype(np.int)
            if R_peaks1.any() and R_peaks0.any():
                self.setQRS(self.x_data, R = R_peaks0, S = R_peaks1)

    def setQRS(self, x, R, S):
        #self.plotDataQ.setData(x[Q], self.y_data[Q])
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
                self.min_y, self.max_y)
            
    def onPrintError(self):
        self.logger.warning("Percentage of received messages: %s", self.comm.GetError())
        self.comm.ResetError()

    def OnNewData(self, result):
        try:
            (counter, data) = result
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
        if self.markers == 'internal':
            i = 0
            
            n = 0
            for x in data:
                parsable = False
                while not parsable:
                    try:
                        int(x[:n])
                        parsable = True
                    except:
                        parsable = False
                        n = n - 1
                    if n == -5:
                        raise RuntimeError
            
            num_data = list()
            marks = list()
            for x in data:
                if x[-1:] != 'N':
                    num_data.append(x[:n])
                    marks.append(x[n:])
                else:
                    num_data.append(x[:-1])
                    marks.append(x[-1:])
            
            for mk in marks:
                for x in mk:
                    if x == 'R':
                        self.R_peak_data = np.append(self.R_peak_data, num_data[i])
                        self.R_peak_time = np.append(self.R_peak_time, t_list[i])
                    if x == 'W':
                        self.Window_markers = np.append(self.Window_markers, num_data[i])
                        self.Window_time = np.append(self.Window_time, t_list[i])
                    if x == 'S':
                        self.SR_peak_data = np.append(self.SR_peak_data, num_data[i])
                        self.SR_peak_time = np.append(self.SR_peak_time, t_list[i])
                i+=1
            self.y_data = np.append(
                self.y_data, np.array(num_data).astype(np.float))
        else:
            self.y_data = np.append(
                self.y_data, np.array(data).astype(np.float))
        
        
        if self.x_data[-1] > self.showlen and self.showlen >= 4:
            self.x_data = np.delete(self.x_data, [0, 1, 2, 3])
            self.y_data = np.delete(self.y_data, [0, 1, 2, 3])
            if self.Window_time.size:
                if self.Window_time[0] < self.x_data[0]:
                    self.Window_time = np.delete(self.Window_time, [0])
                    self.Window_markers = np.delete(self.Window_markers, [0])
            if self.R_peak_time.size:
                if self.R_peak_time[0] < self.x_data[0]:
                    self.R_peak_time = np.delete(self.R_peak_time, [0])
                    self.R_peak_data = np.delete(self.R_peak_data, [0])
            if self.SR_peak_time.size:
                if self.SR_peak_time[0] < self.x_data[0]:
                    self.SR_peak_time = np.delete(self.SR_peak_time, [0])
                    self.SR_peak_data = np.delete(self.SR_peak_data, [0])
            self.plotItem.setXRange(
                self.x_data[-1] - self.showlen, self.x_data[-1])
        if self.markers == 'internal':
            self.plotDataR.setData(self.R_peak_time, self.R_peak_data)
            self.plotDataS.setData(self.Window_time, self.Window_markers)
            self.plotDataQ.setData(self.SR_peak_time, self.SR_peak_data)
        self.setData(self.x_data, self.y_data)
