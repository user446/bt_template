import logging
import serial
import numpy as np
from PyQt5 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg
import ble

class BlueCardioGraph(pg.GraphicsWindow):

    def __init__(self, serial, ble, showlen, logger, parent=None):
        pg.GraphicsWindow.__init__(self)

        self.serial = serial
        self.ble = ble
        self.logger = logger
        self.showlen = showlen
        self.x_data = np.array([])
        self.y_data = np.array([])
        self.max_y = 0
        self.min_y = 0
        self.time_data = 0
        self.data_switch = True
        #self.mainLayout = self.addLayout(row=1, col=0)
        #self.setLayout(self.mainLayout)

        self.data_timer = QtCore.QTimer(self)
        self.data_timer.setInterval(10)  # in milliseconds
        self.data_timer.start()
        if serial:
            self.data_timer.timeout.connect(self.onNewData)
        elif ble:
            self.data_timer.timeout.connect(self.onNewData_fromBLE)

        self.show_timer = QtCore.QTimer(self)
        self.show_timer.setInterval(100)  # in milliseconds
        self.show_timer.start()
        self.show_timer.timeout.connect(self.onFixView)
        
        self.plotItem = self.addPlot(title="BlueCardio Output")

        self.plotDataItem = self.plotItem.plot([], pen=pg.mkPen('b', width=1),
                                               symbolBrush=(255, 0, 0), symbolSize=1, symbolPen=None)
        if(showlen >= 4):
            self.plotItem.setXRange(0, showlen)
         
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

    def setData(self, x, y):
        self.plotDataItem.setData(x, y)

    def onFixView(self):
        # подправляем вид графика, чтобы не выходил за границы видимости
        if self.y_data.any():
            self.max_y = np.amax(self.y_data)
            self.min_y = np.amin(self.y_data)
            self.plotItem.setYRange(
                self.min_y - 10, self.max_y + 10, padding=0)

    def onNewData_fromBLE(self):
        data = self.ble.GetParsedData()
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
        self.y_data = np.append(self.y_data, data)

        if self.x_data[-1] > self.showlen and self.showlen >= 4:
            self.x_data = np.delete(self.x_data, [0, 1, 2, 3])
            self.y_data = np.delete(self.y_data, [0, 1, 2, 3])
            self.plotItem.setXRange(
                self.x_data[-1] - self.showlen, self.x_data[-1])
        self.setData(self.x_data, self.y_data)

    def onNewData(self):
        try:
            ser_data = self.serial.read_until(terminator=serial.LF).decode(
                'utf-8')        # считываем сообщение из порта до конца
        except serial.serialutil.SerialException as e:
            self.logger.warning("warning: %s", e)
            return None

        if not ser_data:
            self.logger.error("Empty line received: %s", ser_data)
            return None

        data = ser_data.split("::")  # разделяем на составляющие
        if len(data) < 3:   # если сообщение не было принято целиком, то
            self.logger.warning("Message wasn't completely received: %s", data)
            return None         # сбрасываем
        self.logger.info(ser_data)   # записываем сообщение в файл

        # разделяем числа по пробелам, выкидываем пустые элементы листа
        new_list = data[1].split()
        if len(new_list) != 4:  # если принято не 4 числа, то что-то не так
            self.logger.warning(
                "Wrong amount of received parameters error: %s", new_list)
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
            self.y_data, np.array(new_list).astype(np.float))

        if self.x_data[-1] > self.showlen and self.showlen >= 4:
            self.x_data = np.delete(self.x_data, [0, 1, 2, 3])
            self.y_data = np.delete(self.y_data, [0, 1, 2, 3])
            self.plotItem.setXRange(
                self.x_data[-1] - self.showlen, self.x_data[-1])
        self.setData(self.x_data, self.y_data)