import struct
import serial
import socket
import numpy as np
from PyQt5 import QtCore

# тред не нужен, т.к. pygatt имеет собственный тред


class BLEPort():
    signal = QtCore.pyqtSignal('PyQt_PyObject')

    def __init__(self, logger):
        self.data = []
        self.datacount = 0
        self.time_data = 0
        self.logger = logger

    # функция вызываемая из pygatt
    def handle_data(self, handle, value):
        self.data = struct.unpack('4f', bytearray(value[0:16]))
        counter = struct.unpack('i', bytearray(value[16:20]))[0]
        if (self.datacount+1) != counter:
            self.logger.warning(
                "Seems like previous packet was lost: %s", counter)
        self.datacount = counter
        self.logger.info("Received data: %s, length: %s, count: %s",
                         self.data, len(self.data), self.datacount)
        pass_signal = self.datacount, self.data
        self.signal.emit(pass_signal)  # вызываем сигнал


class TCPPort(QtCore.QThread):
    signal = QtCore.pyqtSignal('PyQt_PyObject')

    def __init__(self, logger, tcp):
        QtCore.QThread.__init__(self)
        self.data = []
        self.datacount = 0
        self.logger = logger
        self.tcp = tcp

    def GetParsedData(self):
        try:
            dt = self.tcp.recv(16)
        except socket.error as msg:
            self.logger.info("Caught exception socket.error : %s", msg)
            return None
        data = struct.unpack('4fi', dt)
        self.data = data[0:3]
        if (self.datacount+1) != data[4]:
            self.logger.warning(
                "Seems like previous packet was lost: %s", data[4])
            self.datacount = data[4]
        return self.datacount, self.data

    def run(self):
        while True:
            data = self.GetParsedData()
            self.signal.emit(data)


class SerialPort(QtCore.QThread):
    signal = QtCore.pyqtSignal('PyQt_PyObject')

    def __init__(self, logger, serial):
        QtCore.QThread.__init__(self)
        self.data = []
        self.datacount = 0
        self.logger = logger
        self.serial = serial

    def GetParsedData(self):
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
        self.data = data[1].split()
        if len(self.data) != 4:  # если принято не 4 числа, то что-то не так
            self.logger.warning(
                "Wrong amount of received parameters error: %s", self.data)
            return None
        if (self.datacount+1) != int(data[0]):
            self.logger.warning(
                "Seems like previous packet was lost: %s", data[0])
        self.datacount = int(data[0])
        return self.datacount, np.array(self.data).astype(np.float)

    def run(self):
        while True:
            data = self.GetParsedData()
            self.signal.emit(data)