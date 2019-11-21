import struct
import serial
import socket
import time
from collections import deque
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
        self.total_messages = 0
        self.received = 0
        self.error_percent = 0
        
    def ResetError(self):
        self.total_messages = 0
        self.received = 0

    def GetError(self):
        return self.error_percent
    
    # функция вызываемая из pygatt
    def handle_data(self, handle, value):
        self.data = struct.unpack('4f', bytearray(value[0:16]))
        counter = struct.unpack('i', bytearray(value[16:20]))[0]
        self.received = self.received + 1
        if (self.datacount+1) != counter:
            self.logger.warning(
                "Seems like previous packet was lost: %s", counter)
        self.total_messages = self.total_messages + (counter - self.datacount)
        self.datacount = counter
        self.logger.info("Received data: %s, length: %s, count: %s",
                         self.data, len(self.data), self.datacount)
        self.error_percent = self.received/self.total_messages
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
        self.total_messages = 0
        self.received = 0
        self.error_percent = 0
        
    def ResetError(self):
        self.total_messages = 0
        self.received = 0
        
    def GetError(self):
        return self.error_percent

    def GetParsedData(self):
        try:
            dt = bytearray()
            while(len(dt) < 20):
                packet = self.tcp.recv(20 - len(dt))
                if not packet:
                    break
                dt.extend(packet)
            self.received = self.received + 1
        except socket.error as msg:
            self.logger.info("Caught exception socket.error : %s", msg)
            return None
        if(len(dt) == 20):
            data = struct.unpack('4fi', dt)
        else:
            return None
        self.data = data[0:4]
        if (self.datacount+1) != data[4]:
            self.logger.warning(
                "Seems like previous packet was lost: %s", data[4])
        self.total_messages = self.total_messages + (data[4] - self.datacount)
        self.datacount = data[4]
        try:
            self.error_percent = self.received/self.total_messages
        except:
            self.error_percent = 0
        return (self.datacount, self.data)

    def run(self):
        while True:
            pack = self.GetParsedData()
            self.signal.emit(pack)


class SerialPort(QtCore.QThread):
    signal = QtCore.pyqtSignal('PyQt_PyObject')

    def __init__(self, logger, serial):
        QtCore.QThread.__init__(self)
        self.recieve_buffer = ""
        self.linequeue = deque([])
        self.data = []
        self.datacount = 0
        self.logger = logger
        self.serial = serial
        self.total_messages = 0
        self.received = 0
        self.error_percent = 0
        
    def GetError(self):
        return self.error_percent
    
    def ResetError(self):
        self.total_messages = 0
        self.received = 0

    def GetParsedData(self, ser_data):
        try:
            self.logger.info("Serial data: %s", ser_data)
            self.total_messages = self.total_messages + 1
        except serial.serialutil.SerialException as e:
            self.logger.warning("warning: %s", e)
            return None
        
        if not ser_data:
            self.logger.error("Empty line received: %s", ser_data)
            return None
        
        data = ser_data.split('::')  # разделяем на составляющие
        
        if len(data) != 3:   # если сообщение не было принято целиком или имеет слишком много полей, то
            self.logger.warning("Message wasn't completely received: %s", data)
            return None              # сбрасываем
        
        self.logger.info(ser_data)   # записываем сообщение в файл
        # разделяем числа по пробелам, выкидываем пустые элементы листа
        self.data = data[1].split()
        
        if len(self.data) != 4:  # если принято не 4 числа, то что-то не так
            self.logger.warning(
                "Wrong amount of received parameters error: %s", self.data)
            return None         #сбрасываем
        
        try:
            if (self.datacount+2) != int(data[0]):
                self.logger.warning(
                    "Seems like previous packet was lost: %s : ", data[0])
            self.datacount = int(data[0])
        except ValueError:      #не удалось распарсить счетчик сообщения
            self.logger.warning("Invalid literal was received: %s", data[0])
            return None
        
        self.received = self.received + 1
        
        try:
            self.error_percent = self.received/self.total_messages
        except:
            self.error_percent = 0
            
        try:
            num_data = [x[:-1] for x in self.data]
            wrong_data = len([i for i in num_data if float(i) >= self.datacount]) 
            if wrong_data > 0:
                self.logger.warning(
                    "Seems like something went wrong in data: %s", np.array(self.data).astype(np.float))
                return None
            return self.datacount, self.data #np.array(self.data).astype(np.float)
        
        except ValueError:
            self.logger.warning(
                    "Invalid value literals were received: %s", self.data)
            return None

    def run(self):
        while True:
            self.recieve_buffer += self.serial.read(self.serial.inWaiting()
                 or 1).decode('utf-8')
            lines = self.recieve_buffer.split('\n')
            self.linequeue.extend(lines)
            while len(self.linequeue) > 0:
                data = self.GetParsedData(self.linequeue[0])
                self.signal.emit(data)
                del self.linequeue[0]
            time.sleep(0.01)
