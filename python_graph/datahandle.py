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
        self.recieve_buffer = ''
        self.linequeue = deque([])
        self.data = []
        self.datacount = 0
        self.mean = 0
        self.logger = logger
        self.serial = serial
        self.total_messages = 0
        self.received = 0
        self.error_percent = 0
        self.count_error = 0
        self.previous_lost = []
        self.previous_lost_count = 0
        
    def GetError(self):
        return (self.received, self.total_messages)
    
    def ResetError(self):
        self.total_messages = 0
        self.received = 0
        
    def GetPaketData(self, ser_data):
        self.data = struct.unpack('4d', bytearray(ser_data[7:22]))

    def GetParsedData(self, ser_data):
        
        self.logger.info("Serial data: %s", ser_data)
        if(len(ser_data) < 10):
            self.logger.error("Length of a message is too small to try: %d", len(ser_data))
            raise RuntimeError
        else:
            self.total_messages = self.total_messages + 1
        
        if not ser_data:
            self.logger.error("Empty line received: %s", ser_data)
            raise RuntimeError
            
        data = ser_data.split('::')  # разделяем на составляющие
        #3 поля - без среднего по окну
        # if len(data) != 4:   # если сообщение не было принято целиком или имеет слишком много полей, то
        #     self.logger.warning("Message wasn't completely received: %s", data)
        #     self.unparsed = self.unparsed + 1
        #     raise RuntimeError              # сбрасываем
        
        try:
            self.mean = int(data[2])
        except:
            self.logger.error("Unable to parse mean data: %s", data[2])
            raise RuntimeError
        
        self.logger.info(ser_data)   # записываем сообщение в файл
        # разделяем числа по пробелам, выкидываем пустые элементы листа
        self.data = data[1].split()
        
        if len(self.data) != 4:  # если принято не 4 числа, то что-то не так
            self.logger.warning(
                "Wrong amount of received parameters error: %s", self.data)
            raise RuntimeError         #сбрасываем
        
        try:
            counter = data[0].split()
            if (self.datacount+1) != int(counter[1]):
                self.logger.warning(
                    "Seems like previous packet was lost: %s : %s", counter[1], self.data)
                if counter[1] != self.previous_lost_count:
                    self.previous_lost = self.data
                    self.previous_lost_count = counter[1]
                else:
                    self.logger.error("Seems like same broken package was sent!")
                    raise RuntimeError
            self.datacount = int(counter[1])
            #self.logger.warning("Received number of package is: %s", self.datacount)
        except ValueError as e:      #не удалось распарсить счетчик сообщения
            self.logger.warning("Invalid literal was received: %s", counter[1])
            raise e
        
        self.received = self.received + 1
        
        try:
            self.error_percent = self.received/self.total_messages
        except:
            self.error_percent = 0
            
        try:
            num_data = [x[:-1] for x in self.data]
            # wrong_data = len([i for i in num_data if float(i) < 0.01]) #len([i for i in num_data if float(i) > 0.1 or float(i) < 0.001]) 
            # if wrong_data > 0:
            #     self.logger.warning(
            #         "Seems like something went wrong in data: %s", np.array(self.data).astype(np.float))
            #     raise RuntimeError
            return self.datacount, self.data, self.mean #np.array(self.data).astype(np.float)
        
        except ValueError as e:
            self.logger.warning(
                    "Invalid value literals were received: %s", self.data)
            raise e

    def run(self):
        #import pydevd;pydevd.settrace(suspend=False) #раскомментировать только если нужно отладить треды
        while True:
            try:
                self.recieve_buffer += self.serial.read(self.serial.inWaiting()
                    or 1).decode('utf-8')
            except serial.serialutil.SerialException as e:
                self.logger.warning("warning: %s", e)
                raise e
            lines = self.recieve_buffer.split('\n')
            if len(lines) > 1:
                self.linequeue.extend(lines)
                self.recieve_buffer = ''
                while len(self.linequeue) > 0:
                    indata = self.linequeue.popleft()
                    if indata:
                        try:
                            data = self.GetParsedData(indata)
                        except:
                            continue
                    else:
                        continue
                    self.signal.emit(data)

