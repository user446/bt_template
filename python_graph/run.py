# команды необходимые для сборки проекта
# py -m venv venv
# cd ./venv/Scripts
# activate
# pip install PyQt5
# pip install pyqtgraph
# pip install pyserial
# pip install argparse
# pip install pygatt
import sys
import argparse
import logging
import struct
import serial
import numpy as np
import pygatt
from PyQt5 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg


class BLEPort():
    def __init__(self):
        self.data = []
        self.datacount = 0

    def GetParsedData(self):
        return self.data

    def handle_data(self, handle, value):
        """
        handle -- integer, characteristic read handle the data was received on
        value -- bytearray, the data returned in the notification
        """
        self.data = struct.unpack('ffff', bytearray(value[0:16]))
        self.datacount = struct.unpack('i', bytearray(value[16:20]))
        logger.info("Received data: %s, length: %s, count: %s",
                    self.data, len(self.data), self.datacount)
    
class MyWidget(pg.GraphicsWindow):

    def __init__(self, serial, ble, showlen, parent=None):
        super().__init__(parent=parent)

        self.serial = serial
        self.ble = ble
        self.showlen = showlen
        self.x_data = np.array([])
        self.y_data = np.array([])
        self.max_y = 0
        self.min_y = 0
        self.time_data = 0
        self.mainLayout = self.addLayout(row=2, col=0)
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
        self.proxy = QtGui.QGraphicsProxyWidget()
        self.button = QtGui.QPushButton('button')
        self.proxy.setWidget(self.button)
        self.addItem(self.proxy,2,0)

        self.plotDataItem = self.plotItem.plot([], pen=pg.mkPen('b', width=1),
                                               symbolBrush=(255, 0, 0), symbolSize=1, symbolPen=None)
        if(showlen >= 4):
            self.plotItem.setXRange(0, showlen)

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
            logger.warning("warning: %s", e)
            return None

        if not ser_data:
            logger.error("Empty line received: %s", ser_data)
            return None

        data = ser_data.split("::")  # разделяем на составляющие
        if len(data) < 3:   # если сообщение не было принято целиком, то
            logger.warning("Message wasn't completely received: %s", data)
            return None         # сбрасываем
        logger.info(ser_data)   # записываем сообщение в файл

        # разделяем числа по пробелам, выкидываем пустые элементы листа
        new_list = data[1].split()
        if len(new_list) != 4:  # если принято не 4 числа, то что-то не так
            logger.warning(
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


def main(args):
    logger.info("info: Starting BlueCardio graph application")
    logger.info("info: BlueCardio started with arguments: %s", args)

    chose_comport = True
    ser = None
    ble = None

    if args.communication != 'BLE':
        logger.info("info: Trying to open port %s...", args.communication)
        chose_comport = True
    else:
        logger.info(
            "info: Trying to initialize communication via BLE device...")
        chose_comport = False

    if chose_comport:
        # инициализация COM порта
        try:
            ser = serial.Serial(port=args.comport, baudrate=115200,
                                bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE)
            logger.info("info: Port entity initialised for %s",
                        args.communication)
        except serial.serialutil.SerialException as e:
            logger.critical("error: %s", e)
            chose_comport = False
        # попытка открыть порт
        try:
            ser.open()
            logger.info("info: Port %s is opened!", args.communication)
        except serial.serialutil.SerialException as e:
            logger.error("error: %s, trying to open again...", e)
            try:
                ser.close()
                ser.open()
                logger.info("info: Port %s is opened!", args.communication)
            except serial.serialutil.SerialException as e:
                logger.error("error: %s, abort!", e)
                chose_comport = False
    else:
        try:
            ble = BLEPort()
            logger.info("Trying to install connection with BLE device")
            adapter.start()
            device = adapter.connect('92:80:e1:03:00:bb')
            device.subscribe("d973f2e1-b19e-11e2-9e96-0800200c9a66",
                             callback=ble.handle_data)
        except:
            logger.info(
                "Unable to install connection with BLE device for unknown reason")
            adapter.stop()
            return None

    app = QtWidgets.QApplication([])

    pg.setConfigOptions(antialias=False)  # True seems to work as well
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    logger.info("info: QtWidget initialised!")

    if args.logging == 'n':
        logging.disable(logging.INFO)

    win = MyWidget(ser, ble, args.length)
    win.show()
    win.resize(800, 600)
    win.raise_()
    app.exec_()
    if ser is not None:
        ser.close()
    if ble is not None:
        adapter.stop()
    logger.info("info: Abort action received from user")


if __name__ == "__main__":

    logging.basicConfig(filename="log.log",
                        format='%(asctime)s %(message)s',
                        filemode='w')
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    logger.addHandler(logging.StreamHandler(sys.stdout))

    parser = argparse.ArgumentParser(
        description='Script to show BlueCardio realtime output')
    parser.add_argument('-comm', action='store', dest='communication',
                        default='BLE', help='Enter the name of your COM port or enter BLE to communicate directly via BLE device')
    parser.add_argument('-len', action='store', dest='length', type=int, default=4096,
                        help='Enter max length of stored values, if < 4 then all values will be shown')
    parser.add_argument('-log', action='store', dest='logging',
                        default='y', help='Activate logger [y/n]')
    args = parser.parse_args()

    if args.communication == 'BLE':
        adapter = pygatt.BGAPIBackend()
    main(args)
