#команды необходимые для сборки проекта
#py -m venv venv
#cd ./venv/Scripts
#activate
#pip install PyQt5
#pip install pyqtgraph
#pip install pyserial
#pip install argparse
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import serial, argparse, logging, sys
import numpy as np

class MyWidget(pg.GraphicsWindow):

    def __init__(self, serial, showlen, parent=None):
        super().__init__(parent=parent)
        
        self.serial = serial
        self.showlen = showlen
        self.xData = np.array([])
        self.yData = np.array([])
        self.meanY = 0
        self.time_data = 0
        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.data_timer = QtCore.QTimer(self)
        self.data_timer.setInterval(10)  # in milliseconds
        self.data_timer.start()
        self.data_timer.timeout.connect(self.onNewData)
        
        self.show_timer = QtCore.QTimer(self)
        self.show_timer.setInterval(100)  # in milliseconds
        self.show_timer.start()
        self.show_timer.timeout.connect(self.onFixView)
        
        self.plotItem = self.addPlot(title="BlueCardio Output")

        self.plotDataItem = self.plotItem.plot([], pen=pg.mkPen('b', width=5),
                                               symbolBrush=(255, 0, 0), symbolSize=5, symbolPen=None)
    
    def setData(self, x, y):
        self.plotDataItem.setData(x, y)
        
    def onFixView(self):
        #подправляем вид графика, чтобы не выходил за границы видимости
        if self.yData.any():
            self.meanY = np.mean(self.yData)
            self.plotItem.setYRange(self.meanY - 20, self.meanY + 20, padding=0)

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
            logger.warning("Wrong amount of received parameters error: %s", new_list)
            return None
        
        t_list = [0]*4
        i = 0
        while i < 4:
            t_list[i] = self.time_data
            self.time_data += 4
            i += 1

        try:
            self.xData = np.append(self.xData, np.array(t_list).astype(np.int))
        except ValueError:
            return None
        # преобразуем лист в float и записываем его в данные оси y
        self.yData = np.append(self.yData, np.array(new_list).astype(np.float))
        
        if self.xData[-1] > self.showlen and self.showlen >= 4:
            self.xData = np.delete(self.xData, [0, 1, 2, 3])
            self.yData = np.delete(self.yData, [0, 1, 2, 3])
        self.setData(self.xData, self.yData)


def main(args):
    logger.info("info: Starting BlueCardio graph application")
    logger.info("info: BlueCardio started with arguments: %s", args)
    logger.info("info: Trying to open port %s...", args.comport)

    # инициализация COM порта
    try:
        ser = serial.Serial(port=args.comport, baudrate=115200,
                        bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE)
    except serial.serialutil.SerialException as e:
        logger.critical("error: %s", e)
        return None
    
    logger.info("info: Port entity initialised for %s", args.comport)
    # попытка открыть порт
    try:
        ser.open()
    except serial.serialutil.SerialException as e:
        logger.error("error: %s, trying to open again...", e)
        try:
            ser.close()
            ser.open()
        except serial.serialutil.SerialException as e:
            logger.error("error: %s, abort!", e)
            return None
    
    logger.info("info: Port %s is opened!", args.comport)

    app = QtWidgets.QApplication([])

    pg.setConfigOptions(antialias=False)  # True seems to work as well
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    logger.info("info: QtWidget initialised!")
    
    if args.logging:
        logging.disable(logging.INFO)
    
    win = MyWidget(ser, args.length)
    win.show()
    win.resize(800, 600)
    win.raise_()
    app.exec_()
    ser.close()
    logger.info("info: Abort action received from user")

if __name__ == "__main__":
    
    logging.basicConfig(filename="log.log", 
                        format='%(asctime)s %(message)s', 
                        filemode='w') 
    logger=logging.getLogger()
    logger.setLevel(logging.DEBUG) 
    logger.addHandler(logging.StreamHandler(sys.stdout))
    
    parser = argparse.ArgumentParser(description='Script to show BlueCardio realtime output')
    parser.add_argument('-com', action='store', dest='comport', default='COM7', help='Enter the name of your COM port')
    parser.add_argument('-len', action='store', dest='length', type=int, default=4096, help='Enter max length of stored values, if < 4 then all values will be shown')
    parser.add_argument('-log', action='store', dest='logging', type=bool, default=False, help='Activate or disactivate logger')
    args = parser.parse_args()
    main(args)
