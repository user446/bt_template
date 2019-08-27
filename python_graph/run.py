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
import serial, argparse, collections, logging, io, sys
import numpy as np


# открытие файла логов и его очистка
logging.basicConfig(filename="log.log", 
                    format='%(asctime)s %(message)s', 
                    filemode='w') 
logger=logging.getLogger()
logger.setLevel(logging.DEBUG) 
logger.addHandler(logging.StreamHandler(sys.stdout))

t = 0 #глобальная переменная для хранения времни в милисекундах
class MyWidget(pg.GraphicsWindow):

    def __init__(self, serial, showlen, parent=None):
        super().__init__(parent=parent)
        
        self.serial = serial
        self.showlen = int(showlen)
        self.xData = np.array([])
        self.yData = np.array([])
        self.maxY = 0
        self.minY = 0
        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.data_timer = QtCore.QTimer(self)
        self.data_timer.setInterval(10)  # in milliseconds
        self.data_timer.start()
        self.data_timer.timeout.connect(self.onNewData)
        
        self.show_timer = QtCore.QTimer(self)
        self.show_timer.setInterval(500)  # in milliseconds
        self.show_timer.start()
        self.show_timer.timeout.connect(self.onFixView)
        
        self.plotItem = self.addPlot(title="BlueCardio Output")

        self.plotDataItem = self.plotItem.plot([], pen=pg.mkPen('b', width=5),
                                               symbolBrush=(255, 0, 0), symbolSize=5, symbolPen=None)

    def setData(self, x, y):
        self.plotDataItem.setData(x, y)
        
    def onFixView(self):
        #подправляем вид графика, чтобы не выходил за границы видимости
        self.plotItem.setYRange(self.minY - 20, self.maxY + 20, padding=0)

    def onNewData(self):
        try:
            ser_data = self.serial.read_until(terminator=serial.LF).decode(
                'utf-8')        # считываем сообщение из порта до конца
        except OSError as e:
            logger.error("OS error %s", e)
            return None
        except TypeError as e:
            logger.error("Type error %s", e)
            self.serial.close()
            return None
        data = ser_data.split("::")  # разделяем на составляющие
        if len(data) < 3:   # если сообщение не было принято целиком, то
            logger.error("Message wasn't completely received: %s", data)
            return None         # сбрасываем
        logger.info(ser_data)   # записываем сообщение в файл

        # разделяем числа по пробелам, выкидываем пустые элементы листа
        new_list = data[1].split()
        if len(new_list) != 4:  # если принято не 4 числа, то что-то не так
            logger.error("Wrong amount of received parameters: %s", new_list)
            return None

        global t
        t_list = [0]*4
        i = 0
        while i < 4:
            t_list[i] = t
            t += 4
            i += 1

        try:
            self.xData = np.append(self.xData, np.array(t_list).astype(np.int))
        except ValueError:
            return None
        # преобразуем лист в float и записываем его в данные оси y
        self.yData = np.append(self.yData, np.array(new_list).astype(np.float))

        if len(self.xData) > self.showlen:
            self.xData = np.delete(self.xData, [0, 1, 2, 3])
            self.yData = np.delete(self.yData, [0, 1, 2, 3])
            self.maxY = np.amax(self.yData)
            self.minY = np.amin(self.yData)
        self.setData(self.xData, self.yData)


def main(args):
    logger.info("Starting BlueCardio graph application")
    logger.info("BlueCardio started with arguments: %s", args)
    logger.info("Trying to open port %s...", args.comport)

    # инициализация COM порта
    try:
        ser = serial.Serial(port=args.comport, baudrate=115200,
                        bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE)
    except serial.serialutil.SerialException:
        logger.error("Port can not be configured with these parameters of do not exist")
        return None
    
    # попытка открыть порт
    try:
        ser.open()
    except IOError:  # скорее всего уже открыт, а значит переоткрываем
        logger.error("Port %s is already open, retry...", args.comport)
        ser.close()
        ser.open()
    except OSError:
        logger.error("Port %s not found, please insert the device in port %s", args.comport, args.comport)
        return None
    
    logger.info("Port %s is opened!", args.comport)

    app = QtWidgets.QApplication([])

    pg.setConfigOptions(antialias=False)  # True seems to work as well
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    logger.info("QtWidget initialised!")
    

    win = MyWidget(ser, args.length)
    win.show()
    win.resize(800, 600)
    win.raise_()
    app.exec_()
    ser.close()
    logger.info("Abort action received from user")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script to show BlueCardio realtime output')
    parser.add_argument('-com', action='store', dest='comport', help='Enter the name of your COM port')
    parser.add_argument('-len', action='store', dest='length', help='Enter max length of stored values')
    args = parser.parse_args()
    main(args)
