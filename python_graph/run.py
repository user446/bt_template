#команды необходимые для сборки проекта
#py -m venv venv
#cd ./venv/Scripts
#activate
#pip install PyQt5
#pip install pyqtgraph
#pip install pyserial
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import serial
import io
import numpy as np

# инициализация COM порта
ser = serial.Serial(port="COM7", baudrate=115200,
                    bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE)

# открытие файла логов и его очистка
f = open("logs.txt", "w+")
f.truncate()
t = 0
x = []
y = []


class MyWidget(pg.GraphicsWindow):

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(10)  # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)

        self.plotItem = self.addPlot(title="BlueCardio Output")
        self.plotItem.setYRange(0, 100, padding=0)

        self.plotDataItem = self.plotItem.plot([], pen=pg.mkPen('b', width=5),
                                               symbolBrush=(255, 0, 0), symbolSize=5, symbolPen=None)

    def setData(self, x, y):
        self.plotDataItem.setData(x, y)

    def onNewData(self):
        ser_data = ser.read_until(terminator=serial.LF).decode(
            'utf-8')        # считываем сообщение из порта до конца
        data = ser_data.split("::")  # разделяем на составляющие
        if len(data) < 3:   # если сообщение не было принято целиком, то
            return          # сбрасываем
        f.write(ser_data)   # записываем сообщение в файл
        print(ser_data)
        # разделяем числа по пробелам, выкидываем пустые элементы листа
        new_list = list(filter(None, data[1].split(" ")))
        if len(new_list) != 4:  # если принято не 4 числа, то что-то не так
            return

        t_list = [0]*4
        i = 0
        global t
        while i < 4:
            t_list[i] = t
            t += 4
            i += 1

        global x
        global y
        try:
            x = np.append(x, np.array(t_list).astype(np.int))
        except ValueError:
            return
        # преобразуем лист в float и записываем его в данные оси y
        y = np.append(y, np.array(new_list).astype(np.float))

        if len(x) > 512:
            x = np.delete(x, [0, 1, 2, 3])
            y = np.delete(y, [0, 1, 2, 3])
        self.setData(x, y)


def main():
    print("Starting BlueCardio graph application")
    print("Trying to open port...")

    # попытка открыть порт
    try:
        ser.open()
    except IOError:  # скорее всего уже открыт, а значит переоткрываем
        ser.close()
        ser.open()

    app = QtWidgets.QApplication([])

    pg.setConfigOptions(antialias=False)  # True seems to work as well
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')

    win = MyWidget()
    win.show()
    win.resize(800, 600)
    win.raise_()
    app.exec_()


if __name__ == "__main__":
    main()
