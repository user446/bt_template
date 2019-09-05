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
from BlueGraph import BlueCardioGraph
from ble import BLEPort
from mainwindow import MainWindow
import pyqtgraph as pg


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
            ble = BLEPort(logger)
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

    app = QtGui.QApplication([])

    pg.setConfigOptions(antialias=False)  # True seems to work as well
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    logger.info("info: QtWidget initialised!")

    if args.logging == 'n':
        logging.disable(logging.INFO)
    
    #win = QtGui.QMainWindow()
    win = MainWindow()
    plot = BlueCardioGraph(ser, ble, args.length, logger)
    
    bt_stopupdate = QtWidgets.QPushButton('Stop')
    bt_stopupdate.setToolTip('Stop updating data on a plot')
    bt_stopupdate.clicked.connect(plot.DataUpdSwitch)
    
    win.AddNewVidget(plot)
    win.AddNewVidget(bt_stopupdate,1,0)
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
    logger.setLevel(logging.INFO)
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
