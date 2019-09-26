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
import socket
import argparse
import logging
import struct
import serial
import re
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

    ser = None
    ble = None
    sock = None

    if args.communication != 'BLE' and cm is None:
        logger.info("info: Trying to open port %s...", args.communication)
        try:
            ser = serial.Serial(port=args.communication, baudrate=115200,
                                bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE)
            logger.info("info: Port entity initialised for %s",
                        args.communication)
        except serial.serialutil.SerialException as e:
            logger.critical("error: %s", e)
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
                return None
    elif args.communication == 'BLE':
        logger.info(
            "info: Trying to initialize communication via BLE device...")
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
    elif cm is not None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        logger.info("Trying to connect to %s:%s", IP, PORT)
        try:
            sock.connect((IP, int(PORT)))
        except socket.error as msg:
            logger.info("Caught exception socket.error : %s", msg)
            return None
        logger.info("Connected!")

    app = QtGui.QApplication([])

    pg.setConfigOptions(antialias=False)  # True seems to work as well
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    logger.info("info: QtWidget initialised!")

    qrs_compute = False
    if args.logging == 'n':
        logging.disable(logging.INFO)
    if args.qrs == 'y':
        qrs_compute = True

    #win = QtGui.QMainWindow()
    win = MainWindow()
    plot = BlueCardioGraph(ser, ble, sock, qrs_compute, args.length, logger)

    bt_stopupdate = QtWidgets.QPushButton('Stop')
    bt_stopupdate.setToolTip('Stop updating data on a plot')
    bt_stopupdate.clicked.connect(plot.DataUpdSwitch)

    win.AddNewVidget(plot)
    win.AddNewVidget(bt_stopupdate, 1, 0)
    win.show()
    win.resize(800, 600)
    win.raise_()
    app.exec_()
    if ser is not None:
        ser.close()
    if ble is not None:
        adapter.stop()
    if sock is not None:
        sock.close()
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
                        default='BLE', help='Enter the name of your COM%N% port, enter BLE or TCP:%IP%:%PORT%')
    parser.add_argument('-qrs', action='store', dest='qrs',
                        default='n', help='Activate QRS computation [y/n]')
    parser.add_argument('-len', action='store', dest='length', type=int, default=4096,
                        help='Enter max length of stored values, if < 4 then all values will be shown')
    parser.add_argument('-log', action='store', dest='logging',
                        default='n', help='Activate logger [y/n]')
    args = parser.parse_args()

    cm = re.search(
        r'^TCP:.*[0-9]+(?:\.[0-9]+){3}:[0-9]+.*', args.communication)

    if args.communication == 'BLE':
        adapter = pygatt.BGAPIBackend()
    elif cm:
        as_string = str(args.communication).replace(' ', '')
        as_array = as_string.split(':')
        IP = as_array[1]
        PORT = as_array[2]

    main(args)
