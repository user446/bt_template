import glob
import random
import socket
import struct
import logging
import argparse
import sys
from time import sleep
import mne
from collections import deque


def windowed(seq, step=4):
    """ Spits sequance to blocks of size = step.' 
    If last block is less then step, fills it with the last value """
    if step <= 0:
        raise ValueError('step must be >= 1')
    it = iter(seq)
    window = deque([], step)
    i = 0
    for item in it:
        window.append(item)
        i = (i + 1) % step
        if i % step == 0:
            yield tuple(window)
    fillvalue = window[-1]
    while (i % step):
        i = (i + 1) % step
        window.append(fillvalue)
    yield tuple(window)


def connect(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((ip, port))
    s.listen(1)
    logger.info('Connecting...')
    conn, addr = s.accept()
    logger.info('Connection address: %s', addr)
    yield conn, addr


def edfserver(edfpath, args):
    names = glob.glob(edfpath)
    if names is []:
        logger.info('No file to send')
        return

    TCP_IP = '127.0.0.1'
    TCP_PORT = args.port
    if args.channel > 2:
        raise ValueError('Channel number must be less than 3')
    CHANNEL = args.channel

    if(args.filename == 'random'):
        name = random.choice(names)
    else:
        name = './edf\\' + args.filename + '.edf'
    logger.info('File to open: %s', name)
    data = mne.io.read_raw_edf(name)
    raw_data = data.get_data()
    datacount = 0

    conn, addr = connect(TCP_IP, TCP_PORT)
    while True:
        for rec in windowed(zip(data.times, raw_data[CHANNEL])):
            sleep(rec[-1][0]-rec[0][0])
            data = [d[1] for d in rec]
            var = struct.pack('4fi', *data, datacount)
            try:
                conn.sendall(var)
                datacount = datacount + 1
            except socket.error as msg:
                logger.info("Caught exception socket.error : %s", msg)
                conn.close()
                connect(TCP_IP, TCP_PORT)
    conn.close()


if __name__ == "__main__":
    logging.basicConfig(filename="server_log.log",
                        format='%(asctime)s %(message)s',
                        filemode='w')
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    logger.addHandler(logging.StreamHandler(sys.stdout))

    parser = argparse.ArgumentParser(
        description='Script to run edf file server output for BlueCardio')
    parser.add_argument('-port', action='store', dest='port',
                        default='5005', help='Enter the port that will be used for server')
    parser.add_argument('-file', action='store', dest='filename',
                        default='random', type=str, help='Enter the name of the file to open in "./edf" folder, pring "random" to use random file')
    parser.add_argument('-chan', action='store', dest='channel',
                        default='0', type=int, help='Enter the number of channel in file to read, 0-2')
    parser.add_argument('-log', action='store', dest='logging',
                        default='y', help='Activate logger [y/n]')
    args = parser.parse_args()

    if args.logging == 'n':
        logging.disable(logging.INFO)

    edfserver("./edf/*.edf", args)
