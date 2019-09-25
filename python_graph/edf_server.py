import glob, random, socket, struct, logging, argparse, sys
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


def edfserver(edfpath, args):
    names = glob.glob(edfpath)
    if names is []:
        logger.info('No file to send')
        return

    TCP_IP = '127.0.0.1'
    TCP_PORT = args.port
    CHANNEL = args.channel
    
    if(args.file == 'random'):
        name = random.choice(names)
    else:
        name = args.filename
    data = mne.io.read_raw_edf(name)
    raw_data = data.get_data()
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    logger.info('Connecting...')
    conn, addr = s.accept()
    logger.info('Connection address: %s', addr)

    while True:
        for rec in windowed(zip(data.times, raw_data[CHANNEL])):
            sleep(rec[-1][0]-rec[0][0])
            data = [d[1] for d in rec]
            var = struct.pack('4f', *data)
            conn.sendall(var)
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
                        default='random', help='Enter the name of the file to open in "./edf" folder, pring "random" to use random file')
    parser.add_argument('-chan', action='store', dest='channel',
                        default='0', type=int, help='Enter the number of channel in file to read, 0-4')
    parser.add_argument('-log', action='store', dest='logging',
                        default='y', help='Activate logger [y/n]')
    args = parser.parse_args()
    
    if args.logging == 'n':
        logging.disable(logging.INFO)
        
    edfserver("./edf/*.edf", args)
