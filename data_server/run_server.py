import sys, glob, random, socket, mne, struct
import numpy as np
from time import sleep

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 16

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

names = glob.glob("./edf/*.edf")
rnd_name = random.choice(names)
data = mne.io.read_raw_edf(rnd_name)
raw_data = data.get_data()
dt = raw_data[0:1][0,:]
time = data.times

# i = 0
# while True:
#     while i < len(dt):
#         sm = time[i+4] - time[i]
#         data = dt[i:i+4]
#         var = struct.pack('4f', *data)
#         print(dt[i:i+4], sm)
#         i = i + 4
#         sleep(sm)
#     if i == len(dt):
#         i = 0

print('Connecting...')
conn, addr = s.accept()
print('Connection address:', addr)
i = 0
while True:
    while i < len(dt):
        sm = time[i+4] - time[i]
        data = dt[i:i+4]
        var = struct.pack('4f', *data)
        conn.sendall(var)
        print(dt[i:i+4], sm)
        i = i + 4
        sleep(sm)
    if i == len(dt):
        i = 0
conn.close()
