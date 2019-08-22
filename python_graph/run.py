import serial
import io
import numpy as np
import matplotlib as mtp
import matplotlib.pyplot as pt
import matplotlib.animation as anim

ser = serial.Serial(port="COM7", baudrate=115200,
                    bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE)

fig = pt.figure()
ax = pt.axes(xlim = [0,1024], ylim = [-3000,3000])
line, = ax.plot([], [])
xdata = []
ydata = []
f = open("logs.txt", "w+")
f.truncate()
t = 0


def run(data):
    t,y = data
    xdata.append(t)
    ydata.append(y)
    line.set_data(xdata, ydata)
    return line,


def get_data():
    t_list = [0]*4
    i = 0
    global t
    while i < 4:
        t_list[i] =  t
        t += 4
        i += 1
    ser_data = ser.read_until(terminator=serial.LF).decode('utf-8')
    data = ser_data.split("::")
    if len(data) < 3:
        return
    f.write(ser_data)
    
    new_list = list(filter(None, data[1].split(" ")))
    if len(new_list) < 4:
        return
    dt = np.array(new_list).astype(np.float)
    
    print(t_list, dt)
    yield t_list, dt


def main():
    print("Starting BlueCardio graph application")
    print("Trying to open port...")

    try:
        ser.open()
    except IOError:
        ser.close()
        ser.open()
    
    ani = anim.FuncAnimation(fig, run, get_data, interval=50, blit=False)
    pt.show()


if __name__ == "__main__":
    main()
