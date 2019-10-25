import pygatt
import logging
import struct
from time import sleep

received = 0  
prev_counter = -1
packet_lost = 0
packet_read = 0


def handle_data(handle, value):
    global received
    global prev_counter
    global packet_lost
    global packet_read
    
    if len(value) != 20:
        raise RuntimeWarning(f'Unexpected size { len(value) } in handle data of BLE')

    sdata = struct.unpack('4f', bytearray(value[0:16]))
    counter = struct.unpack('i', bytearray(value[16:20]))[0]
    received += 1
    
    if prev_counter<0:
        prev_counter=counter-1
    
    if prev_counter+1 != counter:
        packet_lost += counter-prev_counter+1
    
    packet_read +=1
    prev_counter=counter

if __name__ == "__main__":
    logging.basicConfig()
    logging.getLogger('pygatt').setLevel(logging.INFO)
    adapter = pygatt.BGAPIBackend()
 
    try:
        adapter.start()
        device = adapter.connect('92:80:e1:03:00:bb', 
                                interval_min=60, 
                                interval_max=76, 
                                supervision_timeout=100, 
                                latency=0)
        device.subscribe("d973f2e1-b19e-11e2-9e96-0800200c9a66", callback=handle_data)
        
        while packet_read + packet_lost < 500:
            sleep(1)
        
        print(f'Lost {packet_lost} from { packet_read + packet_lost} ( { (100 * packet_lost / (packet_read + packet_lost))} %)')

    except Exception as ex:
        logging.error(ex)
    finally:
        adapter.stop()
