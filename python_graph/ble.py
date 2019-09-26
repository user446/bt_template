import struct


class BLEPort():
    def __init__(self, logger):
        self.data = []
        self.datacount = 0
        self.logger = logger

    def GetParsedData(self):
        return self.datacount, self.data

    def handle_data(self, handle, value):
        """
        handle -- integer, characteristic read handle the data was received on
        value -- bytearray, the data returned in the notification
        """
        self.data = struct.unpack('ffff', bytearray(value[0:16]))
        self.datacount = struct.unpack('i', bytearray(value[16:20]))[0]
        self.logger.info("Received data: %s, length: %s, count: %s",
                         self.data, len(self.data), self.datacount)
