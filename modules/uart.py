from serial import Serial
import serial.tools.list_ports
import struct

class UART:
    def __init__(self, port, baudrate: int = 9600, timeout: int = 1):
        self.serial = Serial(port=port, boudrate=baudrate, timeout=timeout)
    
    def read(self, data_format:str):
        data_size = struct.calcsize(data_format)
        data = self.serial.read(data_size)
        return struct.unpack(data_format, data)
    
    def write(self, data_format:str, *args):
        data = struct.pack(data_format, *args)
        self.serial.write(data)