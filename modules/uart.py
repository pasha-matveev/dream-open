from serial import Serial
import serial.tools.list_ports
import struct
import threading
import logging

class UART:
    def __init__(self, port:str = '/dev/cu.usbserial-210', boudrate:int = 115200, input_format:str = 'i'):
        self.port = port
        self.baudrate = boudrate
        self.timeout = 1

        self.input_format = input_format
        self.data = None

        self.running = False
        self.serial = None
        self.input_thread = None

    def start(self):
        if self.port in self.get_ports():
            self.serial = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            if self.serial.is_open:
                logging.info(f'Arduino found on {self.port}')
                self.running = True
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
                self.input_thread.start()
                logging.info('Arduino connected')
            else:
                logging.error('Arduino not connected')
        else:
            self.serial = None
            logging.error(f'Arduino not found on {self.port}')
    
    def get_ports(self):
        return list(port for port, desc, hwid in serial.tools.list_ports.comports())
    
    def is_open(self):
        return self.serial.is_open
    
    def write(self, data_format:str, *args):
        if self.running:
            data = struct.pack(data_format, *args)
            self.serial.write(data)

    def input_loop(self):
        data_size = struct.calcsize(self.input_format)
        while True:
            recv_size = self.serial.in_waiting
            if recv_size == data_size:
                raw_data = self.serial.read(data_size)
                self.data = struct.unpack(self.input_format, raw_data)
                logging.info(f'[Arduino]: {uart.data}')
            elif recv_size > 0:
                logging.error(f'Unexpected data size: {recv_size}')
                self.serial.reset_input_buffer()

if __name__ == '__main__':
    uart = UART()
    print(uart.get_ports())