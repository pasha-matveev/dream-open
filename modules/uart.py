from serial import Serial
import serial.tools.list_ports
import struct
import threading
import logging
import queue
import time

class UART:
    def __init__(self, boudrate:int = 115200, input_format:str = 'ffff??'):
        self.port = None
        self.baudrate = boudrate
        self.timeout = 1

        self.input_format = input_format
        self.data = None

        self.serial = None
        self.input_thread = None

        self.queue = queue.Queue(maxsize=10)

        self.start_tm = 0
        self.init_delay = 10

    def autodetect_port(self):
        for port, desc, hwid in serial.tools.list_ports.comports():
            if desc == 'USB Serial':
                logging.info(f'Arduino found on {port}')
                return port
        logging.error('Cannot find Arduino')
        return None

    def start(self):
        self.port = self.autodetect_port()
        if self.port == None:
            raise Exception('No Arduino found')
        
        self.serial = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
        if self.serial.is_open:
            logging.info(f'Arduino found on {self.port}')
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
            self.input_thread.start()
            self.start_tm = time.time()
            logging.info('Arduino connected')
        else:
            logging.error('Cannot connect to Arduino')
            raise Exception('Connection error')
    
    def get_ports(self):
        return list(port for port, desc, hwid in serial.tools.list_ports.comports())
    
    @property
    def is_open(self):
        if self.serial == None:
            return False
        return self.serial.is_open
    
    @property
    def writable(self):
        if self.start_tm == 0:
            return False
        return self.start_tm + self.init_delay < time.time()
    
    def write(self, data_format:str, *args):
        if self.is_open:
            data = struct.pack(data_format, *args)
            self.serial.write(data)

    def _input_loop(self):
        data_size = struct.calcsize(self.input_format)
        while True:
            recv_size = self.serial.in_waiting
            if recv_size == data_size:
                raw_data = self.serial.read(data_size)
                data = struct.unpack(self.input_format, raw_data)
                try:
                    self.queue.put_nowait(data)
                except queue.Full:
                    pass
            elif recv_size > 0:
                logging.error(f'Unexpected data size: {recv_size}')
                self.serial.reset_input_buffer()
    
    @property
    def new_data(self):
        return not self.queue.empty()
    
    def read(self):
        self.data = self.queue.get()
        logging.info(f'[Arduino]: {self.data}')

    def stop(self):
        if self.is_open:
            self.serial.close()

if __name__ == '__main__':
    uart = UART()
    print(uart.get_ports())
