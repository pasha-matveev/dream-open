import subprocess
import threading
import logging
from queue import Queue
import serial.tools.list_ports
import math


class LidarObject:
    def __init__(self):
        self.angle = 0
        self.dist = 0
        self.rotation = 0
        self.width = 0
        self.height = 0
    
    def update(self, angle, dist, rotation, width, height):
        self.angle = float(angle)
        self.dist = float(dist)
        self.rotation = float(rotation)
        self.width = int(width)
        self.height = int(height)
        
        if self.width < self.height:
            self.width, self.height = self.height, self.width
            self.rotation -= math.pi / 2
        
        self.rotation = -self.rotation + math.pi / 2
        
        while self.rotation < 0:
            self.rotation += math.pi * 2
        while self.rotation > math.pi:
            self.rotation -= math.pi


class Lidar:
    def __init__(self, args):
        self.args = args
        self.lidar_path = './lidar/build/bin/main'

        self.proc = None
        self.output_thread = None
        self.output_queue = Queue()

        self.field = LidarObject()
        self.obstacles_data = []
        
        self.data = []

    def find_port(self):
        ports = serial.tools.list_ports.comports()
        for port, desc, hwid in ports:
            if desc == 'CP2102N USB to UART Bridge Controller':
                logging.info(f'Lidar on port: {port}')
                return port
        logging.error('Cannot find Lidar')
        return None

    def start(self):
        self.port = self.find_port()
        if self.port == None:
            raise Exception('No Lidar found')

        command = [self.lidar_path, '1' if self.args.lidar else '0', self.port]

        self.proc = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,   # lines come back as strings
            bufsize=1    # line-buffered on Python side
        )

        self.output_thread = threading.Thread(
            target=self._output_loop, args=(self.proc, self.output_queue), daemon=True)
        self.output_thread.start()

    def _output_loop(self, proc, output_queue):
        for line in iter(proc.stdout.readline, ''):
            line = line.rstrip('\n')
            data = line.split()
            if len(data):
                if data[0] == 'Data':
                    output_queue.put(data[1:])
                else:
                    logging.error(line)
            else:
                logging.error(line)

    def stop(self):
        if self.proc != None:
            self.proc.terminate()
            self.proc.wait()
        if self.output_thread != None:
            self.output_thread.join()

    def compute(self):
        self.data = self.output_queue.get()
        self.field.update(*self.data[0:5])
        # self.obstacles_data = []
        # for i in range(5, len(self.data), 5):
        #     self.obstacles_data.append(LidarObject(*self.data[i:i+5]))

    @property
    def new_data(self):
        return not self.output_queue.empty()
