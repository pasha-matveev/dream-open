import subprocess
import threading
import logging
from queue import Queue


class Lidar:
    def __init__(self, args):
        self.args = args
        self.lidar_path = './lidar/build/bin/main'

        self.proc = None
        self.output_thread = None
        self.output_queue = Queue()

    def start(self):
        command = [self.lidar_path, '1' if self.args.lidar else '0']

        self.proc = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,   # lines come back as strings
            bufsize=1    # line-buffered on Python side
        )

        self.output_thread = threading.Thread(
            target=self.output_loop, args=(self.proc, self.output_queue), daemon=True)
        self.output_thread.start()

    def output_loop(self, proc, output_queue):
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
    
    def new_data(self):
        return not self.output_queue.empty()
