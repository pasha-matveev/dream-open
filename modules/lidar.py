import subprocess
import threading

class Lidar:
    def __init__(self, args):
        self.args = args
        self.lidar_path = './lidar/out/build/Clang 16.0.0 arm64-apple-darwin24.3.0/main'

        self.proc = None
        self.output_thread = None

        self.output_line = None
        self.error_line = None
        self.data = []

        self.angle = None
        self.dist = None

    
    def start(self):
        command = [self.lidar_path, '1' if self.args.lidar else '0']

        self.proc = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=1,  # line-buffered
            close_fds=True
        )

        self.output_thread = threading.Thread(target=self.output_loop, args=(self.proc,), daemon=True)
        self.output_thread.start()

    def output_loop(self, proc):
        for line in iter(proc.stdout.readline, b''):
            self.output_line = line.decode('utf-8', errors='replace').rstrip('\n')
        
        for err_line in iter(proc.stderr.readline, b''):
            self.error_line = err_line.decode('utf-8', errors='replace').rstrip('\n')
            print(f"[CPP ERROR]: {self.error_line}")

        self.data = self.output_line.split(' ')
        self.angle = float(self.data[0])
        self.dist = float(self.data[1])
        
    
    def stop(self):
        self.proc.terminate()
        self.proc.wait()
        self.output_thread.join()