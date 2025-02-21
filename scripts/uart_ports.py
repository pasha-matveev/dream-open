import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
for i, (port, desc, hwid) in enumerate(sorted(ports)):
    print("{}. {}: {} [{}]".format(i+1, port, desc, hwid))
    print(desc)