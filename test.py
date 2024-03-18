import serial
import time
import serial.tools.list_ports
ports = list(serial.tools.list_ports.comports())

'''
for p in ports:
    print("======")
    print(p)
'''

arduino = serial.Serial(port='/dev/cu.usbmodem1101',   baudrate=19200, timeout=.1)

def write_read(x):
    arduino.write(bytes(x,   'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return   data


while True:
    num = input("Enter a number: ")
    value   = write_read(num)
    print(value)
