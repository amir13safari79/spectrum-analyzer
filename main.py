import serial
import serial.tools.list_ports as port_list


print('list of available ports:')
ports = list(port_list.comports())
for p in ports:
    print(p)

# configure serial port:
port = 'COM3'
baudrate = 9600
timeout = 2

ser = serial.Serial(Port=port, baudrate=baudrate, timeout=timeout, byteseize=8, stopbits=serial.STOPBITS_ONE)
ser.set_buffer_size(rx_size=4, tx_size=4)
print(ser.name)

ser.open()
s = ser.read(10)
s = ser.close()
