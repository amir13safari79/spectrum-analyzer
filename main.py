import serial
import numpy as np
import serial.tools.list_ports as port_list
from numpy.fft import fft, ifft, fftshift
from matplotlib import pyplot as plt

print('list of available ports:')
ports = list(port_list.comports())
for p in ports:
    print(p)

# configure serial port:
ser = serial.Serial()
ser.baudrate = 9600
ser.port = 'COM3'
ser.timeout = 2
ser.bytesize = 8
ser.stopbits = serial.STOPBITS_ONE


# get data
ser.open()
ser.reset_input_buffer()
ser.reset_output_buffer()
data = ser.read(1000)
ser.close()

data = hex(int.from_bytes(data, "little"))
data = str(data)
d_tmp = int('0x'+ data[2:4],0)
if d_tmp > 15 and d_tmp < 50:
    data = data[0:2] + data[3:len(data)] 

data_int = []
for i in range(2, len(data), 2):
    first = data[i]
    try:
        second = data[i+1]
    except:
        second = ''
    hex_d = '0x'+first+second
    data_int.append(int(hex_d, 0))

# find and plot fft of data
data_int = np.array(data_int)
fft_data_int = fftshift(fft(data_int))
fft_data_int = fft_data_int / np.amax(fft_data_int)
if(len(data) == 2000):
    k = np.arange(-500, 499)
else:
    k = np.arange(-500, 500)

treshhold = 50
T = 1
duty = np.where(data_int > 50)[0].shape[0] / data_int.shape[0]

print(len(data))
print(data_int)
plt.plot(k, abs(fft_data_int))
plt.title('Frequency spectrum of the input signal')
plt.ylabel('Normalized Amplitude')
plt.xlabel('k')
plt.show()

