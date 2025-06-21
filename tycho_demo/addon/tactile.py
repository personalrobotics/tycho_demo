import time
import numpy as np
import serial


class TouchSensor():

    def __init__(self,):
        self.tear_value = 0
        self.clear = False
        self.connect_port()
        print('Initialized.')
    # Implementation details

    def connect_port(self):
        # Setup input
        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=2000000, timeout=1.0)
        assert self.ser.is_open, 'Failed to open COM port!'
        self.readPressure() #warm up

    def readPressure(self):
        # Request readout
        self.ser.reset_input_buffer() # Remove the confirmation 'w' sent by the sensor
        self.ser.write('a'.encode('utf-8')) # Request data from the sensor

        # Receive data
        w, h = 32, 32
        length = 2 * w * h
        input_string = self.ser.read(length)
        x = np.frombuffer(input_string, dtype=np.uint8).astype(np.uint16)
        if not len(input_string) == length:
            # self.log("Only got %d values => Drop frame." % len(input_string))
            return None

        x = x[0::2] * 32 + x[1::2]
        x = x.reshape(h, w).transpose(1, 0)
        # for each pressure value, remove the base value
        # x = x.astype(np.int16)
        if self.clear:
            self.tear_value = x.copy()
            self.clear = False

        x = x - self.tear_value
        # x = np.clip(x, 0, 2000)  # Adjust based on expected sensor range
        return x

    def _read(self):
        # time.sleep(0.01)
        pressure = self.readPressure()

        if pressure is None:
            return None

        # print(pressure[9:14,15:18])
        return pressure.copy()
        # return {'pressure': pressure}