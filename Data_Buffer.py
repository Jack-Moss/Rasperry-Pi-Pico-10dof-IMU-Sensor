import collections
import utime
from I2C_Icm29048 import *
import ujson
import machine


# Populate buffers at a certain speed - eg 100 every second or every 10 milliseconds
# add to buffer to maintain size
# remove from buffer to maintain size
# Dump buffer to file on detecting a fall

class Buffer(object):
    def __init__(self):
        self.buffer = collections.deque((), 500)
        self.check_speed = 10
        self.icm20948 = ICM20948()
        self.fall_count = 0

    def create_rolling_buffer(self):
        # Startup and light pin for led to show running
        print("Rolling Buffer initiated")
        led_25 = machine.Pin(25, machine.Pin.OUT)
        led_25.value(1)
        #
        while len(self.buffer) <= 499:
            self.add_to_buffer(self.icm20948.Accel[0], self.icm20948.Accel[1], self.icm20948.Accel[2])
            self.add_new_values()
        while len(self.buffer) >= 500:
            self.remove_from_buffer()
            self.add_new_values()
            self.add_to_buffer(self.icm20948.Accel[0], self.icm20948.Accel[1], self.icm20948.Accel[2])
            # TODO ADD DETECTION HERE
            break
        led_25.value(0)

    def add_to_buffer(self, x, y, z):
        self.buffer.append((x, y, z))

    def remove_from_buffer(self):
        self.buffer.popleft()

    def add_new_values(self):
        self.icm20948.accel_read()
        utime.sleep_ms(self.check_speed)

    def detect_fall(self):
        pass

    def dump_buffer_to_file(self):
        print("here")
        # dump_list = {"Fall Count": self.fall_count, "fall_data": []}
        while len(self.buffer) > 0:
            print(self.buffer.popleft())
        #    dump_list["fall_data"].append(self.buffer.popleft())
        #    with open('data.txt', 'w') as outfile:
        #        ujson.dump(dump_list, outfile)

# print(icm20948.roll, icm20948.pitch, icm20948.yaw)
# print(icm20948.Accel[0], icm20948.Accel[1], icm20948.Accel[2])
# print(icm20948.Gyro[0], icm20948.Gyro[1], icm20948.Gyro[2])
