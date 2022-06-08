from I2C_Icm29048 import *
import collections
import data_processing
import utime
import ujson


# Populate buffers at a certain speed - eg 100 every second or every 10 milliseconds
# add to buffer to maintain size
# remove from buffer to maintain size
# Dump buffer to file on detecting a fall

class Buffer(object):
    """The Buffer Object is configured internally, check speed is in milliseconds and
      dump delay is in iterations so (check_speed * dump_delay = delay in milliseconds)"""
    def __init__(self):
        self.buffer = collections.deque((), 500)
        self.icm20948 = ICM20948()
        self.check_speed = 8
        self.dump_delay = 200
        self.dumping = False
        self.climber_weight = 80.0
        self.fall_count = 0
        self.fall_data = []

    def create_rolling_buffer(self):
        """Creates, Runs and outputs to file to data.json."""
        self.prefill_buffer()
        self.run_buffer()
        self.overrun_buffer()
        self.dump_buffer_to_file()

    def prefill_buffer(self):
        """This Creates 300 entries in the deque data buffer"""
        while len(self.buffer) <= 299:
            utime.sleep_ms(self.check_speed)
            self.add_to_buffer(self.icm20948.icm20948_accel_read())

    def run_buffer(self):
        """This adds entries to the right side of the data buffer and removes from the left.
        This loop ends on detecting approximately 0G which would indicate a fall."""
        while self.dumping is False:
            utime.sleep_ms(self.check_speed)
            current_acceleration = self.icm20948.icm20948_accel_read()
            self.remove_from_buffer()
            self.add_to_buffer(current_acceleration)
            # Fall Detection here when free-fall detected eg approx 1 to 0 Meters a Second
            if current_acceleration < 1:
                self.dumping = True

    def overrun_buffer(self):
        """On starting this function uses the dump delay variable to count down as a spacer from the detection of 0G.
           This is to capture the whole fall rather than just the start"""
        while self.dump_delay > 0:
            utime.sleep_ms(self.check_speed)
            current_acceleration = self.icm20948.icm20948_accel_read()
            self.dump_delay -= 1
            self.remove_from_buffer()
            self.add_to_buffer(current_acceleration)

    def add_to_buffer(self, acceleration):
        """:param acceleration: in meters a second
        Timestamps are in Microseconds"""
        self.buffer.append([utime.ticks_us(), acceleration])

    def remove_from_buffer(self):
        """Removes the leftmost value from the current internal buffer"""
        self.buffer.popleft()

    def dump_buffer_to_file(self):
        """Reforms the currently held buffer into a dictionary object then dumps it into a file named data.json
        if the file does not exist it creates it.

        Also formats the metadata into a single output.
        """
        dump_list = []
        formatted_output = {}
        while len(self.buffer) > 0:
            dump_list.append(self.buffer.popleft())

        metadata = data_processing.run_calculations(dump_list, self.climber_weight)
        formatted_output["metadata"] = metadata
        formatted_output["raw fall data"] = dump_list

        with open('data.json', 'a') as outfile:
            ujson.dump(formatted_output, outfile)
            outfile.write("\n")

    def dump_buffer_to_list(self):
        """Formats the buffer from a deque into a list"""
        while len(self.buffer) > 0:
            self.fall_data.append(self.buffer.popleft())

    def fix_ms_times(self):
        first_entry = self.fall_data[0]
        first_entry_time = first_entry[0]
        for entry in self.fall_data:
            entry[0] -= first_entry_time

    def peak_acceleration(self):
        highest_value = 0
        for entry in self.fall_data:
            if entry[1] > highest_value:
                highest_value = entry[1]
        return highest_value

    def calculate_kn(self, climber_weight, max_speed):
        return climber_weight * max_speed


