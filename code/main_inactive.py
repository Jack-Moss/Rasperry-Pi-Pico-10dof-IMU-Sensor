from Data_Buffer import *
import time


def main():
    # Create Data Buffer
    data_buffer = Buffer()
    # Startup ICM29048 and populate rolling buffer
    data_buffer.create_rolling_buffer()
    # run detection on the databuffer to check for acceleration
    # data_buffer.fall_detection()
    time.sleep(5)
    # use databufffer dump to file to create it
    data_buffer.dump_buffer_to_file()

    # turn off LED after successful run


main()
