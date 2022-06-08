from Data_Buffer import *
from machine import Pin
import time


def main():
    led = Pin(25, Pin.OUT)
    led.on()
    # Show Power on
    if check_power_type() == 1:
        print("On USB Connection")
    else:
        for i in range(5):
            led.toggle()
            time.sleep(1)
        # Start True Loop
        while True:
            led.on()
            data_buffer = Buffer()
            data_buffer.create_rolling_buffer()
            led.off()
            time.sleep(1)


def check_power_type():
    #Check the 5v VBUS senser as this will only trigger when on usb  to computer
    #This removes the need to nuke the device to stop main.py running
    vbus_sense_pin = Pin(24, Pin.IN)
    return vbus_sense_pin.value()

main()
