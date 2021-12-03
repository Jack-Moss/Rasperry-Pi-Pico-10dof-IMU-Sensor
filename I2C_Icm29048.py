from machine import I2C
from constants import *
import time
import math

Accel = [0, 0, 0]


def ratio_to_ms(value):
    # g = 9.81
    # output_avg = 2016
    ratio_value = 0.00486607142
    return ratio_value * value


class ICM20948(object):
    def __init__(self, address=I2C_ADD_ICM20948):
        self._address = address
        self._bus = I2C(1)
        time.sleep(0.5)
        # user bank 0 register
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)
        self._write_byte(REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET)
        time.sleep(0.1)
        self._write_byte(REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE)
        # user bank 2 register
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2)
        self._write_byte(REG_ADD_ACCEL_SMPLRT_DIV_2, 0x07)
        self._write_byte(REG_ADD_ACCEL_CONFIG,
                         REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_16g | REG_VAL_BIT_ACCEL_DLPF)
        # user bank 0 register
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)
        time.sleep(0.1)

    def icm20948_accel_read(self):
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)
        data = self._read_block(REG_ADD_ACCEL_XOUT_H, 12)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2)
        Accel[0] = (data[0] << 8) | data[1]
        Accel[1] = (data[2] << 8) | data[3]
        Accel[2] = (data[4] << 8) | data[5]
        if Accel[0] >= 32767:  # deal with the overflow issues
            Accel[0] = Accel[0] - 65535
        elif Accel[0] <= -32767:
            Accel[0] = Accel[0] + 65535
        if Accel[1] >= 32767:
            Accel[1] = Accel[1] - 65535
        elif Accel[1] <= -32767:
            Accel[1] = Accel[1] + 65535
        if Accel[2] >= 32767:
            Accel[2] = Accel[2] - 65535
        elif Accel[2] <= -32767:
            Accel[2] = Accel[2] + 65535
        total_acceleration = math.sqrt(Accel[0] ** 2 + Accel[1] ** 2 + Accel[2] ** 2)
        return ratio_to_ms(total_acceleration)

    def _read_byte(self, cmd):
        rec = self._bus.readfrom_mem(int(self._address), int(cmd), 1)
        return rec[0]

    def _read_block(self, reg, length=1):
        rec = self._bus.readfrom_mem(int(self._address), int(reg), length)
        return rec

    def _read_u16(self, cmd):
        lsb = self._bus.readfrom_mem(int(self._address), int(cmd), 1)
        msb = self._bus.readfrom_mem(int(self._address), int(cmd) + 1, 1)
        return (msb[0] << 8) + lsb[0]

    def _write_byte(self, cmd, val):
        self._bus.writeto_mem(int(self._address), int(cmd), bytes([int(val)]))
        time.sleep(0.0001)
