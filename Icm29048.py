from machine import I2C
from constants import *
import utime
import math

#TODO - Refactor into more classes to improve readability

class ICM20948(object):
    def __init__(self, address=I2C_ADD_ICM20948):
        #Initalise basic motion values
        self.MotionVal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Gyro = [0, 0, 0]
        self.Accel = [0, 0, 0]
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        #Initalise offset values
        self.pu8data = [0, 0, 0, 0, 0, 0, 0, 0]
        self.U8tempX = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.U8tempY = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.U8tempZ = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.GyroOffset = [0, 0, 0]
        self.Ki = 1.0
        self.Kp = 4.50
        self.q0 = 1.0
        self.q1 = self.q2 = self.q3 = 0.0
        self.angles = [0.0, 0.0, 0.0]
        #Run Inital checks
        self._address = address
        self._bus = I2C(1)
        self.icm20948check()
        # Initialization of the device multiple times after power on will result in a return error
        utime.sleep_ms(500)  # We can skip this detection by delaying it by 500 milliseconds
        # user bank 0 register
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)
        self._write_byte(REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET)
        utime.sleep_ms(100)
        self._write_byte(REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE)
        # user bank 2 register
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2)
        self._write_byte(REG_ADD_GYRO_SMPLRT_DIV, 0x07)
        self._write_byte(REG_ADD_GYRO_CONFIG_1,
                         REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS | REG_VAL_BIT_GYRO_DLPF)
        self._write_byte(REG_ADD_ACCEL_SMPLRT_DIV_2, 0x07)
        self._write_byte(REG_ADD_ACCEL_CONFIG,
                         REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_8g | REG_VAL_BIT_ACCEL_DLPF)
        # user bank 0 register
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)
        utime.sleep_ms(100)
        self.icm20948gyro_offset()

    def icm20948_gyro_accel_read(self):
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)
        data = self._read_block(REG_ADD_ACCEL_XOUT_H, 12)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2)
        self.Accel[0] = (data[0] << 8) | data[1]
        self.Accel[1] = (data[2] << 8) | data[3]
        self.Accel[2] = (data[4] << 8) | data[5]
        self.Gyro[0] = ((data[6] << 8) | data[7]) - self.Gyro[0]
        self.Gyro[1] = ((data[8] << 8) | data[9]) - self.Gyro[1]
        self.Gyro[2] = ((data[10] << 8) | data[11]) - self.Gyro[2]
        if self.Accel[0] >= 32767:  # Solve the problem that Python shift will not overflow
            self.Accel[0] = self.Accel[0] - 65535
        elif self.Accel[0] <= -32767:
            self.Accel[0] = self.Accel[0] + 65535
        if self.Accel[1] >= 32767:
            self.Accel[1] = self.Accel[1] - 65535
        elif self.Accel[1] <= -32767:
            self.Accel[1] = self.Accel[1] + 65535
        if self.Accel[2] >= 32767:
            self.Accel[2] = self.Accel[2] - 65535
        elif self.Accel[2] <= -32767:
            self.Accel[2] = self.Accel[2] + 65535
        if self.Gyro[0] >= 32767:
            self.Gyro[0] = self.Gyro[0] - 65535
        elif self.Gyro[0] <= -32767:
            self.Gyro[0] = self.Gyro[0] + 65535
        if self.Gyro[1] >= 32767:
            self.Gyro[1] = self.Gyro[1] - 65535
        elif self.Gyro[1] <= -32767:
            self.Gyro[1] = self.Gyro[1] + 65535
        if self.Gyro[2] >= 32767:
            self.Gyro[2] = self.Gyro[2] - 65535
        elif self.Gyro[2] <= -32767:
            self.Gyro[2] = self.Gyro[2] + 65535

    def icm20948read_secondary(self, u8_i2c_address, u8_reg_address, u8_len):
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3)  # switch bank3
        self._write_byte(REG_ADD_I2C_SLV0_ADDR, u8_i2c_address)
        self._write_byte(REG_ADD_I2C_SLV0_REG, u8_reg_address)
        self._write_byte(REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | u8_len)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)  # switch bank0
        u8_temp = self._read_byte(REG_ADD_USER_CTRL)
        u8_temp |= REG_VAL_BIT_I2C_MST_EN
        self._write_byte(REG_ADD_USER_CTRL, u8_temp)
        utime.sleep_ms(10)
        u8_temp &= ~REG_VAL_BIT_I2C_MST_EN
        self._write_byte(REG_ADD_USER_CTRL, u8_temp)

        for i in range(0, u8_len):
            self.pu8data[i] = self._read_byte(REG_ADD_EXT_SENS_DATA_00 + i)

        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3)  # switch bank3

        u8_temp = self._read_byte(REG_ADD_I2C_SLV0_CTRL)
        u8_temp &= ~(REG_VAL_BIT_I2C_MST_EN & REG_VAL_BIT_MASK_LEN)
        self._write_byte(REG_ADD_I2C_SLV0_CTRL, u8_temp)

        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)  # switch bank0

    def icm20948write_secondary(self, u8i2c_address, u8_reg_address, u8data):
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3)  # switch bank3
        self._write_byte(REG_ADD_I2C_SLV1_ADDR, u8i2c_address)
        self._write_byte(REG_ADD_I2C_SLV1_REG, u8_reg_address)
        self._write_byte(REG_ADD_I2C_SLV1_DO, u8data)
        self._write_byte(REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN | 1)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)  # switch bank0

        u8_temp = self._read_byte(REG_ADD_USER_CTRL)
        u8_temp |= REG_VAL_BIT_I2C_MST_EN
        self._write_byte(REG_ADD_USER_CTRL, u8_temp)
        utime.sleep_ms(100)
        u8_temp &= ~REG_VAL_BIT_I2C_MST_EN
        self._write_byte(REG_ADD_USER_CTRL, u8_temp)

        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3)  # switch bank3

        u8_temp = self._read_byte(REG_ADD_I2C_SLV0_CTRL)
        u8_temp &= ~(REG_VAL_BIT_I2C_MST_EN & REG_VAL_BIT_MASK_LEN)
        self._write_byte(REG_ADD_I2C_SLV0_CTRL, u8_temp)

        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)  # switch bank0

    def icm20948gyro_offset(self):
        s32_temp_gx = 0
        s32_temp_gy = 0
        s32_temp_gz = 0
        for i in range(0, 32):
            self.icm20948_gyro_accel_read()
            s32_temp_gx += self.Gyro[0]
            s32_temp_gy += self.Gyro[1]
            s32_temp_gz += self.Gyro[2]
            utime.sleep_ms(10)
        self.GyroOffset[0] = s32_temp_gx >> 5
        self.GyroOffset[1] = s32_temp_gy >> 5
        self.GyroOffset[2] = s32_temp_gz >> 5

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
        utime.sleep_ms(1)

    def imu_ahrs_update(self):
        gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z = self.MotionVal[0] * 0.0175, self.MotionVal[1] * 0.0175, self.MotionVal[2] \
                                             * 0.0175, self.MotionVal[3], self.MotionVal[4], self.MotionVal[5], \
                                             self.MotionVal[6], self.MotionVal[7], self.MotionVal[8]
        ex_int = ey_int = ez_int = 0.0
        half_t = 0.024

        q0q0 = self.q0 * self.q0
        q0q1 = self.q0 * self.q1
        q0q2 = self.q0 * self.q2
        q0q3 = self.q0 * self.q3
        q1q1 = self.q1 * self.q1
        q1q2 = self.q1 * self.q2
        q1q3 = self.q1 * self.q3
        q2q2 = self.q2 * self.q2
        q2q3 = self.q2 * self.q3
        q3q3 = self.q3 * self.q3

        norm = float(1 / math.sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z))
        accel_x = accel_x * norm
        accel_y = accel_y * norm
        accel_z = accel_z * norm

        norm = float(1 / math.sqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z))
        mag_x = mag_x * norm
        mag_y = mag_y * norm
        mag_z = mag_z * norm

        # compute reference direction of flux
        hx = 2 * mag_x * (0.5 - q2q2 - q3q3) + 2 * mag_y * (q1q2 - q0q3) + 2 * mag_z * (q1q3 + q0q2)
        hy = 2 * mag_x * (q1q2 + q0q3) + 2 * mag_y * (0.5 - q1q1 - q3q3) + 2 * mag_z * (q2q3 - q0q1)
        hz = 2 * mag_x * (q1q3 - q0q2) + 2 * mag_y * (q2q3 + q0q1) + 2 * mag_z * (0.5 - q1q1 - q2q2)
        bx = math.sqrt((hx * hx) + (hy * hy))
        bz = hz

        # estimated direction of gravity and flux (v and w)
        vx = 2 * (q1q3 - q0q2)
        vy = 2 * (q0q1 + q2q3)
        vz = q0q0 - q1q1 - q2q2 + q3q3
        wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2)
        wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3)
        wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2)

        # error is sum of cross product between reference direction of fields and direction measured by sensors
        ex = (accel_y * vz - accel_z * vy) + (mag_y * wz - mag_z * wy)
        ey = (accel_z * vx - accel_x * vz) + (mag_z * wx - mag_x * wz)
        ez = (accel_x * vy - accel_y * vx) + (mag_x * wy - mag_y * wx)

        if ex != 0.0 and ey != 0.0 and ez != 0.0:
            ex_int = ex_int + ex * self.Ki * half_t
            ey_int = ey_int + ey * self.Ki * half_t
            ez_int = ez_int + ez * self.Ki * half_t

            gyro_x = gyro_x + self.Kp * ex + ex_int
            gyro_y = gyro_y + self.Kp * ey + ey_int
            gyro_z = gyro_z + self.Kp * ez + ez_int

        q0 = self.q0 + (-self.q1 * gyro_x - self.q2 * gyro_y - self.q3 * gyro_z) * half_t
        q1 = self.q1 + (q0 * gyro_x + self.q2 * gyro_z - self.q3 * gyro_y) * half_t
        q2 = self.q2 + (q0 * gyro_y - q1 * gyro_z + self.q3 * gyro_x) * half_t
        q3 = self.q3 + (q0 * gyro_z + q1 * gyro_y - q2 * gyro_x) * half_t

        norm = float(1 / math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3))
        self.q0 = q0 * norm
        self.q1 = q1 * norm
        self.q2 = q2 * norm
        self.q3 = q3 * norm

    def icm20948check(self):
        false = 0x00
        true = 0x01
        b_ret = false
        if REG_VAL_WIA == self._read_byte(REG_ADD_WIA):
            b_ret = true
        return b_ret

    def icm20948calculate_avg_value(self):
        self.MotionVal[0] = self.Gyro[0] / 32.8
        self.MotionVal[1] = self.Gyro[1] / 32.8
        self.MotionVal[2] = self.Gyro[2] / 32.8
        self.MotionVal[3] = self.Accel[0]
        self.MotionVal[4] = self.Accel[1]
        self.MotionVal[5] = self.Accel[2]
        self.MotionVal[6] = 1
        self.MotionVal[7] = 1
        self.MotionVal[8] = 1

    def calculate_pitch_roll_yaw(self):
        self.pitch = math.asin(-2 * self.q1 * self.q3 + 2 * self.q0 * self.q2) * 57.3
        self.roll = math.atan2(2 * self.q2 * self.q3 + 2 * self.q0 * self.q1,
                               -2 * self.q1 * self.q1 - 2 * self.q2 * self.q2 + 1) * 57.3
        self.yaw = math.atan2(-2 * self.q1 * self.q2 - 2 * self.q0 * self.q3,
                              2 * self.q2 * self.q2 + 2 * self.q3 * self.q3 - 1) * 57.3

# print(icm20948.roll, icm20948.pitch, icm20948.yaw)
# print(icm20948.Accel[0], icm20948.Accel[1], icm20948.Accel[2])
# print(icm20948.Gyro[0], icm20948.Gyro[1], icm20948.Gyro[2])