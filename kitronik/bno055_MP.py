from machine import Pin, I2C
from bno055 import *
import time

# Configura I2C con SDA=GPI6 pin9 e SCL=GPIO7 pin10
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)

imu = BNO055(i2c)

calibrated = False
while True:
    time.sleep(1)
    if not calibrated:
        calibrated = imu.calibrated()
        print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
    print('Temperature {}°C'.format(imu.temperature()))
    print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
    print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
    print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
    print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
    print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
    print('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))