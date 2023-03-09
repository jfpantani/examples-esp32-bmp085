# BMP085 pressure sensor example
This example show now to use the bmp085 pressure sensor with esp32.
BMP085 is made by bosch sensortec and is capable of measuring atmosferic pressure between
300 ... 1100hPa (+9000M ... -500 above sea level) and deliver data via I2C interface. 
As a bonus, with this device you can measure temperature and calculate barometric altitude.
To write this example I use two other, arduino BMP085 example, that you can find on arduino directory 
and ESP-IDF I2C simple example used to connect with MPU9250 sensor, I found inside framework examples directory.

If you inquiring how does it work, watch this video:

https://www.coursera.org/lecture/pressure-force-motion-humidity-sensors/4-piezoresistive-pressure-sensors-Csj2F?utm_source=link&utm_medium=page_share&utm_content=vlp&utm_campaign=top_button

## How to use example

### Hardware Required

This example use a esp32 and BMP085.

### Setup the Hardware

Connect the BMP085 interface to the esp32 as follows.

No extra parts or primitives were used in this project.

Connection scheme:

```
  -------------------------------------------------------------------
  |    BMP085 Interface   | #define          | Default ESP Pin      |
  | ----------------------|------------------|----------------------|
  | SDA                   | n/a              | GPIO18               |
  | SCL                   | n/a              | GPIO19               |
  | 3V                    | n/a              | 3V                   |
  | Ground                | n/a              | GND                  |
  -------------------------------------------------------------------
```

