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

![esp-lora image1](https://github.com/jfpantani/examples-esp32-bmp085/blob/main/image/bmp085.jpg?raw=true)

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
Note: Some GPIOs can not be used with certain chips because they are reserved for internal use. Please refer to UART documentation for selected target.

### Software Required

esp-idf-v4.4.1

### Setup the Software

Use the command below to set esp-idf target:
```
idf.py set-target esp32
```
Configure example if required:
```
idf.py menuconfig
```

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

```
I (0) cpu_start: App cpu up.
I (224) cpu_start: Pro cpu start user code
I (224) cpu_start: cpu freq: 160000000
I (224) cpu_start: Application information:
I (228) cpu_start: Project name:     bmp085
I (233) cpu_start: App version:      1
I (238) cpu_start: Compile time:     Mar  6 2023 21:46:54
I (244) cpu_start: ELF file SHA256:  b5475c4376784185...
I (250) cpu_start: ESP-IDF:          v4.4.1-dirty
I (255) heap_init: Initializing. RAM available for dynamic allocation:
I (262) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (268) heap_init: At 3FFB2CC8 len 0002D338 (180 KiB): DRAM
I (275) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (281) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (287) heap_init: At 4008C910 len 000136F0 (77 KiB): IRAM
I (295) spi_flash: detected chip: gd
I (298) spi_flash: flash io: dio
I (303) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (0) bmp085-simple-example: I2C initialized successfully
I (10) bmp085-simple-example: Device found at address: 77
I (10) bmp085-simple-example: Read Calibration coefficients:
I (20) bmp085-simple-example: ac1 = 6592
I (20) bmp085-simple-example: ac2 = -1066
I (30) bmp085-simple-example: ac3 = -14439
I (30) bmp085-simple-example: ac4 = 33011
I (40) bmp085-simple-example: ac5 = 25390
I (40) bmp085-simple-example: ac6 = 20259
I (50) bmp085-simple-example: b1 = 5498
I (50) bmp085-simple-example: b2 = 55
I (60) bmp085-simple-example: mb = -32768
I (60) bmp085-simple-example: mc = -11075
I (70) bmp085-simple-example: md = 2432
I (70) bmp085-simple-example: ------------------------------
I (80) bmp085-simple-example: Read uncompensated values:
I (130) bmp085-simple-example: UT = 29174
I (180) bmp085-simple-example: UP = 73660
I (180) bmp085-simple-example: -----------------------------
I (180) bmp085-simple-example: Calculate compensated values:
I (190) bmp085-simple-example: X1 = 6907
I (190) bmp085-simple-example: X2 = -2428
I (200) bmp085-simple-example: B5 = 4479
I (200) bmp085-simple-example: True temperature = 280
I (210) bmp085-simple-example: True pressure = 92344
I (210) bmp085-simple-example: Altitude = 776.1
I (220) bmp085-simple-example: I2C unitialized successfully
```
Line "I (10) bmp085-simple-example: Device found at address: 77" indicates that sensor was found, if not, verify your circuit connections.

## Troubleshooting

You are not supposed to see the echo in the terminal which is used for flashing and monitoring, but in the other UART configured through Kconfig can be used.
