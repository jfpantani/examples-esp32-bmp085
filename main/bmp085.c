/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "bmp085-simple-example";

// Configure I2C interface
#define I2C_MASTER_SCL_IO 19
#define I2C_MASTER_SDA_IO 18
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

// Sensor I2C address
#define BMP085_SENSOR_ADDR 0x77

// Calibration coefficients
#define BMP085_REGADDR_AC1 0xAA
#define BMP085_REGADDR_AC2 0xAC
#define BMP085_REGADDR_AC3 0xAE
#define BMP085_REGADDR_AC4 0xB0
#define BMP085_REGADDR_AC5 0xB2
#define BMP085_REGADDR_AC6 0xB4
#define BMP085_REGADDR_B1 0xB6
#define BMP085_REGADDR_B2 0xB8
#define BMP085_REGADDR_MB 0xBA
#define BMP085_REGADDR_MC 0xBC
#define BMP085_REGADDR_MD 0xBE

// Control register
#define BMP085_CONTROL 0xF4
#define BMP085_TEMPDATA 0xF6
#define BMP085_PRESSUREDATA 0xF6
#define BMP085_READTEMPCMD 0x2E
#define BMP085_READPRESSURECMD 0x34

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/

const uint8_t OSS = 1; /*!<  Default oversample */

/**
 * @brief Variables used to store calibration coeficient
 */
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;

/**
 * @brief b5 is filled in the temperature calculation and used for pressure too.
 */
int32_t b5;

/**
 * @brief Variables used to store temperature e pressure
 */
int32_t temperature;
int32_t pressure;

/**
 * @brief these for altitude conversions
 */
const float p0 = 101325; // Pressure at sea level (Pa)
float altitude;

// functions

/**
 * @brief Dectect device presence
 */
bool detect_device(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP085_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Device found at address: %02x", BMP085_SENSOR_ADDR);
        return true;
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGI(TAG, "Timeout error");
        return false;
    }
    else
    {
        ESP_LOGI(TAG, "Device not found");
        return false;
    }
}

int bmp085_register_read16(uint8_t reg_addr)
{
    uint8_t retbuf[2];

    i2c_master_write_read_device(I2C_MASTER_NUM, BMP085_SENSOR_ADDR, &reg_addr, 1, retbuf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return (int)retbuf[0] << 8 | retbuf[1];
}

/**
 * @brief Write byte to BMP085 sensor register
 */
static esp_err_t bmp085_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP085_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

static esp_err_t bmp085_register_write_up(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data + (OSS << 6)};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP085_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief read BMP085 sensor register
 */
int bmp085_register_read(uint8_t reg_addr)
{
    uint8_t read_buf[2];

    uint8_t write_buf = reg_addr;

    i2c_master_write_to_device(I2C_MASTER_NUM, BMP085_SENSOR_ADDR, &write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    i2c_master_read_from_device(I2C_MASTER_NUM, BMP085_SENSOR_ADDR, read_buf, sizeof(read_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return (int)read_buf[0] << 8 | read_buf[2];
}

/**
 * @brief read BMP085 sensor register uncompesated pressure:
 */
uint32_t bmp085_register_read_up(uint8_t reg_addr)
{
    uint8_t read_buf[3];

    uint8_t write_buf = reg_addr;

    i2c_master_write_to_device(I2C_MASTER_NUM, BMP085_SENSOR_ADDR, &write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    i2c_master_read_from_device(I2C_MASTER_NUM, BMP085_SENSOR_ADDR, read_buf, sizeof(read_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return (((unsigned long)read_buf[0] << 16) | ((unsigned long)read_buf[1] << 8) | (unsigned long)read_buf[2]) >> (8 - OSS);
}

/**
 * @brief Calculate true temperature
 */
int32_t calc_true_temperature(uint32_t ut)
{
    int32_t x1, x2;
   // int32_t tt;

   // ESP_LOGI(TAG, "Calculate true temperature");
    x1 = ((ut - ac6) * ac5) >> 15;
    x2 = (mc << 11) / (x1 + md);
    b5 = x1 + x2;
    ESP_LOGI(TAG, "X1 = %d", x1);
    ESP_LOGI(TAG, "X2 = %d", x2);
    ESP_LOGI(TAG, "B5 = %d", b5);

    return ((b5 + 8) >> 4);
}

/**
 * @brief Calculate true pressure
 */
int32_t calc_true_pressure(uint32_t up)
{
    int32_t x1, x2, x3, b3, b6, p;
    uint32_t b4, b7;

    b6 = b5 - 4000;

    // B3
    x1 = (b2 * (b6 * b6) >> 12) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t)ac1) * 4 + x3) << OSS) + 2) >> 2;

    // B4
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;

    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;

    b7 = ((unsigned long)(up - b3) * (50000 >> OSS));
    if (b7 < 0x80000000)
        p = (b7 << 1) / b4;
    else
        p = (b7 / b4) << 1;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    return p;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void bmp085_task(void *arg)
{

    static const char *BMP085_TASK_TAG = "BMP085_TASK";
    /**
     * @brief Variables used to store uncompensated values
     */
    uint32_t ut, up;

    ESP_ERROR_CHECK(i2c_master_init());

    ESP_LOGI(TAG, "I2C initialized successfully");

    if (detect_device() == true)
    {
        ESP_LOGI(TAG, "Read Calibration coefficients:");
        ac1 = bmp085_register_read16(BMP085_REGADDR_AC1);
        ESP_LOGI(TAG, "ac1 = %d", ac1);
        ac2 = bmp085_register_read16(BMP085_REGADDR_AC2);
        ESP_LOGI(TAG, "ac2 = %d", ac2);
        ac3 = bmp085_register_read16(BMP085_REGADDR_AC3);
        ESP_LOGI(TAG, "ac3 = %d", ac3);
        ac4 = bmp085_register_read16(BMP085_REGADDR_AC4);
        ESP_LOGI(TAG, "ac4 = %d", ac4);
        ac5 = bmp085_register_read16(BMP085_REGADDR_AC5);
        ESP_LOGI(TAG, "ac5 = %d", ac5);
        ac6 = bmp085_register_read16(BMP085_REGADDR_AC6);
        ESP_LOGI(TAG, "ac6 = %d", ac6);
        b1 = bmp085_register_read16(BMP085_REGADDR_B1);
        ESP_LOGI(TAG, "b1 = %d", b1);
        b2 = bmp085_register_read16(BMP085_REGADDR_B2);
        ESP_LOGI(TAG, "b2 = %d", b2);
        mb = bmp085_register_read16(BMP085_REGADDR_MB);
        ESP_LOGI(TAG, "mb = %d", mb);
        mc = bmp085_register_read16(BMP085_REGADDR_MC);
        ESP_LOGI(TAG, "mc = %d", mc);
        md = bmp085_register_read16(BMP085_REGADDR_MD);
        ESP_LOGI(TAG, "md = %d", md);
        ESP_LOGI(TAG, "------------------------------");

        ESP_LOGI(TAG, "Read uncompensated values:");
        // Read ut
        ESP_ERROR_CHECK(bmp085_register_write_byte(BMP085_CONTROL, BMP085_READTEMPCMD));
        vTaskDelay(50 / portTICK_PERIOD_MS);
        ut = bmp085_register_read(BMP085_TEMPDATA);
        ESP_LOGI(TAG, "UT = %d", ut);

        // Read up
        ESP_ERROR_CHECK(bmp085_register_write_up(BMP085_CONTROL, BMP085_READPRESSURECMD));
        vTaskDelay(50 / portTICK_PERIOD_MS);

        up = bmp085_register_read_up(BMP085_PRESSUREDATA);
        ESP_LOGI(TAG, "UP = %d", up);
        ESP_LOGI(TAG, "-----------------------------");

        ESP_LOGI(TAG, "Calculate compensated values:");
        temperature = calc_true_temperature(ut);
        ESP_LOGI(TAG, "True temperature = %d", temperature);
        pressure = calc_true_pressure(up);
        ESP_LOGI(TAG, "True pressure = %d", pressure);
        altitude = 44330 * (1 - pow((pressure / p0), 0.190295));
        ESP_LOGI(TAG, "Altitude = %0.1f", altitude);
        
    }
    else
    {
        ESP_LOGI(TAG, "NOT OK");
    }

    ESP_LOGI(TAG, "I2C unitialized successfully");
    vTaskDelete(NULL);
}

void app_main(void)
{

    xTaskCreatePinnedToCore(bmp085_task, "bmp085_task", 4096, NULL, 10, NULL, 1);
}
