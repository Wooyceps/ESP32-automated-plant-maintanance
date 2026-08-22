#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "SENSORS";

// Wspolna magistrala I2C: SDA -> D4 (GPIO22) / SCL -> D5 (GPIO23)
#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_GPIO   22
#define I2C_SCL_GPIO   23

#define VEML7700_ADDR  0x10
#define BMP280_ADDR    0x76   // SDO -> VDD = 0x77, SDO -> GND = 0x76

static void i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

static void veml7700_init(void) {
    // rejestr ALS_CONF_0 = 0x0000 -> gain x1, integration time 100ms, power on
    uint8_t data[3] = { 0x00, 0x00, 0x00 };
    i2c_master_write_to_device(I2C_PORT, VEML7700_ADDR, data, sizeof(data), pdMS_TO_TICKS(100));
}

static float veml7700_read_lux(void) {
    uint8_t reg = 0x04; // rejestr ALS
    uint8_t data[2] = {0};
    esp_err_t err = i2c_master_write_read_device(I2C_PORT, VEML7700_ADDR, &reg, 1, data, sizeof(data), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VEML7700 (0x%02X) odczyt nieudany: %s", VEML7700_ADDR, esp_err_to_name(err));
        return 0.0f;
    }

    uint16_t raw = (data[1] << 8) | data[0];
    return raw * 0.0576f; // wspolczynnik dla gain x1, 100ms (z datasheetu Vishay)
}

// --- BMP280: kalibracja i kompensacja wg datasheetu Bosch ---
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_calib_t;

static bmp280_calib_t calib;

static void bmp280_init(void) {
    uint8_t buf[24];
    uint8_t reg = 0x88; // start kalibracji
    i2c_master_write_read_device(I2C_PORT, BMP280_ADDR, &reg, 1, buf, sizeof(buf), pdMS_TO_TICKS(100));

    calib.dig_T1 = (buf[1] << 8) | buf[0];
    calib.dig_T2 = (buf[3] << 8) | buf[2];
    calib.dig_T3 = (buf[5] << 8) | buf[4];
    calib.dig_P1 = (buf[7] << 8) | buf[6];
    calib.dig_P2 = (buf[9] << 8) | buf[8];
    calib.dig_P3 = (buf[11] << 8) | buf[10];
    calib.dig_P4 = (buf[13] << 8) | buf[12];
    calib.dig_P5 = (buf[15] << 8) | buf[14];
    calib.dig_P6 = (buf[17] << 8) | buf[16];
    calib.dig_P7 = (buf[19] << 8) | buf[18];
    calib.dig_P8 = (buf[21] << 8) | buf[20];
    calib.dig_P9 = (buf[23] << 8) | buf[22];

    uint8_t ctrl_meas[2] = { 0xF4, 0x27 }; // osrs_t=1, osrs_p=1, mode=normal
    i2c_master_write_to_device(I2C_PORT, BMP280_ADDR, ctrl_meas, sizeof(ctrl_meas), pdMS_TO_TICKS(100));
}

static void bmp280_read(float *temperature_c, float *pressure_hpa) {
    uint8_t reg = 0xF7; // start danych: cisnienie + temperatura
    uint8_t data[6] = {0};
    esp_err_t err = i2c_master_write_read_device(I2C_PORT, BMP280_ADDR, &reg, 1, data, sizeof(data), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 (0x%02X) odczyt nieudany: %s", BMP280_ADDR, esp_err_to_name(err));
        *temperature_c = 0.0f;
        *pressure_hpa = 0.0f;
        return;
    }

    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * (int32_t)calib.dig_T2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)calib.dig_T1) * ((adc_T >> 4) - (int32_t)calib.dig_T1)) >> 12) * (int32_t)calib.dig_T3) >> 14;
    int32_t t_fine = var1 + var2;
    *temperature_c = ((t_fine * 5 + 128) >> 8) / 100.0f;

    int64_t p1 = (int64_t)t_fine - 128000;
    int64_t p2 = p1 * p1 * (int64_t)calib.dig_P6;
    p2 += (p1 * (int64_t)calib.dig_P5) << 17;
    p2 += ((int64_t)calib.dig_P4) << 35;
    p1 = ((p1 * p1 * (int64_t)calib.dig_P3) >> 8) + ((p1 * (int64_t)calib.dig_P2) << 12);
    p1 = (((((int64_t)1) << 47) + p1) * (int64_t)calib.dig_P1) >> 33;

    if (p1 == 0) {
        *pressure_hpa = 0;
        return;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - p2) * 3125) / p1;
    p1 = ((int64_t)calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    p2 = ((int64_t)calib.dig_P8 * p) >> 19;
    p = ((p + p1 + p2) >> 8) + (((int64_t)calib.dig_P7) << 4);

    *pressure_hpa = (p / 256.0f) / 100.0f;
}

void app_main(void) {
    i2c_init();
    veml7700_init();
    bmp280_init();

    while (1) {
        float lux = veml7700_read_lux();
        float temperature_c, pressure_hpa;
        bmp280_read(&temperature_c, &pressure_hpa);

        ESP_LOGI(TAG, "Temperatura: %.2f C   |   Cisnienie: %.2f hPa   |   Naswietlenie: %.2f lux",
                 temperature_c, pressure_hpa, lux);

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
