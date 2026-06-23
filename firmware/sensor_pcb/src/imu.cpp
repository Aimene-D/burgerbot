#include "imu.h"
#include "config.h"
#include <Arduino.h>
#include <Adafruit_LSM6DS3.h>
#include <Wire.h>
#include <esp_task_wdt.h>

static Adafruit_LSM6DS3 s_lsm6ds3;
static bool             s_imu_present = false;
static uint8_t          s_lsm6ds3_i2c_addr = LSM6DS3_I2C_ADDR_PRIMARY;

static float s_gyro_bias_x = 0.0f;
static float s_gyro_bias_y = 0.0f;
static float s_gyro_bias_z = 0.0f;

static bool probeI2cAddress(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

bool initImu() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    delay(80);

    const bool imu_primary_ok = probeI2cAddress(LSM6DS3_I2C_ADDR_PRIMARY);
    const bool imu_alt_ok     = probeI2cAddress(LSM6DS3_I2C_ADDR_ALT);

    if (imu_primary_ok) s_lsm6ds3_i2c_addr = LSM6DS3_I2C_ADDR_PRIMARY;
    else if (imu_alt_ok) s_lsm6ds3_i2c_addr = LSM6DS3_I2C_ADDR_ALT;
    else return false;

    s_imu_present = s_lsm6ds3.begin_I2C(s_lsm6ds3_i2c_addr, &Wire);
    if (!s_imu_present) return false;

    return true;
}

bool readImu(float& ax, float& ay, float& az,
             float& gx, float& gy, float& gz) {
    if (!s_imu_present) return false;

    sensors_event_t accel, gyro, temp;
    if (!s_lsm6ds3.getEvent(&accel, &gyro, &temp)) {
        return false;
    }

    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;

    gx = gyro.gyro.x - s_gyro_bias_x;
    gy = gyro.gyro.y - s_gyro_bias_y;
    gz = gyro.gyro.z - s_gyro_bias_z;

    return true;
}

void calibrateGyroBias() {
    if (!s_imu_present) {
        s_gyro_bias_x = s_gyro_bias_y = s_gyro_bias_z = 0.0f;
        return;
    }

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    uint32_t valid = 0;

    for (uint32_t i = 0; i < GYRO_BIAS_SAMPLES; i++) {
        sensors_event_t accel, gyro, temp;
        if (s_lsm6ds3.getEvent(&accel, &gyro, &temp)) {
            sum_x += gyro.gyro.x;
            sum_y += gyro.gyro.y;
            sum_z += gyro.gyro.z;
            valid++;
        }
        delay(static_cast<int>(GYRO_BIAS_SAMPLE_DELAY_MS));
        if (i % 100 == 0) {
            esp_task_wdt_reset();
        }
    }

    if (valid > 0) {
        s_gyro_bias_x = static_cast<float>(sum_x / valid);
        s_gyro_bias_y = static_cast<float>(sum_y / valid);
        s_gyro_bias_z = static_cast<float>(sum_z / valid);
    }
}

void getGyroBias(float& bx, float& by, float& bz) {
    bx = s_gyro_bias_x;
    by = s_gyro_bias_y;
    bz = s_gyro_bias_z;
}
