/*
 * Copyright (c) 2021-2022 Antonio González
 * Copyright (c) 2025 Mitja Ševerkar
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>

#include "oldi2c.h"

#include "bmi160.h"

const uint TIMEOUT_US = 30000;

void bmi160_write_byte(bmi160_t* bmi160, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    oldi2c_write_timeout_us(bmi160->i2c_port, bmi160->i2c_addr, buf, 2, false, TIMEOUT_US);
}

uint8_t bmi160_read_byte(bmi160_t* bmi160, uint8_t reg) {
    uint8_t value;
    oldi2c_write_timeout_us(bmi160->i2c_port, bmi160->i2c_addr, &reg, 1, true, TIMEOUT_US);
    oldi2c_read_timeout_us(bmi160->i2c_port, bmi160->i2c_addr, &value, 1, false, TIMEOUT_US);
    return value;
}

void bmi160_read_bytes(bmi160_t* bmi160, uint8_t reg, uint8_t *buf, uint8_t len) {
    oldi2c_write_timeout_us(bmi160->i2c_port, bmi160->i2c_addr, &reg, 1, true, TIMEOUT_US);
    oldi2c_read_timeout_us(bmi160->i2c_port, bmi160->i2c_addr, buf, len, false, TIMEOUT_US);
}

void bmi160_init(i2c_inst_t* i2c_port, uint8_t i2c_addr,
                 bmi160_t* bmi160) {
    bmi160->i2c_port = i2c_port;
    bmi160->i2c_addr = i2c_addr;
}

int bmi160_setup(bmi160_t* bmi160) {
    // Check chip ID
    uint8_t chip_id = bmi160_read_byte(bmi160, BMI160_REG_CHIP_ID);
    if(chip_id != 0xD1) {
        return -1;
    }

    bmi160_write_byte(bmi160, BMI160_REG_GYR_CONF, 0x28); // ODR=100Hz, filter=normal
    bmi160_write_byte(bmi160, BMI160_REG_GYR_RANGE, 0b0011);
    bmi160_write_byte(bmi160, BMI160_REG_ACC_CONF, 0x28); // ODR=100Hz, filter=normal
    bmi160_write_byte(bmi160, BMI160_REG_ACC_RANGE, 0b0011);

    // Switch gyroscope to normal mode
    bmi160_write_byte(bmi160, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL_MODE);
    bmi160_write_byte(bmi160, BMI160_REG_CMD, BMI160_CMD_ACCEL_NORMAL_MODE);
    return 0;
}

void bmi160_trigger_error(bmi160_t* bmi160) {
    bmi160->gyro[0] = MIN_INT16;
    bmi160->gyro[1] = MIN_INT16;
    bmi160->gyro[2] = MIN_INT16;
    bmi160->accel[0] = MIN_INT16;
    bmi160->accel[1] = MIN_INT16;
    bmi160->accel[2] = MIN_INT16;
    return;
}

void bmi160_read_data(bmi160_t* bmi160) {
    uint8_t buf[6];
    bmi160_read_bytes(bmi160, BMI160_REG_GYR_DATA, buf, 6);
    bmi160->gyro[0] = (int16_t)((buf[1] << 8) | buf[0]);
    bmi160->gyro[1] = (int16_t)((buf[3] << 8) | buf[2]);
    bmi160->gyro[2] = (int16_t)((buf[5] << 8) | buf[4]);

    bmi160_read_bytes(bmi160, BMI160_REG_ACC_DATA, buf, 6);
    bmi160->accel[0] = (int16_t)((buf[1] << 8) | buf[0]);
    bmi160->accel[1] = (int16_t)((buf[3] << 8) | buf[2]);
    bmi160->accel[2] = (int16_t)((buf[5] << 8) | buf[4]);

    //std::cout << "BMI on " << (int)bmi160->i2c_addr << " reporting dst: " << (int)buf[0] << " " << (int)buf[1] << " " << (int)buf[2] << std::endl;
}

const int repeat = 20;
int16_t x_gyro[repeat];
int16_t y_gyro[repeat];
int16_t z_gyro[repeat];
int16_t x_accel[repeat];
int16_t y_accel[repeat];
int16_t z_accel[repeat];

void bmi160_median(bmi160_t* bmi160) {

    //uint8_t pmu_status = bmi160_read_byte(bmi160, BMI160_REG_PMU_STAT);
    //printf("PMU: Gyro=%d Accel=%d\n", (pmu_status >> 2) & 0x3, pmu_status & 0x3);

    for(int i = 0; i < repeat; i++) {
        bmi160_read_data(bmi160);
        x_gyro[i] = bmi160->gyro[0];
        y_gyro[i] = bmi160->gyro[1];
        z_gyro[i] = bmi160->gyro[2];
        x_accel[i] = bmi160->accel[0];
        y_accel[i] = bmi160->accel[1];
        z_accel[i] = bmi160->accel[2];
    }
    int median = (repeat - 1) / 2;
    std::sort(x_gyro, x_gyro + repeat);
    std::sort(y_gyro, y_gyro + repeat);
    std::sort(z_gyro, z_gyro + repeat);
    std::sort(x_accel, x_accel + repeat);
    std::sort(y_accel, y_accel + repeat);
    std::sort(z_accel, z_accel + repeat);

    bmi160->gyro[0] = x_gyro[median];
    bmi160->gyro[1] = y_gyro[median];
    bmi160->gyro[2] = z_gyro[median];
    bmi160->accel[0] = x_accel[median];
    bmi160->accel[1] = y_accel[median];
    bmi160->accel[2] = z_accel[median];
}

// Constants - adjust based on your BMI160 configuration
const float RAD_TO_DEG = 57.2957795131f; // 180/π
const float ACCEL_SENSITIVITY = 16384.0f; // ±2g range

void bmi160_calculate_absolute_angle(bmi160_t* bmi160, float* angles) {
    // Convert raw data to G-forces
    float ax = bmi160->accel[0] / ACCEL_SENSITIVITY;
    float ay = bmi160->accel[1] / ACCEL_SENSITIVITY;
    float az = bmi160->accel[2] / ACCEL_SENSITIVITY;

    // Compute roll (rotation around X-axis)
    angles[0] = atan2f(ay, az) * RAD_TO_DEG;

    // Compute pitch (rotation around Y-axis)
    angles[1] = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}