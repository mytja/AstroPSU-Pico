/*
 * Copyright (c) 2021-2022 Antonio González
 * Copyright (c) 2025 Mitja Ševerkar
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"
#include "hardware/i2c.h"
#include "registers.h"

// BMI160 I2C address (SA0 pin low)
#define BMI160_ADDR 0x68

// Register addresses
#define BMI160_REG_CHIP_ID  0x00
#define BMI160_REG_PMU_STAT 0x03
#define BMI160_REG_GYR_DATA 0x0C
#define BMI160_REG_ACC_DATA 0x12
#define BMI160_REG_CMD      0x7E
#define BMI160_REG_ACC_CONF 0x40
#define BMI160_REG_ACC_RANGE 0x41
#define BMI160_REG_GYR_CONF 0x42
#define BMI160_REG_GYR_RANGE 0x43


// Commands
#define BMI160_CMD_GYR_NORMAL_MODE 0x15
#define BMI160_CMD_MAG_NORMAL_MODE 0x19
#define BMI160_CMD_ACCEL_NORMAL_MODE 0x11

// Gyroscope range settings
#define BMI160_GYR_RANGE_2000_DPS 0x00

#define BMI160_I2C_ADDRESS_A 0x68
#define BMI160_I2C_ADDRESS_B 0x69

const int16_t MIN_INT16 = (1 << 15);

// I think gyro data can be negative numbers, so therefore not int instead of uint
typedef struct bmi160 {
    i2c_inst_t *i2c_port;
    uint8_t i2c_addr;
    int16_t gyro[3];
    int16_t accel[3];
} bmi160_t;

void bmi160_init(i2c_inst_t* i2c_port, uint8_t i2c_addr,
                 bmi160_t* bmi160);
int bmi160_setup(bmi160_t* bmi160);
void bmi160_read_data(bmi160_t* bmi160);
void bmi160_median(bmi160_t* bmi160);
void bmi160_trigger_error(bmi160_t* bmi160);
void bmi160_calculate_absolute_angle(bmi160_t* bmi160, float* angles);
