/*
 * Copyright (c) 2021-2022 Antonio González
 * Copyright (c) 2025 Mitja Ševerkar
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "sht3x.h"

const uint16_t MAX_UINT16 = (1 << 16) - 1;

// Command to fetch data is (per datasheet):
// 0x E0 00
const uint8_t FETCH_DATA_COMMAND[2] = {0x24, 0x16};

void sht3x_init(i2c_inst_t* i2c_port, uint8_t i2c_addr,
                sht3x_t* sht3x) {
    sht3x->i2c_port = i2c_port;
    sht3x->i2c_addr = i2c_addr;
}

void sht3x_read_data(sht3x_t* sht3x) {
    // First two bytes are reserved for Temperature MSB & Temperature LSB
    // The next byte (third one) is reserved for the CRC checksum for temperature
    // Next two bytes (fourth and fifth one) are reserved for Humidity MSB & Humidity LSB
    // The last byte (sixth one) is reserved for the CRC checksum for humidity
    //
    // Due to this being a crude implementation, we will skip CRC checking.
    // That's generally not recommended in production.
    uint8_t dst[6];
    int err = i2c_write_timeout_us(sht3x->i2c_port, sht3x->i2c_addr, FETCH_DATA_COMMAND, 2, true, 5000);
    if(err <= 0) {
        sht3x->temperature = -200;
        sht3x->humidity = -1;
        return;
    }
    err = i2c_read_timeout_us(sht3x->i2c_port, sht3x->i2c_addr, dst, 6, false, 5000);
    if(err <= 0) {
        sht3x->temperature = -200;
        sht3x->humidity = -1;
        return;
    }

    // Make a 16-bit integer from 2 bytes of data
    uint16_t temp_raw = (dst[0] << 8) | dst[1];
    sht3x->temperature = -45.0 + 175.0 * ((double)temp_raw / (double)MAX_UINT16);

    uint16_t hum_raw = (dst[3] << 8) | dst[4];
    sht3x->humidity = 100.0 * ((double)hum_raw / (double)MAX_UINT16);
}
