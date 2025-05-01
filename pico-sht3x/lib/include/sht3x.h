/*
 * Copyright (c) 2021-2022 Antonio González
 * Copyright (c) 2025 Mitja Ševerkar
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"
#include "hardware/i2c.h"
#include "registers.h"

#define SHT3X_I2C_ADDRESS_A 0x44
#define SHT3X_I2C_ADDRESS_B 0x45

typedef struct sht3x {
    i2c_inst_t *i2c_port;
    uint8_t i2c_addr;
    double temperature;
    double humidity;
} sht3x_t;

/*! \brief Initialise the SHT3x device
 *
 * \param i2c_port The I2C instance, either i2c0 or i2c1
 * \param i2c_addr The I2C address of the SHT3x device
 * \param sht3x Pointer to the structure that stores the SHT3x info
 */
void sht3x_init(i2c_inst_t* i2c_port, uint8_t i2c_addr,
                sht3x_t* sht3x);

/*! \brief Read the last converted value
 *
 * \param sht3x Pointer to the structure that stores the SHT3x info
 */
void sht3x_read_data(sht3x_t* sht3x);
