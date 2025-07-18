#include "hardware/i2c.h"
#include "pico/timeout_helper.h"

#include "oldi2c.h"

/* Implement SDK 1.5.1 I2C function because it's broken in new SDK */
int _oldi2c_write_blocking_internal(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop,
                                    check_timeout_fn timeout_check, struct timeout_state *ts) {
    i2c->hw->enable = 0;
    i2c->hw->tar = addr;
    i2c->hw->enable = 1;

    bool abort = false;
    bool timeout = false;

    uint32_t abort_reason = 0;
    int byte_ctr;

    int ilen = (int)len;
    for(byte_ctr = 0; byte_ctr < ilen; ++byte_ctr) {
        bool first = byte_ctr == 0;
        bool last = byte_ctr == ilen - 1;

        i2c->hw->data_cmd =
            bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
            bool_to_bit(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB |
            *src++;

        // Wait until the transmission of the address/data from the internal
        // shift register has completed. For this to function correctly, the
        // TX_EMPTY_CTRL flag in IC_CON must be set. The TX_EMPTY_CTRL flag
        // was set in i2c_init.
        do {
            if(timeout_check) {
                timeout = timeout_check(ts, false);
                abort |= timeout;
            }
            tight_loop_contents();
        } while(!timeout && !(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS));

        // If there was a timeout, don't attempt to do anything else.
        if(!timeout) {
            abort_reason = i2c->hw->tx_abrt_source;
            if(abort_reason) {
                // Note clearing the abort flag also clears the reason, and
                // this instance of flag is clear-on-read! Note also the
                // IC_CLR_TX_ABRT register always reads as 0.
                i2c->hw->clr_tx_abrt;
                abort = true;
            }

            if(abort || (last && !nostop)) {
                // If the transaction was aborted or if it completed
                // successfully wait until the STOP condition has occured.

                // TODO Could there be an abort while waiting for the STOP
                // condition here? If so, additional code would be needed here
                // to take care of the abort.
                do {
                    if(timeout_check) {
                        timeout = timeout_check(ts, false);
                        abort |= timeout;
                    }
                    tight_loop_contents();
                } while(!timeout && !(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS));

                // If there was a timeout, don't attempt to do anything else.
                if(!timeout) {
                    i2c->hw->clr_stop_det;
                }
            }
        }

        // Note the hardware issues a STOP automatically on an abort condition.
        // Note also the hardware clears RX FIFO as well as TX on abort,
        // because we set hwparam IC_AVOID_RX_FIFO_FLUSH_ON_TX_ABRT to 0.
        if(abort)
            break;
    }

    int rval;

    // A lot of things could have just happened due to the ingenious and
    // creative design of I2C. Try to figure things out.
    if(abort) {
        if(timeout)
            rval = PICO_ERROR_TIMEOUT;
        else if(!abort_reason || abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS) {
            // No reported errors - seems to happen if there is nothing connected to the bus.
            // Address byte not acknowledged
            rval = PICO_ERROR_GENERIC;
        } else if(abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS) {
            // Address acknowledged, some data not acknowledged
            rval = byte_ctr;
        } else {
            // panic("Unknown abort from I2C instance @%08x: %08x\n", (uint32_t) i2c->hw, abort_reason);
            rval = PICO_ERROR_GENERIC;
        }
    } else {
        rval = byte_ctr;
    }

    // nostop means we are now at the end of a *message* but not the end of a *transfer*
    i2c->restart_on_next = nostop;
    return rval;
}

static int _oldi2c_read_blocking_internal(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop,
                                          check_timeout_fn timeout_check, timeout_state_t *ts) {
    i2c->hw->enable = 0;
    i2c->hw->tar = addr;
    i2c->hw->enable = 1;

    bool abort = false;
    bool timeout = false;
    uint32_t abort_reason;
    int byte_ctr;
    int ilen = (int)len;
    for(byte_ctr = 0; byte_ctr < ilen; ++byte_ctr) {
        bool first = byte_ctr == 0;
        bool last = byte_ctr == ilen - 1;
        while(!i2c_get_write_available(i2c))
            tight_loop_contents();

        i2c->hw->data_cmd =
            bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
            bool_to_bit(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB |
            I2C_IC_DATA_CMD_CMD_BITS; // -> 1 for read

        do {
            abort_reason = i2c->hw->tx_abrt_source;
            abort = (bool)i2c->hw->clr_tx_abrt;
            if(timeout_check) {
                timeout = timeout_check(ts, false);
                abort |= timeout;
            }
        } while(!abort && !i2c_get_read_available(i2c));

        if(abort)
            break;

        *dst++ = (uint8_t)i2c->hw->data_cmd;
    }

    int rval;

    if(abort) {
        if(timeout)
            rval = PICO_ERROR_TIMEOUT;
        else if(!abort_reason || abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS) {
            // No reported errors - seems to happen if there is nothing connected to the bus.
            // Address byte not acknowledged
            rval = PICO_ERROR_GENERIC;
        } else {
            //            panic("Unknown abort from I2C instance @%08x: %08x\n", (uint32_t) i2c->hw, abort_reason);
            rval = PICO_ERROR_GENERIC;
        }
    } else {
        rval = byte_ctr;
    }

    i2c->restart_on_next = nostop;
    return rval;
}

int oldi2c_read_blocking_until(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop, absolute_time_t until) {
    timeout_state_t ts;
    return _oldi2c_read_blocking_internal(i2c, addr, dst, len, nostop, init_single_timeout_until(&ts, until), &ts);
}

int oldi2c_read_timeout_us(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop, uint timeout_us) {
    absolute_time_t t = make_timeout_time_us(timeout_us);
    return oldi2c_read_blocking_until(i2c, addr, dst, len, nostop, t);
}

int oldi2c_write_blocking_until(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop,
                                absolute_time_t until) {
    timeout_state_t ts;
    return _oldi2c_write_blocking_internal(i2c, addr, src, len, nostop, init_single_timeout_until(&ts, until), &ts);
}

int oldi2c_write_timeout_us(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop, uint timeout_us) {
    absolute_time_t t = make_timeout_time_us(timeout_us);
    return oldi2c_write_blocking_until(i2c, addr, src, len, nostop, t);
}