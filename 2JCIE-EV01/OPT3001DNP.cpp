// SPDX-License-Identifier: MIT
/*
 * MIT License
 * Copyright (c) 2019, 2018 - present OMRON Corporation
 * Copyright (c) 2019 Renesas Electronics Corporation
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "OPT3001DNP.h"

#define OPT3001_ADDR  (0x45 << 1)

#define OPT3001_REG_RESULT          0x00
#define OPT3001_REG_CONFIG          0x01
#define OPT3001_REG_LOLIMIT         0x02
#define OPT3001_REG_HILIMIT         0x03
#define OPT3001_REG_MANUFACTUREID   0x7E
#define OPT3001_DEVICEID            0x7F
#define OPT3001_CMD_CONFIG_MSB      0xC6
#define OPT3001_CMD_CONFIG_LSB      0x10

#define conv8s_u16_be(b, n) \
    (uint16_t)(((uint16_t)b[n] << 8) | (uint16_t)b[n + 1])

// OPT3001DNP implementation
OPT3001DNP::OPT3001DNP(PinName sda, PinName scl, int hz) :
     mI2c_(sda, scl)
{
    mAddr = OPT3001_ADDR;
    mI2c_.frequency(hz);
}

bool OPT3001DNP::setup(void)
{
    int ret;
    uint8_t wk_buf[3];

    wk_buf[0] = OPT3001_REG_CONFIG;
    wk_buf[1] = OPT3001_CMD_CONFIG_MSB;
    wk_buf[2] = OPT3001_CMD_CONFIG_LSB;
    ret = mI2c_.write(mAddr, (char *)wk_buf, 3);
    if (ret != 0) {
        return false;
    }

    return true;
}

bool OPT3001DNP::read(uint32_t* light)
{
    int ret;
    uint8_t rbuf[2];
    uint16_t raw_data;

    ret = read_reg(OPT3001_REG_CONFIG, rbuf, sizeof(rbuf));
    if (ret != 0) {
        return false;
    }

    if ((rbuf[1] & 0x80) == 0) {
        return false;  // sensor is working...
    }

    ret = read_reg(OPT3001_REG_RESULT, rbuf, sizeof(rbuf));
    if (ret != 0) {
        return false;
    }

    raw_data = conv8s_u16_be(rbuf, 0);

    if (light != NULL) {
        *light = convert_lux_value_x100(raw_data);
    }

    return true;
}

uint32_t OPT3001DNP::convert_lux_value_x100(uint16_t value_raw)
{
    uint32_t value_converted;
    uint32_t lsb_size_x100;
    uint32_t data;

    // Convert the value to centi-percent RH
    lsb_size_x100 = 1 << ((value_raw >> 12) & 0x0F);
    data = value_raw & 0x0FFF;
    value_converted = lsb_size_x100 * data;

    return value_converted;
}

int OPT3001DNP::read_reg(uint8_t reg, uint8_t* pbuf, uint8_t len)
{
    int ret;

    mI2c_.lock();
    ret = mI2c_.write(mAddr, (char *)&reg, 1);
    if (ret == 0) {
        ret = mI2c_.read(mAddr, (char *)pbuf, len);
    }
    mI2c_.unlock();

    return ret;
}

