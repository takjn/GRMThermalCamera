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

#include "LIS2DW12.h"

#define SPI_CLK_SPEED       1000000
#define SPI_BITS            8
#define SPI_MODE            0

#define LIS2DW_REG_WHO_AM_I 0x0F
#define LIS2DW_REG_CTRL1    0x20
#define LIS2DW_REG_OUT_X_L  0x28

#define LIS2DW_VAL_WHO_AM_I 0x44

// LIS2DW12 implementation
LIS2DW12::LIS2DW12(PinName mosi, PinName miso, PinName sclk, PinName sse) :
     mspi_(mosi, miso, sclk), ssl_(sse)
{
    ssl_ = 1;
    mspi_.format(SPI_BITS, SPI_MODE);
    mspi_.frequency(SPI_CLK_SPEED);
}

bool LIS2DW12::setup(void)
{
    uint32_t retry = 100;
    uint8_t wk_buf[6] = {0};

    // Check connection
    while ((wk_buf[0] != LIS2DW_VAL_WHO_AM_I) && (retry > 0)) {
        read_reg(LIS2DW_REG_WHO_AM_I, wk_buf, 1);
        ThisThread::sleep_for(10);
        retry--;
    }
    if (retry <= 0) {
        return false;
    }

    // Normal config
    wk_buf[0] = 0x54;   // CTRL1: 100Hz, High-Performance
    wk_buf[1] = 0x06;   // CTRL2: IF_ADD_INC, I2C_DISABLE
    wk_buf[2] = 0x00;   // CTRL3:
    wk_buf[3] = 0x00;   // CTRL4_INT1_PAD_CTRL:
    wk_buf[4] = 0x00;   // CTRL5_INT2_PAD_CTRL:
    wk_buf[5] = 0x14;   // CTRL6: FS 4g
    write_reg(LIS2DW_REG_CTRL1, wk_buf, 6);

    return true;
}

bool LIS2DW12::read(int16_t* accl, uint8_t count)
{
    int32_t accsum[3] = {0, 0, 0};

    // get accel data (x,y,z) x N
    mspi_.lock();
    ssl_ = 0;
    (void)mspi_.write(LIS2DW_REG_OUT_X_L | 0x80);
    for (uint8_t i = 0; i < count; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            accsum[j] += ((int32_t)mspi_.write(0));
            accsum[j] += ((int32_t)mspi_.write(0) << 8);
        }
    }
    ssl_ = 1;
    mspi_.unlock();
    accl[0] = (int16_t)(accsum[0] / count);
    accl[1] = (int16_t)(accsum[1] / count);
    accl[2] = (int16_t)(accsum[2] / count);

    return true;
}

void LIS2DW12::write_reg(uint8_t reg, uint8_t* pbuf, uint8_t len)
{
    mspi_.lock();
    ssl_ = 0;
    mspi_.write(reg & 0x7F);
    for (uint8_t i = 0; i < len; i++) {
        mspi_.write(pbuf[i]);
    }
    ssl_ = 1;
    mspi_.unlock();
}

void LIS2DW12::read_reg(uint8_t reg, uint8_t* pbuf, uint8_t len)
{
    mspi_.lock();
    ssl_ = 0;
    (void)mspi_.write(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        pbuf[i] = mspi_.write(0);
    }
    ssl_ = 1;
    mspi_.unlock();
}

