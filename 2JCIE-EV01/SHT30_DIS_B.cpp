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

#include "SHT30_DIS_B.h"

#define SHT30_ADDR  (0x44 << 1)
#define SHT30_STATUSMASK  0xFC1F

#define GPIO_LED_R_PIN 4
#define GPIO_LED_G_PIN 5
#define GPIO_LED_B_PIN 6

#define conv16_u8_h(a) (uint8_t)(a >> 8)
#define conv16_u8_l(a) (uint8_t)(a & 0xFF)

#define conv8s_u16_be(b, n) \
    (uint16_t)(((uint16_t)b[n] << 8) | (uint16_t)b[n + 1])

#define SHT30_READSTATUS           0xF32D
#define SHT30_CLEARSTATUS          0x3041
#define SHT30_SOFTRESET            0x30A2
#define SHT30_HEATEREN             0x306D
#define SHT30_HEATERDIS            0x3066

#define SHT30_MEAS_HIGHPRD         0x2334
#define SHT30_READ_PERIODIC        0xE000

// SHT30_DIS_B implementation
SHT30_DIS_B::SHT30_DIS_B(PinName sda, PinName scl, int hz) :
     mI2c_(sda, scl)
{
    mAddr = SHT30_ADDR;
    mI2c_.frequency(hz);
}

bool SHT30_DIS_B::setup(void)
{
    int ret;
    uint8_t wk_buf[3];

    // reset
    wk_buf[0] = (uint8_t)(SHT30_SOFTRESET >> 8);
    wk_buf[1] = (uint8_t)(SHT30_SOFTRESET);
    ret = mI2c_.write(mAddr, (char *)wk_buf, 2);
    if (ret != 0) {
        return false;
    }
    ThisThread::sleep_for(10);

    // startcheck
    wk_buf[0] = (uint8_t)(SHT30_CLEARSTATUS >> 8);
    wk_buf[1] = (uint8_t)(SHT30_CLEARSTATUS);
    ret = mI2c_.write(mAddr, (char *)wk_buf, 2);
    if (ret != 0) {
        return false;
    }

    int retry = 10;

    while (retry > 0) {
        if (read_reg16(SHT30_READSTATUS, wk_buf, 3) == 0) {
            uint16_t stat = conv8s_u16_be(wk_buf, 0);
            if ((stat & SHT30_STATUSMASK) == 0x0000) {
                break;
            }
        }
        retry--;
        ThisThread::sleep_for(10);
    }
    if (retry <= 0) {
        return false;
    }

    // start measurement
    wk_buf[0] = (uint8_t)(SHT30_MEAS_HIGHPRD >> 8);
    wk_buf[1] = (uint8_t)(SHT30_MEAS_HIGHPRD);
    ret = mI2c_.write(mAddr, (char *)wk_buf, 2);
    if (ret != 0) {
        return false;
    }

    return true;
}

bool SHT30_DIS_B::read(int32_t* humi, int32_t* temp)
{
    int ret;
    uint8_t wk_buf[6];

    ret = read_reg16(SHT30_READ_PERIODIC, wk_buf, 6);
    if (ret != 0) {
        return false;
    }

    uint16_t ST, SRH;
    ST = conv8s_u16_be(wk_buf, 0);
    SRH = conv8s_u16_be(wk_buf, 3);

    double stemp = (double)ST * 17500.0 / 65535.0 - 4500.0;
    *temp = (int32_t)stemp;

    double shum = (double)SRH * 10000.0 / 65535.0;
    *humi = (int32_t)shum;

    return true;
}

int SHT30_DIS_B::read_reg16(uint16_t reg, uint8_t* pbuf, uint8_t len)
{
    int ret;
    char wk_buf[2];

    wk_buf[0] = (char)(reg >> 8);
    wk_buf[1] = (char)reg;

    mI2c_.lock();
    ret = mI2c_.write(mAddr, (char *)wk_buf, 2);
    if (ret == 0) {
        ret = mI2c_.read(mAddr, (char *)pbuf, len);
    }
    mI2c_.unlock();

    return ret;
}

