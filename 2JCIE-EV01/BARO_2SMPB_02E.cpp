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

#include "BARO_2SMPB_02E.h"

#define BARO_2SMPB02E_ADDRESS     (0x56 << 1)

#define BARO_2SMPB02E_CHIP_ID     0x5C

#define conv8s_s24_be(a, b, c) \
        (int32_t)((((uint32_t)a << 16) & 0x00FF0000) | \
                  (((uint32_t)b << 8) & 0x0000FF00) | \
                   ((uint32_t)c & 0x000000FF))

#define BARO_2SMPB02E_REGI2C_PRES_TXD2               0xF7
#define BARO_2SMPB02E_REGI2C_IO_SETUP                0xF5
#define BARO_2SMPB02E_REGI2C_CTRL_MEAS               0xF4
#define BARO_2SMPB02E_REGI2C_IIR                     0xF1
#define BARO_2SMPB02E_REGI2C_CHIP_ID                 0xD1
#define BARO_2SMPB02E_REGI2C_COEFS                   0xA0

/* Register values */
#define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_0001MS       ((uint8_t)0x00)
#define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_0125MS       ((uint8_t)0x20)
#define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_0250MS       ((uint8_t)0x40)
#define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_0500MS       ((uint8_t)0x60)
#define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_1000MS       ((uint8_t)0x80)
#define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_2000MS       ((uint8_t)0xA0)
#define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_4000MS       ((uint8_t)0xC0)
#define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_8000MS       ((uint8_t)0xE0)

#define BARO_2SMPB02E_VAL_TEMPAVERAGE_01     ((uint8_t)0x20)
#define BARO_2SMPB02E_VAL_TEMPAVERAGE_02     ((uint8_t)0x40)
#define BARO_2SMPB02E_VAL_TEMPAVERAGE_04     ((uint8_t)0x60)

#define BARO_2SMPB02E_VAL_PRESAVERAGE_01     ((uint8_t)0x04)
#define BARO_2SMPB02E_VAL_PRESAVERAGE_02     ((uint8_t)0x08)
#define BARO_2SMPB02E_VAL_PRESAVERAGE_04     ((uint8_t)0x0C)
#define BARO_2SMPB02E_VAL_PRESAVERAGE_08     ((uint8_t)0x10)
#define BARO_2SMPB02E_VAL_PRESAVERAGE_16     ((uint8_t)0x14)
#define BARO_2SMPB02E_VAL_PRESAVERAGE_32     ((uint8_t)0x18)

#define BARO_2SMPB02E_VAL_POWERMODE_SLEEP  ((uint8_t)0x00)
#define BARO_2SMPB02E_VAL_POWERMODE_FORCED ((uint8_t)0x01)
#define BARO_2SMPB02E_VAL_POWERMODE_NORMAL ((uint8_t)0x03)

#define BARO_2SMPB02E_VAL_IIR_OFF     ((uint8_t)0x00)
#define BARO_2SMPB02E_VAL_IIR_02TIMES ((uint8_t)0x01)
#define BARO_2SMPB02E_VAL_IIR_04TIMES ((uint8_t)0x02)
#define BARO_2SMPB02E_VAL_IIR_08TIMES ((uint8_t)0x03)
#define BARO_2SMPB02E_VAL_IIR_16TIMES ((uint8_t)0x04)
#define BARO_2SMPB02E_VAL_IIR_32TIMES ((uint8_t)0x05)

/* Coeff */
#define BARO_2SMPB02E_COEFF_S_A1   ((double)( 4.3E-04))
#define BARO_2SMPB02E_COEFF_A_A1   ((double)(-6.3E-03))
#define BARO_2SMPB02E_COEFF_S_A2   ((double)( 1.2E-10))
#define BARO_2SMPB02E_COEFF_A_A2   ((double)(-1.9E-11))
#define BARO_2SMPB02E_COEFF_S_BT1  ((double)( 9.1E-02))
#define BARO_2SMPB02E_COEFF_A_BT1  ((double)( 1.0E-01))
#define BARO_2SMPB02E_COEFF_S_BT2  ((double)( 1.2E-06))
#define BARO_2SMPB02E_COEFF_A_BT2  ((double)( 1.2E-08))
#define BARO_2SMPB02E_COEFF_S_BP1  ((double)( 1.9E-02))
#define BARO_2SMPB02E_COEFF_A_BP1  ((double)( 3.3E-02))
#define BARO_2SMPB02E_COEFF_S_B11  ((double)( 1.4E-07))
#define BARO_2SMPB02E_COEFF_A_B11  ((double)( 2.1E-07))
#define BARO_2SMPB02E_COEFF_S_BP2  ((double)( 3.5E-10))
#define BARO_2SMPB02E_COEFF_A_BP2  ((double)(-6.3E-10))
#define BARO_2SMPB02E_COEFF_S_B12  ((double)( 7.6E-13))
#define BARO_2SMPB02E_COEFF_A_B12  ((double)( 2.9E-13))
#define BARO_2SMPB02E_COEFF_S_B21  ((double)( 1.2E-14))
#define BARO_2SMPB02E_COEFF_A_B21  ((double)( 2.1E-15))
#define BARO_2SMPB02E_COEFF_S_BP3  ((double)( 7.9E-17))
#define BARO_2SMPB02E_COEFF_A_BP3  ((double)( 1.3E-16))

#define BARO_2SMPB02E_VAL_MEASMODE_HIGHSPEED \
    (BARO_2SMPB02E_VAL_PRESAVERAGE_02 | BARO_2SMPB02E_VAL_TEMPAVERAGE_01)
#define BARO_2SMPB02E_VAL_MEASMODE_LOWPOWER \
    (BARO_2SMPB02E_VAL_PRESAVERAGE_04 | BARO_2SMPB02E_VAL_TEMPAVERAGE_01)
#define BARO_2SMPB02E_VAL_MEASMODE_STANDARD \
    (BARO_2SMPB02E_VAL_PRESAVERAGE_08 | BARO_2SMPB02E_VAL_TEMPAVERAGE_01)
#define BARO_2SMPB02E_VAL_MEASMODE_HIGHACCURACY \
    (BARO_2SMPB02E_VAL_PRESAVERAGE_16 | BARO_2SMPB02E_VAL_TEMPAVERAGE_02)
#define BARO_2SMPB02E_VAL_MEASMODE_ULTRAHIGH \
    (BARO_2SMPB02E_VAL_PRESAVERAGE_32 | BARO_2SMPB02E_VAL_TEMPAVERAGE_04)

// BARO_2SMPB_02E implementation
BARO_2SMPB_02E::BARO_2SMPB_02E(PinName sda, PinName scl, int hz) :
     mI2c_(sda, scl)
{
    mAddr = BARO_2SMPB02E_ADDRESS;
    mI2c_.frequency(hz);
}

bool BARO_2SMPB_02E::setup(void)
{
    int ret;
    uint8_t wk_buf[32] = {0};

    // 1.
    ret = read_reg(BARO_2SMPB02E_REGI2C_CHIP_ID, wk_buf, 1);
    if ((ret != 0) || wk_buf[0] != BARO_2SMPB02E_CHIP_ID) {
        return false;
    }

    // 2.
    ret = read_reg(BARO_2SMPB02E_REGI2C_COEFS, wk_buf, 25);
    if (ret != 0) {
        return false;
    }

    // pressure parameters
    setting._B00 = conv20q4_dbl(&wk_buf[0], (wk_buf[24] & 0xf0) >> 4);
    setting._BT1 = conv16_dbl(BARO_2SMPB02E_COEFF_A_BT1, BARO_2SMPB02E_COEFF_S_BT1, &wk_buf[2]);
    setting._BT2 = conv16_dbl(BARO_2SMPB02E_COEFF_A_BT2, BARO_2SMPB02E_COEFF_S_BT2, &wk_buf[4]);
    setting._BP1 = conv16_dbl(BARO_2SMPB02E_COEFF_A_BP1, BARO_2SMPB02E_COEFF_S_BP1, &wk_buf[6]);
    setting._B11 = conv16_dbl(BARO_2SMPB02E_COEFF_A_B11, BARO_2SMPB02E_COEFF_S_B11, &wk_buf[8]);
    setting._BP2 = conv16_dbl(BARO_2SMPB02E_COEFF_A_BP2, BARO_2SMPB02E_COEFF_S_BP2, &wk_buf[10]);
    setting._B12 = conv16_dbl(BARO_2SMPB02E_COEFF_A_B12, BARO_2SMPB02E_COEFF_S_B12, &wk_buf[12]);
    setting._B21 = conv16_dbl(BARO_2SMPB02E_COEFF_A_B21, BARO_2SMPB02E_COEFF_S_B21, &wk_buf[14]);
    setting._BP3 = conv16_dbl(BARO_2SMPB02E_COEFF_A_BP3, BARO_2SMPB02E_COEFF_S_BP3, &wk_buf[16]);

    // temperature parameters
    setting._A0 = conv20q4_dbl(&wk_buf[18], (wk_buf[24] & 0x0f));
    setting._A1 = conv16_dbl(BARO_2SMPB02E_COEFF_A_A1, BARO_2SMPB02E_COEFF_S_A1, &wk_buf[20]);
    setting._A2 = conv16_dbl(BARO_2SMPB02E_COEFF_A_A2, BARO_2SMPB02E_COEFF_S_A2, &wk_buf[22]);

    // 3. setup a sensor at 125msec sampling and 32-IIR filter.
    wk_buf[0] = BARO_2SMPB02E_REGI2C_IO_SETUP;
    wk_buf[1] = BARO_2SMPB02E_VAL_IOSETUP_STANDBY_0125MS;
    ret = mI2c_.write(mAddr, (char *)wk_buf, 2);
    if (ret != 0) {
        return false;
    }

    wk_buf[0] = BARO_2SMPB02E_REGI2C_IIR;
    wk_buf[1] = BARO_2SMPB02E_VAL_IIR_32TIMES;
    ret = mI2c_.write(mAddr, (char *)wk_buf, 2);
    if (ret != 0) {
        return false;
    }

    // then, start to measurements.
    wk_buf[0] = BARO_2SMPB02E_REGI2C_CTRL_MEAS;
    wk_buf[1] = (uint8_t)(BARO_2SMPB02E_VAL_MEASMODE_ULTRAHIGH | BARO_2SMPB02E_VAL_POWERMODE_NORMAL);
    ret = mI2c_.write(mAddr, (char *)wk_buf, 2);
    if (ret != 0) {
        return false;
    }

    return true;
}

bool BARO_2SMPB_02E::read(uint32_t* pres, int16_t* temp, uint32_t* dp, uint32_t* dt)
{
    bool ret;
    uint8_t rbuf[6] = {0};
    uint32_t rawtemp, rawpres;

    ret = read_reg(BARO_2SMPB02E_REGI2C_PRES_TXD2, rbuf, sizeof(rbuf));
    if (ret != 0) {
        return false;
    }

    *dp = rawpres = conv8s_s24_be(rbuf[0], rbuf[1], rbuf[2]);
    *dt = rawtemp = conv8s_s24_be(rbuf[3], rbuf[4], rbuf[5]);
    output_compensation(rawtemp, rawpres, pres, temp);

    return true;
}

double BARO_2SMPB_02E::conv16_dbl(double a, double s, uint8_t* buf)
{
    uint16_t val;
    int16_t ret;

    val = (uint16_t)((uint16_t)(buf[0] << 8) | (uint16_t)buf[1]);
    if ((val & 0x8000) != 0) {
        ret = (int16_t)((int32_t)val - 0x10000);
    } else {
        ret = val;
    }
    return a + (double)ret * s / 32767.0;
}

double BARO_2SMPB_02E::conv20q4_dbl(uint8_t* buf, uint8_t ex)
{
    int32_t ret;
    uint32_t val;

    val = (uint32_t)((buf[0] << 12) | (buf[1] << 4) | ex);
    if ((val & 0x80000) != 0) {
        ret = (int32_t)val - 0x100000;
    } else {
        ret = val;
    }
    return (double)ret / 16.0;
}

void BARO_2SMPB_02E::output_compensation(uint32_t raw_temp_val, uint32_t raw_press_val,
    uint32_t* pres, int16_t* temp)
{
    double Tr, Po;
    double Dt, Dp;

    Dt = (int32_t)raw_temp_val - 0x800000;
    Dp = (int32_t)raw_press_val - 0x800000;

    // temperature compensation
    baro_2smpb02e_setting_t* c = &setting;
    Tr = c->_A0 + c->_A1 * Dt + c->_A2 * (Dt * Dt);

    // barometer compensation
    Po = c->_B00 + (c->_BT1 * Tr) + (c->_BP1 * Dp) +
         (c->_B11 * Tr * Dp) + c->_BT2 * (Tr * Tr) +
         (c->_BP2 * (Dp * Dp)) + (c->_B12 * Dp * (Tr * Tr)) +
         (c->_B21 * (Dp * Dp) * Tr) + (c->_BP3 * (Dp * Dp * Dp));

    *temp = (int16_t)(Tr / 2.56);     // x100degC
    *pres = (uint32_t)(Po * 10.0);    // x10Pa
}

int BARO_2SMPB_02E::read_reg(uint8_t reg, uint8_t* pbuf, uint8_t len)
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

