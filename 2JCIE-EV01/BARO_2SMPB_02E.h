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

#ifndef BARO_2SMPB_02E_H
#define BARO_2SMPB_02E_H

#include "mbed.h"

/** Digital Barometric Pressure Sensor [2SMPB-02E] class
 *
 * @note Synchronization level: Thread safe
 *
 * Example of how to get data from this sensor:
 * @code
 *
 * #include "mbed.h"
 * #include "BARO_2SMPB_02E.h"
 *
 * BARO_2SMPB_02E baro_2smpb(I2C_SDA, I2C_SCL);
 *
 * int main() {
 *     uint32_t pres, dp, dt;
 *     int16_t temp;
 * 
 *     baro_2smpb.setup();
 *     ThisThread::sleep_for(100);
 * 
 *     while(1) {
 *         baro_2smpb.read(&pres, &temp, &dp, &dt);
 *         printf("pressure    : %7.1f [Pa] (%08Xh)\r\n", pres / 10.0 , (unsigned int)dp);
 *         printf("temperature : %5.2f [degC] (%08Xh)\r\n", temp / 100.0, (unsigned int)dt);
 *         ThisThread::sleep_for(200);
 *     }
 * }
 * @endcode
 */
class BARO_2SMPB_02E
{
public:
    /** Create a sensor instance
     *  
     *  @param sda I2C data line pin
     *  @param scl I2C clock line pin
     *  @param hz The bus frequency in hertz
     */
    BARO_2SMPB_02E(PinName sda, PinName scl, int hz = 150000);

    /** Initialize a sensor device
     *
     *  Should be called once per lifetime of the object.
     *  Please wait about 100ms before calling the read function.
     *  @return true on success, false on failure
     */
    bool setup(void);

    /** Read the current data from sensor
     *
     *  @param pres  Hundredfold pressure value (pascal)
     *  @param temp  Hundredfold temperature value (degree Celsius)
     *  @param dp    Raw data of pressure value
     *  @param dt    Raw data of temperature value
     *  @return true on success, false on failure
     */
    bool read(uint32_t* pres, int16_t* temp, uint32_t* dp, uint32_t* dt);

private:
    typedef struct {
        /* Compensation Factor */
        double _A0, _A1, _A2;
        double _B00, _BT1, _BP1;
        double _B11, _BT2, _BP2;
        double _B12, _B21, _BP3;
    } baro_2smpb02e_setting_t;

    I2C mI2c_;
    int mAddr;
    baro_2smpb02e_setting_t setting;

    double conv16_dbl(double a, double s, uint8_t* buf);
    double conv20q4_dbl(uint8_t* buf, uint8_t ex);
    void output_compensation(uint32_t raw_temp_val, uint32_t raw_press_val,
        uint32_t* pres, int16_t* temp);
    int read_reg(uint8_t reg, uint8_t* pbuf, uint8_t len);
};

#endif
