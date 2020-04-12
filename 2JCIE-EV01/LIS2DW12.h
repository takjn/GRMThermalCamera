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

#ifndef LIS2DW12_H
#define LIS2DW12_H

#include "mbed.h"

/** MEMS Digital Output Motion Sensor [LIS2DW12] class
 *
 * @note Synchronization level: Thread safe
 *
 * Example of how to get data from this sensor:
 * @code
 *
 * #include "mbed.h"
 * #include "LIS2DW12.h"
 *
 * LIS2DW12 lis2dw(SPI_MOSI, SPI_MISO, SPI_SCKL, SPI_SSL);
 * #define CONV_RAW_TO_MG(x) (int)((double)(x) * 4000.0 / 32767.0)
 *
 * int main() {
 *     int16_t accl[3];
 * 
 *     lis2dw.setup();
 *     ThisThread::sleep_for(100);
 * 
 *     while(1) {
 *         baro_2smpb.read(&pres, &temp, &dp, &dt);
 *         printf("x : %5d [mg]\r\n", CONV_RAW_TO_MG(accl[0]));
 *         printf("y : %5d [mg]\r\n", CONV_RAW_TO_MG(accl[1]));
 *         printf("z : %5d [mg]\r\n", CONV_RAW_TO_MG(accl[2]));
 *         ThisThread::sleep_for(200);
 *     }
 * }
 * @endcode
 */
class LIS2DW12
{
public:
    /** Create a sensor instance
     *  @param mosi SPI Master Out, Slave In pin.
     *  @param miso SPI Master In, Slave Out pin.
     *  @param sclk SPI Clock pin.
     *  @param ssel SPI Chip Select pin.
     */
    LIS2DW12(PinName mosi, PinName miso, PinName sclk, PinName sse);

    /** Initialize a sensor device
     *
     *  Should be called once per lifetime of the object.
     *  Please wait about 100ms before calling the read function.
     *  @return true on success, false on failure
     */
    bool setup(void);

    /** Read the current data from sensor
     *
     *  @param accl  Average acceleration value. (accl[0] = x-axis, accl[1] = y-axis, accl[2] = z-axis)
     *  @param count Number of times to get. (default 1)
     *  @return true on success, false on failure
     */
    bool read(int16_t* accl, uint8_t count = 1);

private:
    SPI mspi_;
    DigitalOut ssl_;

    void write_reg(uint8_t reg, uint8_t* pbuf, uint8_t len);
    void read_reg(uint8_t reg, uint8_t* pbuf, uint8_t len);
};

#endif
