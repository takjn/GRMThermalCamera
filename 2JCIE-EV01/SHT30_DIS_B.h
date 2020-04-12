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

#ifndef SHT30_DIS_B_H
#define SHT30_DIS_B_H

#include "mbed.h"

/** Humidity and Temperature Sensor [SHT30-DIS-B] class
 *
 * @note Synchronization level: Thread safe
 *
 * Example of how to get data from this sensor:
 * @code
 *
 * #include "mbed.h"
 * #include "SHT30_DIS_B.h"
 *
 * SHT30_DIS_B sht30(I2C_SDA, I2C_SCL);
 *
 * int main() {
 *     int32_t humi, temp;
 * 
 *     sht30.setup();
 *     ThisThread::sleep_for(100);
 * 
 *     while(1) {
 *         sht30.read(&humi, &temp);
 *         printf("humidity    : %5.2f [%%RH]\r\n", humi / 100.0);
 *         printf("temperature : %5.2f [degC]\r\n", temp / 100.0);
 *         ThisThread::sleep_for(200);
 *     }
 * }
 * @endcode
 */
class SHT30_DIS_B
{
public:
    /** Create a sensor instance
     *  
     *  @param sda I2C data line pin
     *  @param scl I2C clock line pin
     *  @param hz The bus frequency in hertz
     */
    SHT30_DIS_B(PinName sda, PinName scl, int hz = 150000);

    /** Initialize a sensor device
     *
     *  Should be called once per lifetime of the object.
     *  Please wait about 100ms before calling the read function.
     *  @return true on success, false on failure
     */
    bool setup(void);

    /** Read the current data from sensor
     *
     *  @param humi  Hundredfold humidity value (relative humidity)
     *  @param temp  Hundredfold temperature value (degree Celsius)
     *  @return true on success, false on failure
     */
    bool read(int32_t* humi, int32_t* temp);

private:
    I2C mI2c_;
    int mAddr;

    int read_reg16(uint16_t reg, uint8_t* pbuf, uint8_t len);
};

#endif
