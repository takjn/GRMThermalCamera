// SPDX-License-Identifier: MIT
/*
 * MIT License
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

#include "mbed.h"
#include "EasyAttach_CameraAndLCD.h"
#include "AsciiFont.h"
#include "r_dk2_if.h"
#include "r_drp_bayer2grayscale.h"
#include "r_drp_median_blur.h"
#include "r_drp_canny_calculate.h"
#include "r_drp_canny_hysterisis.h"
#include "r_drp_resize_bilinear.h"
#include "dcache-control.h"
#include "DisplayApp.h"

#include "D6T_44L_06.h"
#include "SHT30_DIS_B.h"
#include "OPT3001DNP.h"
#include "BARO_2SMPB_02E.h"
#include "LIS2DW12.h"

#ifndef MBED_CONF_APP_LCD
    #error "MBED_CONF_APP_LCD is not set"
#endif

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
#define VIDEO_PIXEL_HW         (1280)    /* VGA */
#define VIDEO_PIXEL_VW         (720)    /* VGA */

static DisplayBase Display;
static DisplayApp  display_app;

// Buffer for video
#define FRAME_BUFFER_STRIDE    (((VIDEO_PIXEL_HW * 1) + 63u) & ~63u)
#define FRAME_BUFFER_HEIGHT    (VIDEO_PIXEL_VW)
static uint8_t fbuf_bayer[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(128)));
static uint8_t fbuf_gray[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));

// Buffer for sensor data
#define HEATMAP_PIXEL_HW         (320)    /* VGA */
#define HEATMAP_PIXEL_VW         (240)    /* VGA */

#define SENSOR_RAW_BUFFER_STRIDE    (((4 * 1) + 31u) & ~31u)
#define SENSOR_RAW_BUFFER_HEIGHT    (4)
#define SENSOR_WORK_BUFFER_STRIDE    (((HEATMAP_PIXEL_HW * 1) + 31u) & ~31u)
#define SENSOR_RESULT_BUFFER_STRIDE  (((HEATMAP_PIXEL_HW * 4) + 31u) & ~31u)
#define SENSOR_RESULT_BUFFER_HEIGHT  (HEATMAP_PIXEL_VW)
static uint8_t sensor_raw_buffer[SENSOR_RAW_BUFFER_STRIDE * SENSOR_RAW_BUFFER_HEIGHT]__attribute((section("NC_BSS")));
static uint8_t sensor_work_buffer[SENSOR_WORK_BUFFER_STRIDE * SENSOR_RESULT_BUFFER_HEIGHT]__attribute((section("NC_BSS")));
uint8_t sensor_result_buffer[SENSOR_RESULT_BUFFER_STRIDE * SENSOR_RESULT_BUFFER_HEIGHT]__attribute((section("NC_BSS")));


static uint8_t fbuf_work0[SENSOR_WORK_BUFFER_STRIDE * SENSOR_RESULT_BUFFER_HEIGHT]__attribute((aligned(32)));
static uint8_t fbuf_work1[SENSOR_WORK_BUFFER_STRIDE * SENSOR_RESULT_BUFFER_HEIGHT]__attribute((aligned(32)));
static uint8_t fbuf_clat8[SENSOR_WORK_BUFFER_STRIDE * SENSOR_RESULT_BUFFER_HEIGHT]__attribute((aligned(32)));
static uint8_t drp_work_buf[((SENSOR_WORK_BUFFER_STRIDE * ((SENSOR_RESULT_BUFFER_HEIGHT / 3) + 2)) * 2) * 3]__attribute((section("NC_BSS")));


// Variables for DRP
#define DRP_FLG_TILE_ALL       (R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5)
#define DRP_FLG_CAMER_IN       (0x00000100)
static uint8_t drp_lib_id[R_DK2_TILE_NUM] = {0};
static Thread drpTask(osPriorityHigh, 1024*8);
static uint8_t nc_memory[512] __attribute((section("NC_BSS")));
static r_drp_resize_bilinear_t param_resize_bilinear __attribute((section("NC_BSS")));

// Variables for omron sensor
// D6T_44L_06
static D6T_44L_06 d6t_44l(I2C_SDA, I2C_SCL);
static Thread sensorTask(osPriorityNormal, 1024*8);
#define TILE_TEMP_MARGIN_UPPER (20)
#define TILE_TEMP_MARGIN_UNDER (70)
static uint8_t display_alpha = 128;
static int16_t pdta;
static int16_t buf[16];

// 2JCIE-EV01-RP1
#define CONV_RAW_TO_MG(x) (int)((double)(x) * 4000.0 / 32767.0)
SHT30_DIS_B sht30(I2C_SDA, I2C_SCL);                    // [SHT30-DIS-B] : Temperature / humidity sensor
OPT3001DNP opt3001(I2C_SDA, I2C_SCL);                   // [OPT3001DNP]  : Ambient light sensor
BARO_2SMPB_02E baro_2smpb(I2C_SDA, I2C_SCL);            // [2SMPB-02E]   : MEMS digital barometric pressure sensor
LIS2DW12 lis2dw(SPI_MOSI, SPI_MISO, SPI_SCKL, SPI_SSL); // [LIS2DW12]    : MEMS digital motion sensor

static InterruptIn button0(USER_BUTTON0);
static InterruptIn button1(USER_BUTTON1);

/*******************************************************************************
* Function Name: normalize0to1
* Description  : Normalize thermal data of a reception packet to the range of 0-1.
*                Also the value outside min, max is saturated.
* Arguments    : data     - input tempetue data
*                min      - smallest threshold tempature value
*                           (The integer which set a centigrade to 10 times)
*                max      - highest threshold  tempature value
*                           (The integer which set a centigrade to 10 times)
* Return Value : normalized output data
*******************************************************************************/
float normalize0to1(int16_t data, int min, int max)
{
    float normalized;

    if (data <= min) {
        normalized = 0.0;
    }
    else if (max <= data) {
        normalized = 1.0;
    }
    else {
        normalized = ((float)(data - min)) / ((float)(max - min));
    }
    return normalized;
}
/*******************************************************************************
 End of function normalize0to1
*******************************************************************************/

/*******************************************************************************
* Function Name: conv_normalize_to_color
* Description  : Change a floating-point data of 0.0-1.0 to the pixel color of
*                ARGB8888.
*                Low temperature changes blue and high temperature to red.
* Arguments    : data        - Normalized thermal data from 0.0 to 1.0.
* Return Value : ARGB8888 pixel color data.
*******************************************************************************/
// pre-calculation to increase speed
static uint32_t colors[256];
static void init_colors() {
    uint8_t green;
    uint8_t blue;
    uint8_t red;

    for (int i=0;i<256;i++) {
        float cosval = cos( 4 * M_PI * (float(i)/255.0));
        uint8_t color = (uint8_t)((((-cosval)/2) + 0.5) * 255);

        if (0 == i) {
            /* Display blue when the temperature is below the minimum. */
            blue  = 0xFF;
            green = 0x00;
            red   = 0x00;
        }
        else if (255 == i) {
            /* Display red when the maximum temperature is exceeded. */
            blue  = 0x00;
            green = 0x00;
            red   = 0xFF;
        }
        else {
            if (i < 64) {
                blue  = 0xFF;
                green = color;
                red   = 0x00;
            }
            else if (i < 128) {
                blue  = color;
                green = 0xFF;
                red   = 0x00;
            }
            else if (i < 192) {
                blue  = 0x00;
                green = 0xFF;
                red   = color;
            }
            else {
                blue  = 0x00;
                green = color;
                red   = 0xFF;
            }
        }

        colors[i] = ((0xFF << 24) | (red << 16) | (green << 8) | blue);
    }
} 

/*******************************************************************************
 End of function conv_normalize_to_color
*******************************************************************************/

static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type) {
    drpTask.flags_set(DRP_FLG_CAMER_IN);
}

static void cb_drp_finish(uint8_t id) {
    uint32_t tile_no;
    uint32_t set_flgs = 0;

    // Change the operation state of the DRP library notified by the argument to finish
    for (tile_no = 0; tile_no < R_DK2_TILE_NUM; tile_no++) {
        if (drp_lib_id[tile_no] == id) {
            set_flgs |= (1 << tile_no);
        }
    }
    drpTask.flags_set(set_flgs);
}

static void Start_Video_Camera(void) {
    // Video capture setting (progressive form fixed)
    Display.Video_Write_Setting(
        DisplayBase::VIDEO_INPUT_CHANNEL_0,
        DisplayBase::COL_SYS_NTSC_358,
        (void *)fbuf_bayer,
        FRAME_BUFFER_STRIDE,
        DisplayBase::VIDEO_FORMAT_RAW8,
        DisplayBase::WR_RD_WRSWA_NON,
        VIDEO_PIXEL_VW,
        VIDEO_PIXEL_HW
    );
    EasyAttach_CameraStart(Display, DisplayBase::VIDEO_INPUT_CHANNEL_0);
}

static void Start_LCD_Display(void) {
    DisplayBase::rect_t rect;

    // for camera image
    rect.vs = HEATMAP_PIXEL_VW;
    rect.vw = HEATMAP_PIXEL_VW;
    rect.hs = HEATMAP_PIXEL_HW;
    rect.hw = HEATMAP_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        (void *)fbuf_clat8,
        SENSOR_WORK_BUFFER_STRIDE,
        DisplayBase::GRAPHICS_FORMAT_CLUT8,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);

    // for sensor image
    rect.vs = 0;
    rect.vw = HEATMAP_PIXEL_VW;
    rect.hs = 0;
    rect.hw = HEATMAP_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_2,
        (void *)sensor_result_buffer,
        SENSOR_RESULT_BUFFER_STRIDE,
        DisplayBase::GRAPHICS_FORMAT_ARGB8888,
        DisplayBase::WR_RD_WRSWA_NON,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_2);

    ThisThread::sleep_for(50);
    EasyAttach_LcdBacklight(true);
}

static void drp_task(void) {
    uint32_t idx = 0;

    EasyAttach_Init(Display);
    // Interrupt callback function setting (Field end signal for recording function in scaler 0)
    Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VFIELD, 0, IntCallbackFunc_Vfield);
    Start_Video_Camera();
    Start_LCD_Display();

    R_DK2_Initialize();

    char str[64];
    AsciiFont ascii_font(sensor_result_buffer, HEATMAP_PIXEL_HW, HEATMAP_PIXEL_VW, SENSOR_RESULT_BUFFER_STRIDE, 4);

    while (true) {
        ThisThread::flags_wait_all(DRP_FLG_CAMER_IN);

        // Bayer2Grayscale
        R_DK2_Load(
                   g_drp_lib_bayer2grayscale,
                   R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
                   R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);

        r_drp_bayer2grayscale_t * param_b2g = (r_drp_bayer2grayscale_t *)nc_memory;
        for (idx = 0; idx < R_DK2_TILE_NUM; idx++) {
            param_b2g[idx].src    = (uint32_t)fbuf_bayer + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
            param_b2g[idx].dst    = (uint32_t)fbuf_gray + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
            param_b2g[idx].width  = VIDEO_PIXEL_HW;
            param_b2g[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
            param_b2g[idx].top    = (idx == 0) ? 1 : 0;
            param_b2g[idx].bottom = (idx == 5) ? 1 : 0;
            R_DK2_Start(drp_lib_id[idx], (void *)&param_b2g[idx], sizeof(r_drp_bayer2grayscale_t));
        }
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);

        // resize bilinear
        R_DK2_Load(g_drp_lib_resize_bilinear,
            R_DK2_TILE_0,
            R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        memset(&param_resize_bilinear, 0, sizeof(param_resize_bilinear));
        param_resize_bilinear.src    = (uint32_t)fbuf_gray;
        param_resize_bilinear.dst    = (uint32_t)fbuf_work0;
        param_resize_bilinear.src_width  = VIDEO_PIXEL_HW;
        param_resize_bilinear.src_height = VIDEO_PIXEL_VW;
        param_resize_bilinear.dst_width  = HEATMAP_PIXEL_HW;
        param_resize_bilinear.dst_height = HEATMAP_PIXEL_VW;
        R_DK2_Start(drp_lib_id[0], (void *)&param_resize_bilinear, sizeof(r_drp_resize_bilinear_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);

        // MedianBlur
        R_DK2_Load(
                   g_drp_lib_median_blur,
                   R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
                   R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);

        r_drp_median_blur_t * param_median = (r_drp_median_blur_t *)nc_memory;
        for (idx = 0; idx < R_DK2_TILE_NUM; idx++) {
            param_median[idx].src    = (uint32_t)fbuf_work0 + (HEATMAP_PIXEL_HW * (HEATMAP_PIXEL_VW / R_DK2_TILE_NUM) * idx);
            param_median[idx].dst    = (uint32_t)fbuf_work1 + (HEATMAP_PIXEL_HW * (HEATMAP_PIXEL_VW / R_DK2_TILE_NUM) * idx);
            param_median[idx].width  = HEATMAP_PIXEL_HW;
            param_median[idx].height = HEATMAP_PIXEL_VW / R_DK2_TILE_NUM;
            param_median[idx].top    = (idx == 0) ? 1 : 0;
            param_median[idx].bottom = (idx == 5) ? 1 : 0;
            R_DK2_Start(drp_lib_id[idx], (void *)&param_median[idx], sizeof(r_drp_median_blur_t));
        }
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);

        // CannyCalculate
        R_DK2_Load(
                   g_drp_lib_canny_calculate,
                   R_DK2_TILE_0 | R_DK2_TILE_2 | R_DK2_TILE_4,
                   R_DK2_TILE_PATTERN_2_2_2, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);

        r_drp_canny_calculate_t * param_canny_cal = (r_drp_canny_calculate_t *)nc_memory;
        for (idx = 0; idx < 3; idx++) {
            param_canny_cal[idx].src    = (uint32_t)fbuf_work1 + (HEATMAP_PIXEL_HW * (HEATMAP_PIXEL_VW / 3) * idx);
            param_canny_cal[idx].dst    = (uint32_t)fbuf_work0 + (HEATMAP_PIXEL_HW * (HEATMAP_PIXEL_VW / 3) * idx);
            param_canny_cal[idx].width  = HEATMAP_PIXEL_HW;
            param_canny_cal[idx].height = (HEATMAP_PIXEL_VW / 3);
            param_canny_cal[idx].top    = ((idx * 2) == 0) ? 1 : 0;
            param_canny_cal[idx].bottom = ((idx * 2) == 4) ? 1 : 0;
            param_canny_cal[idx].work   = (uint32_t)&drp_work_buf[((HEATMAP_PIXEL_HW * ((HEATMAP_PIXEL_VW / 3) + 2)) * 2) * idx];
            param_canny_cal[idx].threshold_high = 0x28;
            param_canny_cal[idx].threshold_low  = 0x18;
            R_DK2_Start(drp_lib_id[(idx * 2)], (void *)&param_canny_cal[idx], sizeof(r_drp_canny_calculate_t));
        }
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);

        // CannyHysterisis
        R_DK2_Load(
                   g_drp_lib_canny_hysterisis,
                   R_DK2_TILE_0,
                   R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);

        r_drp_canny_hysterisis_t * param_canny_hyst = (r_drp_canny_hysterisis_t *)nc_memory;
        param_canny_hyst[0].src    = (uint32_t)fbuf_work0;
        param_canny_hyst[0].dst    = (uint32_t)fbuf_clat8;
        param_canny_hyst[0].width  = HEATMAP_PIXEL_HW;
        param_canny_hyst[0].height = HEATMAP_PIXEL_VW;
        param_canny_hyst[0].work   = (uint32_t)drp_work_buf;
        param_canny_hyst[0].iterations = 2;
        R_DK2_Start(drp_lib_id[0], (void *)&param_canny_hyst[0], sizeof(r_drp_canny_hysterisis_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);

        /* resize bilinear */
        R_DK2_Load(g_drp_lib_resize_bilinear,
            R_DK2_TILE_0,
            R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        memset(&param_resize_bilinear, 0, sizeof(param_resize_bilinear));
        param_resize_bilinear.src    = (uint32_t)sensor_raw_buffer;
        param_resize_bilinear.dst    = (uint32_t)sensor_work_buffer;
        param_resize_bilinear.src_width  = SENSOR_RAW_BUFFER_STRIDE;
        param_resize_bilinear.src_height = SENSOR_RAW_BUFFER_HEIGHT;
        param_resize_bilinear.dst_width  = HEATMAP_PIXEL_HW;
        param_resize_bilinear.dst_height = HEATMAP_PIXEL_VW;
        R_DK2_Start(drp_lib_id[0], (void *)&param_resize_bilinear, sizeof(r_drp_resize_bilinear_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);

        // draw sensor value
        // sprintf(str, "PTAT: %6.1f[degC]   ", pdta / 10.0);
        // ascii_font.DrawStr(str, (AsciiFont::CHAR_PIX_WIDTH)*2, (AsciiFont::CHAR_PIX_HEIGHT)*2, 0xFFFFFFFF, 2);
        for (int y=0;y<4;y++) {
            for (int x=0;x<4;x++) {
                int px = HEATMAP_PIXEL_HW / 4 * (x + 1) - HEATMAP_PIXEL_HW / 6;
                int py = HEATMAP_PIXEL_VW / 4 * (y + 1) - HEATMAP_PIXEL_VW / 7;

                sprintf(str, "%4.1f   ", buf[y*4+x] / 10.0);
                ascii_font.DrawStr(str, px, py, 0xFFFFFFFF, 2);
            }
        }

        uint32_t *p = (uint32_t*)sensor_result_buffer;
        uint j=0;
        for (uint i=0;i<sizeof(sensor_work_buffer);i++) {
            if (p[j] != 0xFFFFFFFF) {
                p[j] = colors[sensor_work_buffer[i]];
            }
            j++;
        }

        display_app.SendRgb888(sensor_result_buffer, HEATMAP_PIXEL_HW, HEATMAP_PIXEL_VW);
    }
}

static void sensor_task(void) {
    int32_t humi, temp32;
    uint32_t illm;
    uint32_t pres, dp, dt;
    int16_t temp16;
    int16_t accl[3];

    // printf("\x1b[2J");  // Clear screen

    // setup sensors
    d6t_44l.setup();
    baro_2smpb.setup();
    sht30.setup();
    opt3001.setup();
    lis2dw.setup();
    ThisThread::sleep_for(150);

    while (true) {
        ThisThread::sleep_for(10);

        if (d6t_44l.read(&pdta, &buf[0]) == false) {
            continue;
        }
        for (int i = 0; i < 16; i++) {
            uint8_t a = (normalize0to1(buf[i], pdta - TILE_TEMP_MARGIN_UNDER,  pdta + TILE_TEMP_MARGIN_UPPER) * 255.0);
            sensor_raw_buffer[i]=a;
        }

        // printf("\x1b[%d;%dH", 0, 0);  // Move cursor (y , x)
        // printf("GR-MANGO x Omron 2JCIE-EV01 Demo\r\n\r\n");

        // sht30.read(&humi, &temp32);
        // printf("[SHT30-DIS-B] Temperature / humidity sensor\r\n");
        // printf("   temperature : %5.2f [degC]\r\n", temp32 / 100.0);
        // printf("   humidity    : %5.2f [%%RH]\r\n", humi / 100.0);
        // printf("\r\n");

        // opt3001.read(&illm);
        // printf("[OPT3001DNP] Ambient light sensor\r\n");
        // printf("   illuminance : %5.2f [lx]\r\n", illm / 100.0);
        // printf("\r\n");

        // baro_2smpb.read(&pres, &temp16, &dp, &dt);
        // printf("[2SMPB-02E] MEMS digital barometric pressure sensor\r\n");
        // printf("   pressure    : %7.1f [Pa] (%08Xh)\r\n", pres / 10.0 , (unsigned int)dp);
        // printf("   temperature : %5.2f [degC] (%08Xh)\r\n", temp16 / 100.0, (unsigned int)dt);
        // printf("\r\n");

        // lis2dw.read(accl);
        // printf("[LIS2DW12] MEMS digital motion sensor\r\n");
        // printf("   x : %5d [mg]\r\n", CONV_RAW_TO_MG(accl[0]));
        // printf("   y : %5d [mg]\r\n", CONV_RAW_TO_MG(accl[1]));
        // printf("   z : %5d [mg]\r\n", CONV_RAW_TO_MG(accl[2]));
        // printf("\r\n");
    }
}

static void button_fall0(void) {
    display_alpha+=64;
}

static void button_fall1(void) {
    display_alpha-=64;
}

int main(void) {
    // Pre-calculation for speed up
    init_colors();

    // Set up button
    button0.fall(&button_fall0);
    button1.fall(&button_fall1);

    // Start DRP task
    drpTask.start(callback(drp_task));

    // Start sensor task
    sensorTask.start(callback(sensor_task));

    wait(osWaitForever);
}

