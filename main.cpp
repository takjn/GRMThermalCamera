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
#include "r_drp_simple_isp.h"
#include "r_drp_resize_bilinear.h"
#include "dcache-control.h"
#include "DisplayApp.h"
#include "JPEG_Converter.h"

#include "D6T_44L_06.h"

#ifndef MBED_CONF_APP_LCD
    #error "MBED_CONF_APP_LCD is not set"
#endif

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
#define VIDEO_PIXEL_HW         (1280)
#define VIDEO_PIXEL_VW         (720)

static DisplayBase Display;
static DisplayApp  display_app;
static uint8_t JpegBuffer[1024 * 128]__attribute((aligned(32)));

// Buffer for video
#define FRAME_BUFFER_STRIDE    (((VIDEO_PIXEL_HW * 1) + 63u) & ~63u)
#define FRAME_BUFFER_HEIGHT    (VIDEO_PIXEL_VW)
static uint8_t fbuf_bayer[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(128)));
static uint8_t fbuf_camera[FRAME_BUFFER_STRIDE*2 * FRAME_BUFFER_HEIGHT]__attribute((section("OCTA_BSS"),aligned(32)));

// Buffer for sensor data
#define HEATMAP_PIXEL_HW         (1280)
#define HEATMAP_PIXEL_VW         (720)

#define SENSOR_RAW_BUFFER_STRIDE    (((4 * 1) + 31u) & ~31u)
#define SENSOR_RAW_BUFFER_HEIGHT    (4)
#define SENSOR_WORK_BUFFER_STRIDE    (((HEATMAP_PIXEL_HW * 1) + 31u) & ~31u)
#define SENSOR_RESULT_BUFFER_STRIDE  (((HEATMAP_PIXEL_HW * 4) + 31u) & ~31u)
#define SENSOR_RESULT_BUFFER_HEIGHT  (HEATMAP_PIXEL_VW)
static uint8_t sensor_raw_buffer[SENSOR_RAW_BUFFER_STRIDE * SENSOR_RAW_BUFFER_HEIGHT]__attribute((section("NC_BSS")));
static uint8_t sensor_work_buffer[SENSOR_WORK_BUFFER_STRIDE * SENSOR_RESULT_BUFFER_HEIGHT]__attribute((section("OCTA_BSS"),aligned(32)));
uint8_t sensor_result_buffer[SENSOR_RESULT_BUFFER_STRIDE * SENSOR_RESULT_BUFFER_HEIGHT]__attribute((section("OCTA_BSS"),aligned(32)));

#define SENSOR_WORK_BUFFER_STRIDE_2  (((HEATMAP_PIXEL_HW * 2) + 31u) & ~31u)
static uint8_t fbuf_yuv[SENSOR_WORK_BUFFER_STRIDE_2 * SENSOR_RESULT_BUFFER_HEIGHT]__attribute((aligned(32)));

// RGB to YUV convertion
// https://stackoverflow.com/questions/1737726/how-to-perform-rgb-yuv-conversion-in-c-c
#define CLIP(X) ( (X) > 255 ? 255 : (X) < 0 ? 0 : X)

// RGB -> YUV
#define RGB2Y(R, G, B) CLIP(( (  66 * (R) + 129 * (G) +  25 * (B) + 128) >> 8) +  16)
#define RGB2U(R, G, B) CLIP(( ( -38 * (R) -  74 * (G) + 112 * (B) + 128) >> 8) + 128)
#define RGB2V(R, G, B) CLIP(( ( 112 * (R) -  94 * (G) -  18 * (B) + 128) >> 8) + 128)


// Variables for DRP
#define DRP_FLG_TILE_ALL       (R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5)
#define DRP_FLG_CAMER_IN       (0x00000100)
static uint8_t drp_lib_id[R_DK2_TILE_NUM] = {0};
static Thread drpTask(osPriorityHigh, 1024*8);
static r_drp_simple_isp_t param_isp __attribute((section("NC_BSS")));
static r_drp_resize_bilinear_t param_resize_bilinear __attribute((section("NC_BSS")));

// Alpha ratio for blending camera image and heat map
static float display_alpha = 0.5;
static bool show_value = true;

// Variables for omron sensor
// D6T_44L_06
static D6T_44L_06 d6t_44l(I2C_SDA, I2C_SCL);
static Thread sensorTask(osPriorityNormal, 1024*8);
#define TILE_TEMP_MARGIN_UPPER (20)
#define TILE_TEMP_MARGIN_UNDER (70)
static int16_t pdta;
static int16_t buf[16];

// 2JCIE-EV01-RP1
#define CONV_RAW_TO_MG(x) (int)((double)(x) * 4000.0 / 32767.0)

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

    rect.vs = 0;
    rect.vw = HEATMAP_PIXEL_VW;
    rect.hs = 0;
    rect.hw = HEATMAP_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        (void *)fbuf_yuv,
        SENSOR_WORK_BUFFER_STRIDE_2,
        DisplayBase::GRAPHICS_FORMAT_YCBCR422,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);

    ThisThread::sleep_for(50);
    EasyAttach_LcdBacklight(true);
}

static void drp_task(void) {
    JPEG_Converter  Jcu;
    JPEG_Converter::bitmap_buff_info_t bitmap_buff_info;
    JPEG_Converter::encode_options_t   encode_options;
    size_t encode_size;
    bitmap_buff_info.width              = HEATMAP_PIXEL_HW;
    bitmap_buff_info.height             = HEATMAP_PIXEL_VW;
    bitmap_buff_info.format             = JPEG_Converter::WR_RD_YCbCr422;
    bitmap_buff_info.buffer_address     = (void *)fbuf_yuv;
    encode_options.encode_buff_size     = sizeof(JpegBuffer);
    encode_options.input_swapsetting    = JPEG_Converter::WR_RD_WRSWA_32_16BIT;  //WR_RD_WRSWA_32BIT
    encode_options.width = HEATMAP_PIXEL_HW;
    encode_options.height = HEATMAP_PIXEL_VW;

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

        /* SimpleIsp bayer2yuv_6 */
        R_DK2_Load(g_drp_lib_simple_isp_bayer2yuv_6,
            R_DK2_TILE_0,
            R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        memset(&param_isp, 0, sizeof(param_isp));
        param_isp.src    = (uint32_t)fbuf_bayer;
        param_isp.dst    = (uint32_t)fbuf_camera;
        param_isp.width  = VIDEO_PIXEL_HW;
        param_isp.height = VIDEO_PIXEL_VW;
        param_isp.gain_r = 0x1800;
        param_isp.gain_g = 0x1000;
        param_isp.gain_b = 0x1C00;
        param_isp.bias_r = 20;
        param_isp.bias_g = 20;
        param_isp.bias_b = 20;
        R_DK2_Start(drp_lib_id[0], (void *)&param_isp, sizeof(r_drp_simple_isp_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);

        // resize bilinear
        R_DK2_Load(g_drp_lib_resize_bilinear,
            R_DK2_TILE_0,
            R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        // magnification sensor data
        memset(&param_resize_bilinear, 0, sizeof(param_resize_bilinear));
        param_resize_bilinear.src    = (uint32_t)sensor_raw_buffer;
        param_resize_bilinear.dst    = (uint32_t)sensor_work_buffer;
        param_resize_bilinear.src_width  = 4;
        param_resize_bilinear.src_height = 4;
        param_resize_bilinear.dst_width  = HEATMAP_PIXEL_HW;
        param_resize_bilinear.dst_height = HEATMAP_PIXEL_VW;
        R_DK2_Start(drp_lib_id[0], (void *)&param_resize_bilinear, sizeof(r_drp_resize_bilinear_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);

        R_DK2_Unload(0, drp_lib_id);

        if (show_value) {
            // draw sensor value
            for (int y=0;y<4;y++) {
                for (int x=0;x<4;x++) {
                    int px = HEATMAP_PIXEL_HW / 4 * (x + 1) - HEATMAP_PIXEL_HW / 6;
                    int py = HEATMAP_PIXEL_VW / 4 * (y + 1) - HEATMAP_PIXEL_VW / 7;

                    sprintf(str, "%4.1f   ", buf[y*4+x] / 10.0);
                    ascii_font.DrawStr(str, px, py, 0xFFFFFFFF, 4);
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

            // convert RGB to YUV for JCU
            float display_alpha2 = (1 - display_alpha);
            for (uint i=0;i<sizeof(fbuf_yuv);i+=4) {
                uint8_t r1 = sensor_result_buffer[i*2+2];
                uint8_t g1 = sensor_result_buffer[i*2+1];
                uint8_t b1 = sensor_result_buffer[i*2+0];
                uint8_t r2 = sensor_result_buffer[i*2+6];
                uint8_t g2 = sensor_result_buffer[i*2+5];
                uint8_t b2 = sensor_result_buffer[i*2+4];

                uint8_t y1 = RGB2Y(r1, g1, b1);
                uint8_t u1 = RGB2U(r1, g1, b1);
                uint8_t v1 = RGB2V(r1, g1, b1);
                uint8_t y2 = RGB2Y(r2, g2, b2);

                fbuf_yuv[i+0]=((float)fbuf_camera[i+0]*display_alpha + (float)u1*display_alpha2); // u
                fbuf_yuv[i+1]=((float)fbuf_camera[i+1]*display_alpha + (float)y1*display_alpha2); // y
                fbuf_yuv[i+2]=((float)fbuf_camera[i+2]*display_alpha + (float)v1*display_alpha2); // v
                fbuf_yuv[i+3]=((float)fbuf_camera[i+3]*display_alpha + (float)y2*display_alpha2); // y
            }
        } else {
            memcpy(fbuf_yuv, fbuf_camera, sizeof(fbuf_camera));
        }
        dcache_clean(fbuf_yuv, sizeof(fbuf_yuv));

        dcache_invalid(JpegBuffer, sizeof(JpegBuffer));
        if (Jcu.encode(&bitmap_buff_info, JpegBuffer, &encode_size, &encode_options) == JPEG_Converter::JPEG_CONV_OK) {
            display_app.SendJpeg(JpegBuffer, (int)encode_size);
        }
    }
}

static void sensor_task(void) {
    // setup sensors
    d6t_44l.setup();
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
    }
}

static void button_fall0(void) {
    display_alpha+=0.2;
    if (display_alpha > 1.0) {
        display_alpha = 0.0;
    }
}

static void button_fall1(void) {
    show_value = !show_value;
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

