#ifndef _NT99141_SETTINGS_H_
#define _NT99141_SETTINGS_H_
//CONFIG_NT99141_SUPPORT
#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "nt99141_regs.h"

static const ratio_settings_t ratio_table[] = {
    //  mw,   mh,  sx,  sy,   ex,   ey, ox, oy,   tx,   ty
    { 1280, 720,   0,   4, 1283, 723, 0, 4, 1660, 963 }, 

};

#define REG_DLY 0xffff
#define REGLIST_TAIL 0x0000

static const DRAM_ATTR uint16_t sensor_default_regs[][2] = {
 //initial
{0x3021, 0x00},
{REG_DLY, 100}, // delay 100ms
{0x3109, 0x04},
{0x3040, 0x04},
{0x3041, 0x02},
{0x3042, 0xFF},
{0x3043, 0x08},
{0x3052, 0xE0},
{0x305F, 0x33},
{0x3100, 0x07},
{0x3106, 0x03},
{0x3105, 0x01},
{0x3108, 0x05},
{0x3110, 0x22},
{0x3111, 0x57},
{0x3112, 0x22},
{0x3113, 0x55},
{0x3114, 0x05},
{0x3135, 0x00},
{0x32F0, 0x01},
{0x3290, 0x01},
{0x3291, 0x80},
{0x3296, 0x01},
{0x3297, 0x73},
{0x3250, 0x80},
{0x3251, 0x03},
{0x3252, 0xFF},
{0x3253, 0x00},
{0x3254, 0x03},
{0x3255, 0xFF},
{0x3256, 0x00},
{0x3257, 0x50},
{0x3270, 0x00},
{0x3271, 0x0C},
{0x3272, 0x18},
{0x3273, 0x32},
{0x3274, 0x44},
{0x3275, 0x54},
{0x3276, 0x70},
{0x3277, 0x88},
{0x3278, 0x9D},
{0x3279, 0xB0},
{0x327A, 0xCF},
{0x327B, 0xE2},
{0x327C, 0xEF},
{0x327D, 0xF7},
{0x327E, 0xFF},
{0x3302, 0x00},
{0x3303, 0x40},
{0x3304, 0x00},
{0x3305, 0x96},
{0x3306, 0x00},
{0x3307, 0x29},
{0x3308, 0x07},
{0x3309, 0xBA},
{0x330A, 0x06},
{0x330B, 0xF5},
{0x330C, 0x01},
{0x330D, 0x51},
{0x330E, 0x01},
{0x330F, 0x30},
{0x3310, 0x07},
{0x3311, 0x16},
{0x3312, 0x07},
{0x3313, 0xBA},
{0x3326, 0x02},
{0x32F6, 0x0F},
{0x32F9, 0x42},
{0x32FA, 0x24},
{0x3325, 0x4A},
{0x3330, 0x00},
{0x3331, 0x0A},
{0x3332, 0xFF},
{0x3338, 0x30},
{0x3339, 0x84},
{0x333A, 0x48},
{0x333F, 0x07},
{0x3360, 0x10},
{0x3361, 0x18},
{0x3362, 0x1f},
{0x3363, 0x37},
{0x3364, 0x80},
{0x3365, 0x80},
{0x3366, 0x68},
{0x3367, 0x60},
{0x3368, 0x30},
{0x3369, 0x28},
{0x336A, 0x20},
{0x336B, 0x10},
{0x336C, 0x00},
{0x336D, 0x20},
{0x336E, 0x1C},
{0x336F, 0x18},
{0x3370, 0x10},
{0x3371, 0x38},
{0x3372, 0x3C},
{0x3373, 0x3F},
{0x3374, 0x3F},
{0x338A, 0x34},
{0x338B, 0x7F},
{0x338C, 0x10},
{0x338D, 0x23},
{0x338E, 0x7F},
{0x338F, 0x14},
{0x3375, 0x08},
{0x3376, 0x0C},
{0x3377, 0x18},
{0x3378, 0x20},
{0x3012, 0x02},
{0x3013, 0xD0},
{0x3025, 0x02}, //colorbar
{REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_fmt_jpeg[][2] = {
    {0x32F0, 0x70}, // YUV422
    {REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_fmt_raw[][2] = {
    {0x32F0, 0x50}, // RAW
    {REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_fmt_grayscale[][2] = {
    {0x32F1, 0x01},
    {REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_fmt_yuv422[][2] = {
    {0x32F0, 0x00}, // YUV422
    {REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_fmt_rgb565[][2] = {
    {0x32F0, 0x01}, // RGB
    {REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint8_t sensor_saturation_levels[9][1] = {
    {0x60},//-4
    {0x68},//-3
    {0x70},//-2
    {0x78},//-1
    {0x80},//0
    {0x88},//+1
    {0x90},//+2
    {0x98},//+3
    {0xA0},//+4
};

static const DRAM_ATTR uint8_t sensor_special_effects[7][4] = {
    {0x00, 0x80, 0x80, 0x01},//Normal
    {0x03, 0x80, 0x80, 0x01},//Negative
    {0x01, 0x80, 0x80, 0x01},//Grayscale
    {0x05, 0x2A, 0xF0, 0x01},//Red Tint
    {0x05, 0x60, 0x20, 0x01},//Green Tint
    {0x05, 0xF0, 0x80, 0x01},//Blue Tint
    {0x02, 0x80, 0x80, 0x01},//Sepia
	
};

// AE LEVEL
static const DRAM_ATTR uint16_t sensor_ae_level[][2] = {

// 1. [AE_Target : 0x24]
// Set_Device_Format = FORMAT_16_8
// SET_Device_Addr = 0x54
 {0x32B8, 0x29 },
 {0x32B9, 0x1F },
 {0x32BC, 0x24 },
 {0x32BD, 0x27 },
 {0x32BE, 0x21 },
//------------------------------------------------------------------------
// 2. [AE_Target : 0x28]
// Set_Device_Format = FORMAT_16_8
// SET_Device_Addr = 0x54
 {0x32B8, 0x2D },
 {0x32B9, 0x23 },
 {0x32BC, 0x28 },
 {0x32BD, 0x2B },
 {0x32BE, 0x25 },
//------------------------------------------------------------------------
// 3. [AE_Target : 0x2C]
// Set_Device_Format = FORMAT_16_8
// SET_Device_Addr = 0x54
 {0x32B8, 0x32 },
 {0x32B9, 0x26 },
 {0x32BC, 0x2C },
 {0x32BD, 0x2F },
 {0x32BE, 0x29 },
//------------------------------------------------------------------------
// 4, [AE_Target : 0x30]
// Set_Device_Format = FORMAT_16_8
// SET_Device_Addr = 0x54
 {0x32B8, 0x36 },
 {0x32B9, 0x2A },
 {0x32BC, 0x30 },
 {0x32BD, 0x33 },
 {0x32BE, 0x2D },
//------------------------------------------------------------------------
// 5. [AE_Target : 0x34]
// Set_Device_Format = FORMAT_16_8
// SET_Device_Addr = 0x54
 {0x32B8, 0x3B },
 {0x32B9, 0x2D },
 {0x32BC, 0x34 },
 {0x32BD, 0x38 },
 {0x32BE, 0x30 },
//------------------------------------------------------------------------
// 6. [AE_Target : 0x38]
// Set_Device_Format = FORMAT_16_8
// SET_Device_Addr = 0x54
 {0x32B8, 0x3F },
 {0x32B9, 0x31 },
 {0x32BC, 0x38 },
 {0x32BD, 0x3C },
 {0x32BE, 0x34 },
//------------------------------------------------------------------------
// 7. [AE_Target : 0x3D]
// Set_Device_Format = FORMAT_16_8
// SET_Device_Addr = 0x54
 {0x32B8, 0x44 },
 {0x32B9, 0x34 },
 {0x32BC, 0x3C },
 {0x32BD, 0x40 },
 {0x32BE, 0x38 },
//------------------------------------------------------------------------
// 8. [AE_Target : 0x40]
// Set_Device_Format = FORMAT_16_8
// SET_Device_Addr = 0x54
 {0x32B8, 0x48 },
 {0x32B9, 0x38 },
 {0x32BC, 0x40 },
 {0x32BD, 0x44 },
 {0x32BE, 0x3C },
//------------------------------------------------------------------------
// 9. [AE_Target : 0x44]
// Set_Device_Format = FORMAT_16_8
// SET_Device_Addr = 0x54
 {0x32B8, 0x4D },
 {0x32B9, 0x3B },
 {0x32BC, 0x44 },
 {0x32BD, 0x49 },
 {0x32BE, 0x3F },
};

static const DRAM_ATTR uint16_t sensor_framesize_HD[][2] = {
//[JPEG_1280x720_8.18_8.18_Fps]
{0x3021, 0x00},
{REG_DLY, 100}, // delay 100ms
{0x32BF, 0x60}, 
{0x32C0, 0x5A}, 
{0x32C1, 0x5A}, 
{0x32C2, 0x5A}, 
{0x32C3, 0x00}, 
{0x32C4, 0x20}, 
{0x32C5, 0x20}, 
{0x32C6, 0x20}, 
{0x32C7, 0x00}, 
{0x32C8, 0x3C}, 
{0x32C9, 0x5A}, 
{0x32CA, 0x7A}, 
{0x32CB, 0x7A}, 
{0x32CC, 0x7A}, 
{0x32CD, 0x7A}, 
{0x32DB, 0x5E}, 
{0x32F0, 0x70}, 
{0x3400, 0x08}, 
{0x3400, 0x00}, 
{0x3401, 0x4E}, 
{0x3404, 0x00}, 
{0x3405, 0x00}, 
{0x3410, 0x00}, 
{0x3200, 0x3E}, 
{0x3201, 0x0F}, 
{0x3028, 0x0F}, 
{0x3029, 0x00}, 
{0x302A, 0x08}, 
{0x3022, 0x24}, 
{0x3023, 0x24}, 
{0x3002, 0x00}, 
{0x3003, 0x04}, 
{0x3004, 0x00}, 
{0x3005, 0x04}, 
{0x3006, 0x05}, 
{0x3007, 0x03}, 
{0x3008, 0x02}, 
{0x3009, 0xD3}, 
{0x300A, 0x06}, 
{0x300B, 0x7C}, 
{0x300C, 0x02}, 
{0x300D, 0xE0}, 
{0x300E, 0x05}, 
{0x300F, 0x00}, 
{0x3010, 0x02}, 
{0x3011, 0xD0}, 
{0x32B8, 0x3F}, 
{0x32B9, 0x31}, 
{0x32BB, 0x87}, 
{0x32BC, 0x38}, 
{0x32BD, 0x3C}, 
{0x32BE, 0x34}, 
{0x3201, 0x3F}, 
{0x3021, 0x06},
{0x3025, 0x00}, //normal 
{0x3400, 0x01}, 
{0x3060, 0x01}, 
{REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_framesize_VGA[][2] = {
//[JPEG_640x480_10.14_10.14_Fps]
{0x3021, 0x00},
{REG_DLY, 100}, // delay 100ms
{0x32BF, 0x60}, 
{0x32C0, 0x5A}, 
{0x32C1, 0x5A}, 
{0x32C2, 0x5A}, 
{0x32C3, 0x00}, 
{0x32C4, 0x20}, 
{0x32C5, 0x20}, 
{0x32C6, 0x20}, 
{0x32C7, 0x00}, 
{0x32C8, 0x4B}, 
{0x32C9, 0x5A}, 
{0x32CA, 0x7A}, 
{0x32CB, 0x7A}, 
{0x32CC, 0x7A}, 
{0x32CD, 0x7A}, 
{0x32DB, 0x62}, 
{0x32F0, 0x70}, 
{0x3400, 0x08}, 
{0x3400, 0x00}, 
{0x3401, 0x4E}, 
{0x3404, 0x00}, 
{0x3405, 0x00}, 
{0x3410, 0x00}, 
{0x32E0, 0x02}, 
{0x32E1, 0x80}, 
{0x32E2, 0x01}, 
{0x32E3, 0xE0}, 
{0x32E4, 0x00}, 
{0x32E5, 0x80}, 
{0x32E6, 0x00}, 
{0x32E7, 0x80}, 
{0x3200, 0x3E}, 
{0x3201, 0x0F}, 
{0x3028, 0x0F}, 
{0x3029, 0x00}, 
{0x302A, 0x08}, 
{0x3022, 0x24}, 
{0x3023, 0x24}, 
{0x3002, 0x00}, 
{0x3003, 0xA4}, 
{0x3004, 0x00}, 
{0x3005, 0x04}, 
{0x3006, 0x04}, 
{0x3007, 0x63}, 
{0x3008, 0x02}, 
{0x3009, 0xD3}, 
{0x300A, 0x05}, 
{0x300B, 0x3C}, 
{0x300C, 0x02}, 
{0x300D, 0xE0}, 
{0x300E, 0x03}, 
{0x300F, 0xC0}, 
{0x3010, 0x02}, 
{0x3011, 0xD0}, 
{0x32B8, 0x3F}, 
{0x32B9, 0x31}, 
{0x32BB, 0x87}, 
{0x32BC, 0x38}, 
{0x32BD, 0x3C}, 
{0x32BE, 0x34}, 
{0x3201, 0x7F}, 
{0x3021, 0x06},
{0x3025, 0x00}, //normal 
{0x3400, 0x01}, 
{0x3060, 0x01}, 
{REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_framesize_QVGA[][2] = {
//[JPEG_320x240_10.14_10.14_Fps] 
{0x3021, 0x00},
{REG_DLY, 100}, // delay 100ms
{0x32BF, 0x60}, 
{0x32C0, 0x5A}, 
{0x32C1, 0x5A}, 
{0x32C2, 0x5A}, 
{0x32C3, 0x00}, 
{0x32C4, 0x20}, 
{0x32C5, 0x20}, 
{0x32C6, 0x20}, 
{0x32C7, 0x00}, 
{0x32C8, 0x4B}, 
{0x32C9, 0x5A}, 
{0x32CA, 0x7A}, 
{0x32CB, 0x7A}, 
{0x32CC, 0x7A}, 
{0x32CD, 0x7A}, 
{0x32DB, 0x62}, 
{0x32F0, 0x70}, 
{0x3400, 0x08}, 
{0x3400, 0x00}, 
{0x3401, 0x4E}, 
{0x3404, 0x00}, 
{0x3405, 0x00}, 
{0x3410, 0x00}, 
{0x32E0, 0x01}, 
{0x32E1, 0x40}, 
{0x32E2, 0x00}, 
{0x32E3, 0xF0}, 
{0x32E4, 0x02}, 
{0x32E5, 0x02}, 
{0x32E6, 0x02}, 
{0x32E7, 0x03}, 
{0x3200, 0x3E}, 
{0x3201, 0x0F}, 
{0x3028, 0x0F}, 
{0x3029, 0x00}, 
{0x302A, 0x08}, 
{0x3022, 0x24}, 
{0x3023, 0x24}, 
{0x3002, 0x00}, 
{0x3003, 0xA4}, 
{0x3004, 0x00}, 
{0x3005, 0x04}, 
{0x3006, 0x04}, 
{0x3007, 0x63}, 
{0x3008, 0x02}, 
{0x3009, 0xD3}, 
{0x300A, 0x05}, 
{0x300B, 0x3C}, 
{0x300C, 0x02}, 
{0x300D, 0xE0}, 
{0x300E, 0x03}, 
{0x300F, 0xC0}, 
{0x3010, 0x02}, 
{0x3011, 0xD0}, 
{0x32B8, 0x3F}, 
{0x32B9, 0x31}, 
{0x32BB, 0x87}, 
{0x32BC, 0x38}, 
{0x32BD, 0x3C}, 
{0x32BE, 0x34}, 
{0x3201, 0x7F}, 
{0x3021, 0x06},
{0x3025, 0x00}, //normal
{0x3400, 0x01}, 
{0x3060, 0x01}, 
{REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_framesize_VGA_xyskip[][2] = {
// [JPEG_640x360_20.00_25.01_Fps_XY_Skip]
// Set_Device_Format = FORMAT_16_8 
// SET_Device_Addr = 0x54 
{0x3021, 0x00},
{REG_DLY, 100}, // delay 100ms
{0x32BF, 0x60 },
{0x320A, 0xB2 },
{0x32C0, 0x64 },
{0x32C1, 0x64 },
{0x32C2, 0x64 },
{0x32C3, 0x00 },
{0x32C4, 0x20 },
{0x32C5, 0x20 },
{0x32C6, 0x20 },
{0x32C7, 0x00 },
{0x32C8, 0x62 },
{0x32C9, 0x64 },
{0x32CA, 0x84 },
{0x32CB, 0x84 },
{0x32CC, 0x84 },
{0x32CD, 0x84 },
{0x32DB, 0x68 },
{0x32F0, 0x70 },
{0x3400, 0x08 },
{0x3400, 0x00 },
{0x3401, 0x4E },
{0x3404, 0x00 },
{0x3405, 0x00 },
{0x3410, 0x00 },
{0x3200, 0x3E },
{0x3201, 0x0F },
{0x3028, 0x0F },
{0x3029, 0x00 },
{0x302A, 0x08 },
{0x3022, 0x24 },
{0x3023, 0x6C },
{0x3002, 0x00 },
{0x3003, 0x04 },
{0x3004, 0x00 },
{0x3005, 0x04 },
{0x3006, 0x05 },
{0x3007, 0x03 },
{0x3008, 0x02 },
{0x3009, 0xD3 },
{0x300A, 0x03 },
{0x300B, 0xFC },
{0x300C, 0x01 },
{0x300D, 0x88 },
{0x300E, 0x02 },
{0x300F, 0x80 },
{0x3010, 0x01 },
{0x3011, 0x68 },
{0x32B8, 0x3F },
{0x32B9, 0x31 },
{0x32BB, 0x87 },
{0x32BC, 0x38 },
{0x32BD, 0x3C },
{0x32BE, 0x34 },
{0x3201, 0x3F },
{0x3025, 0x00 }, //normal
{0x3021, 0x06 },
{0x3400, 0x01 },
{0x3060, 0x01 },
{REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_framesize_VGA_xskip[][2] = {
//[JPEG_640x480_Xskip_13.32_13.32_Fps]
{0x3021, 0x00},
{REG_DLY, 100}, // delay 100ms
{0x32BF, 0x60}, 
{0x32C0, 0x5A}, 
{0x32C1, 0x5A}, 
{0x32C2, 0x5A}, 
{0x32C3, 0x00}, 
{0x32C4, 0x20}, 
{0x32C5, 0x20}, 
{0x32C6, 0x20}, 
{0x32C7, 0x00}, 
{0x32C8, 0x62}, 
{0x32C9, 0x5A}, 
{0x32CA, 0x7A}, 
{0x32CB, 0x7A}, 
{0x32CC, 0x7A}, 
{0x32CD, 0x7A}, 
{0x32DB, 0x68}, 
{0x32F0, 0x70}, 
{0x3400, 0x08}, 
{0x3400, 0x00}, 
{0x3401, 0x4E}, 
{0x3404, 0x00}, 
{0x3405, 0x00}, 
{0x3410, 0x00}, 
{0x32E0, 0x02}, 
{0x32E1, 0x80}, 
{0x32E2, 0x01}, 
{0x32E3, 0xE0}, 
{0x32E4, 0x00}, 
{0x32E5, 0x00}, 
{0x32E6, 0x00}, 
{0x32E7, 0x80}, 
{0x3200, 0x3E}, 
{0x3201, 0x0F}, 
{0x3028, 0x0F}, 
{0x3029, 0x00}, 
{0x302A, 0x08}, 
{0x3022, 0x24}, 
{0x3023, 0x2C}, 
{0x3002, 0x00}, 
{0x3003, 0x04}, 
{0x3004, 0x00}, 
{0x3005, 0x04}, 
{0x3006, 0x05}, 
{0x3007, 0x03}, 
{0x3008, 0x02}, 
{0x3009, 0xD3}, 
{0x300A, 0x03}, 
{0x300B, 0xFC}, 
{0x300C, 0x02}, 
{0x300D, 0xE0}, 
{0x300E, 0x02}, 
{0x300F, 0x80}, 
{0x3010, 0x02}, 
{0x3011, 0xD0}, 
{0x32B8, 0x3F}, 
{0x32B9, 0x31}, 
{0x32BB, 0x87}, 
{0x32BC, 0x38}, 
{0x32BD, 0x3C}, 
{0x32BE, 0x34}, 
{0x3201, 0x7F}, 
{0x3021, 0x06},
{0x3025, 0x00}, //normal 
{0x3400, 0x01}, 
{0x3060, 0x01}, 
{REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_framesize_QVGA_xskip[][2] = {
{0x3021, 0x00},
{REG_DLY, 100}, // delay 100ms
//[JPEG_320x240_Xskip_13.32_13.32_Fps]
{0x32BF, 0x60}, 
{0x32C0, 0x5A}, 
{0x32C1, 0x5A}, 
{0x32C2, 0x5A}, 
{0x32C3, 0x00}, 
{0x32C4, 0x20}, 
{0x32C5, 0x20}, 
{0x32C6, 0x20}, 
{0x32C7, 0x00}, 
{0x32C8, 0x62}, 
{0x32C9, 0x5A}, 
{0x32CA, 0x7A}, 
{0x32CB, 0x7A}, 
{0x32CC, 0x7A}, 
{0x32CD, 0x7A}, 
{0x32DB, 0x68}, 
{0x32F0, 0x70}, 
{0x3400, 0x08}, 
{0x3400, 0x00}, 
{0x3401, 0x4E}, 
{0x3404, 0x00}, 
{0x3405, 0x00}, 
{0x3410, 0x00}, 
{0x32E0, 0x01}, 
{0x32E1, 0x40}, 
{0x32E2, 0x00}, 
{0x32E3, 0xF0}, 
{0x32E4, 0x01}, 
{0x32E5, 0x01}, 
{0x32E6, 0x02}, 
{0x32E7, 0x03}, 
{0x3200, 0x3E}, 
{0x3201, 0x0F}, 
{0x3028, 0x0F}, 
{0x3029, 0x00}, 
{0x302A, 0x08}, 
{0x3022, 0x24}, 
{0x3023, 0x2C}, 
{0x3002, 0x00}, 
{0x3003, 0x04}, 
{0x3004, 0x00}, 
{0x3005, 0x04}, 
{0x3006, 0x05}, 
{0x3007, 0x03}, 
{0x3008, 0x02}, 
{0x3009, 0xD3}, 
{0x300A, 0x03}, 
{0x300B, 0xFC}, 
{0x300C, 0x02}, 
{0x300D, 0xE0}, 
{0x300E, 0x02}, 
{0x300F, 0x80}, 
{0x3010, 0x02}, 
{0x3011, 0xD0}, 
{0x32B8, 0x3F}, 
{0x32B9, 0x31}, 
{0x32BB, 0x87}, 
{0x32BC, 0x38}, 
{0x32BD, 0x3C}, 
{0x32BE, 0x34}, 
{0x3201, 0x7F}, 
{0x3021, 0x06},
{0x3025, 0x00}, //normal 
{0x3400, 0x01}, 
{0x3060, 0x01},
{REGLIST_TAIL, 0x00}, // tail
};


static const DRAM_ATTR uint16_t sensor_framesize_VGA_crop[][2] = {
//[JPEG_640x480_Crop_19.77_19.77_Fps]
{0x3021, 0x00},
{REG_DLY, 100}, // delay 100ms
{0x32BF, 0x60}, 
{0x32C0, 0x5A}, 
{0x32C1, 0x5A}, 
{0x32C2, 0x5A}, 
{0x32C3, 0x00}, 
{0x32C4, 0x20}, 
{0x32C5, 0x20}, 
{0x32C6, 0x20}, 
{0x32C7, 0x00}, 
{0x32C8, 0x62}, 
{0x32C9, 0x5A}, 
{0x32CA, 0x7A}, 
{0x32CB, 0x7A}, 
{0x32CC, 0x7A}, 
{0x32CD, 0x7A}, 
{0x32DB, 0x68}, 
{0x32F0, 0x70}, 
{0x3400, 0x08}, 
{0x3400, 0x00}, 
{0x3401, 0x4E}, 
{0x3404, 0x00}, 
{0x3405, 0x00}, 
{0x3410, 0x00}, 
{0x3200, 0x3E}, 
{0x3201, 0x0F}, 
{0x3028, 0x0F}, 
{0x3029, 0x00}, 
{0x302A, 0x08}, 
{0x3022, 0x24}, 
{0x3023, 0x24}, 
{0x3002, 0x01}, 
{0x3003, 0x44}, 
{0x3004, 0x00}, 
{0x3005, 0x7C}, 
{0x3006, 0x03}, 
{0x3007, 0xC3}, 
{0x3008, 0x02}, 
{0x3009, 0x5B}, 
{0x300A, 0x03}, 
{0x300B, 0xFC}, 
{0x300C, 0x01}, 
{0x300D, 0xF0}, 
{0x300E, 0x02}, 
{0x300F, 0x80}, 
{0x3010, 0x01}, 
{0x3011, 0xE0}, 
{0x32B8, 0x3F}, 
{0x32B9, 0x31}, 
{0x32BB, 0x87}, 
{0x32BC, 0x38}, 
{0x32BD, 0x3C}, 
{0x32BE, 0x34}, 
{0x3201, 0x3F}, 
{0x3021, 0x06},
{0x3025, 0x00}, //normal 
{0x3400, 0x01}, 
{0x3060, 0x01}, 
{REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_framesize_QVGA_crop[][2] = {
//[JPEG_320x240_Crop_19.77_19.77_Fps]
{0x3021, 0x00},
{REG_DLY, 100}, // delay 100ms
{0x32BF, 0x60}, 
{0x32C0, 0x5A}, 
{0x32C1, 0x5A}, 
{0x32C2, 0x5A}, 
{0x32C3, 0x00}, 
{0x32C4, 0x20}, 
{0x32C5, 0x20}, 
{0x32C6, 0x20}, 
{0x32C7, 0x00}, 
{0x32C8, 0x62}, 
{0x32C9, 0x5A}, 
{0x32CA, 0x7A}, 
{0x32CB, 0x7A}, 
{0x32CC, 0x7A}, 
{0x32CD, 0x7A}, 
{0x32DB, 0x68}, 
{0x32F0, 0x70}, 
{0x3400, 0x08}, 
{0x3400, 0x00}, 
{0x3401, 0x4E}, 
{0x3404, 0x00}, 
{0x3405, 0x00}, 
{0x3410, 0x00}, 
{0x32E0, 0x01}, 
{0x32E1, 0x40}, 
{0x32E2, 0x00}, 
{0x32E3, 0xF0}, 
{0x32E4, 0x01}, 
{0x32E5, 0x01}, 
{0x32E6, 0x01}, 
{0x32E7, 0x02}, 
{0x3200, 0x3E}, 
{0x3201, 0x0F}, 
{0x3028, 0x0F}, 
{0x3029, 0x00}, 
{0x302A, 0x08}, 
{0x3022, 0x24}, 
{0x3023, 0x24}, 
{0x3002, 0x01}, 
{0x3003, 0x44}, 
{0x3004, 0x00}, 
{0x3005, 0x7C}, 
{0x3006, 0x03}, 
{0x3007, 0xC3}, 
{0x3008, 0x02}, 
{0x3009, 0x5B}, 
{0x300A, 0x03}, 
{0x300B, 0xFC}, 
{0x300C, 0x01}, 
{0x300D, 0xF0}, 
{0x300E, 0x02}, 
{0x300F, 0x80}, 
{0x3010, 0x01}, 
{0x3011, 0xE0}, 
{0x32B8, 0x3F}, 
{0x32B9, 0x31}, 
{0x32BB, 0x87}, 
{0x32BC, 0x38}, 
{0x32BD, 0x3C}, 
{0x32BE, 0x34}, 
{0x3201, 0x7F}, 
{0x3021, 0x06},
{0x3025, 0x00}, //normal 
{0x3400, 0x01}, 
{0x3060, 0x01}, 
{REGLIST_TAIL, 0x00}, // tail
};

#endif


