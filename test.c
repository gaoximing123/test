/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * V4.0
 */
#define LOG_TAG "sensor_gc5024_pike"
#include <utils/Log.h>
#include "sensor.h"
#include "jpeg_exif_header.h"
#include "sensor_drv_u.h"
#include "sensor_raw.h"

#if defined(CONFIG_CAMERA_ISP_VERSION_V3) || defined(CONFIG_CAMERA_ISP_VERSION_V4)
#include "sensor_gc5024_raw_param_main.c"
#else
#include "sensor_gc5024_raw_param.c"
#endif

#ifndef CONFIG_CAMERA_AUTOFOCUS_NOT_SUPPORT
#include "../af_dw9714.h"
static uint16_t cur_pos = 0;
#endif

//#define FEATURE_OTP    /*OTP function switch*/
#if defined(FEATURE_OTP)
#define SENSOR_NAME					"gc5024 otp"
#else
#define SENSOR_NAME					"gc5024"
#endif
#define I2C_SLAVE_ADDR				0x6e    /* 8bit slave address*/
 
#define GC5024_PID_ADDR				0xf0
#define GC5024_PID_VALUE			0x50
#define GC5024_VER_ADDR				0xf1
#define GC5024_VER_VALUE			0x24

/* sensor parameters begin */
/* effective sensor output image size */
#define SNAPSHOT_WIDTH				2592
#define SNAPSHOT_HEIGHT				1944
#define PREVIEW_WIDTH				2592
#define PREVIEW_HEIGHT				1944

/*Mipi output*/
#define LANE_NUM					2
#define RAW_BITS					10

#define SNAPSHOT_MIPI_PER_LANE_BPS	864  //720
#define PREVIEW_MIPI_PER_LANE_BPS	864

/*line time unit: 0.1us*/
#define SNAPSHOT_LINE_TIME		168
#define PREVIEW_LINE_TIME		168     //222
/* frame length*/
#define SNAPSHOT_FRAME_LENGTH		1984
#define PREVIEW_FRAME_LENGTH		1984

/* please ref your spec */
#define FRAME_OFFSET				0
#define SENSOR_MAX_GAIN				0xabcd
#define SENSOR_BASE_GAIN			0x40
#define SENSOR_MIN_SHUTTER			1

/* please ref your spec
 * 1 : average binning
 * 2 : sum-average binning
 * 4 : sum binning
 */
#define BINNING_FACTOR				1

/* please ref spec
 * 1: sensor auto caculate
 * 0: driver caculate
 */
#define SUPPORT_AUTO_FRAME_LENGTH	1

/*delay 1 frame to write sensor gain*/
//#define GAIN_DELAY_1_FRAME
#ifdef GAIN_DELAY_1_FRAME
static uint16_t cur_mode
#endif
/* sensor parameters end */

/* isp parameters, please don't change it*/
#if defined(CONFIG_CAMERA_ISP_VERSION_V3) || defined(CONFIG_CAMERA_ISP_VERSION_V4)
#define ISP_BASE_GAIN			0x80
#else
#define ISP_BASE_GAIN				0x10
#endif
/* please don't change it */
#define EX_MCLK						24
#define GC5024_RAW_PARAM_COM		0x0000
//#define IMAGE_NORMAL 
#define IMAGE_H_MIRROR 
//#define IMAGE_V_MIRROR 
//#define IMAGE_HV_MIRROR 

#ifdef IMAGE_NORMAL
#define MIRROR 		0xd4
#define PH_SWITCH 	0x1b
#define STARTY 		0x02
#define STARTX 		0x0c
#endif

#ifdef IMAGE_H_MIRROR
#define MIRROR 		0xd5
#define PH_SWITCH 	0x1a
#define STARTY 		0x02
#define STARTX 		0x01
#endif

#ifdef IMAGE_V_MIRROR
#define MIRROR 		0xd6
#define PH_SWITCH 	0x1b
#define STARTY 		0x03
#define STARTX 		0x0c
#endif

#ifdef IMAGE_HV_MIRROR
#define MIRROR 		0xd7
#define PH_SWITCH 	0x1a
#define STARTY 		0x03
#define STARTX 		0x01
#endif


struct hdr_info_t {
	uint32_t capture_max_shutter;
	uint32_t capture_shutter;
	uint32_t capture_gain;
};

struct sensor_ev_info_t {
	uint16_t preview_shutter;
	uint16_t preview_gain;
};

/*==============================================================================
 * Description:
 * global variable
 *============================================================================*/
static struct hdr_info_t s_hdr_info={
	2000000/SNAPSHOT_LINE_TIME,/*min 5fps*/
	SNAPSHOT_FRAME_LENGTH-FRAME_OFFSET,
	SENSOR_BASE_GAIN
	};
static uint32_t s_current_default_frame_length=PREVIEW_FRAME_LENGTH;
static struct sensor_ev_info_t s_sensor_ev_info={
	PREVIEW_FRAME_LENGTH-FRAME_OFFSET,
	SENSOR_BASE_GAIN
	};



#ifdef FEATURE_OTP

#include "sensor_gc5024_gcore_otp.c"

struct raw_param_info_tab s_gc5024_raw_param_tab[] = {
	{GC5024_RAW_PARAM_COM, &s_gc5024_mipi_raw_info, gc5024_gcore_identify_otp, gc5024_gcore_update_otp},
	{RAW_INFO_END_ID, PNULL, PNULL, PNULL}
};
static struct otp_info_t s_gc5024_gcore_otp_info={0x00};
static struct otp_info_t *s_gc5024_otp_info_ptr=&s_gc5024_gcore_otp_info;
static struct raw_param_info_tab *s_gc5024_raw_param_tab_ptr=&s_gc5024_raw_param_tab;  /*otp function interface*/
#else
struct raw_param_info_tab s_gc5024_raw_param_tab[] = {
	{GC5024_RAW_PARAM_COM, &s_gc5024_mipi_raw_info, PNULL, PNULL},
	{RAW_INFO_END_ID, PNULL, PNULL, PNULL}
};

#endif

static SENSOR_IOCTL_FUNC_TAB_T s_gc5024_ioctl_func_tab;
static struct sensor_raw_info *s_gc5024_mipi_raw_info_ptr = &s_gc5024_mipi_raw_info;
static EXIF_SPEC_PIC_TAKING_COND_T s_gc5024_exif_info;

static const SENSOR_REG_T gc5024_init_setting[] = {
	/* SYS */
	{0xfe,0x00},
	{0xfe,0x00},
	{0xfe,0x00},
	{0xf7,0x01},
	{0xf8,0x11},
	{0xf9,0xae},
	{0xfa,0x84},
	{0xfc,0xae},	
	{0xfe,0x00},
	{0xfe,0x00},
	{0xfe,0x00},	
	{0x88,0x03}, 
	{0xe7,0xc0},
	
	/* Analog */
	{0xfe,0x00},
	{0x03,0x06},
	{0x04,0xfc},
	{0x05,0x01},
	{0x06,0xc5},
	{0x07,0x00},
	{0x08,0x08},
	{0x0a,0x00},
	{0x0c,0x00},
	{0x0d,0x07},
	{0x0e,0xa8},
	{0x0f,0x0a},
	{0x10,0x40},	
	{0x11,0x31},
	{0x12,0x26},
	{0x13,0x10},
	{0x17,MIRROR},//Don't Change Here!!!
	{0x18,0x02},
	{0x19,0x0d},
	{0x1a,PH_SWITCH},//Don't Change Here!!!
	{0x1b,0x11},
	{0x1c,0x2c},
	{0x1e,0x50},
	{0x1f,0x70}, 
	{0x20,0x86},
	{0x21,0x0b},
	{0x24,0xb0},
	{0x29,0x3b},
	{0x2d,0x16},
	{0x2f,0x77}, 
	{0x32,0x49},
	{0xcd,0xaa},
	{0xd0,0xd2},
	{0xd1,0xd4},
	{0xd2,0xcb},
	{0xd3,0x33},//73    change
	{0xd8,0x18},
	{0xd9,0x70},
	{0xda,0x03},
	{0xdb,0x30},
	{0xdc,0xba},	
	{0xe2,0x20},
	{0xe4,0x70},
	{0xe6,0x08},
	{0xe8,0x03},
	{0xe9,0x02},
	{0xea,0x03},
	{0xeb,0x03},
	{0xec,0x03},
	{0xed,0x02},
	{0xee,0x03},
	{0xef,0x03},
	
	/* ISP */
	{0x80,0x50},
	{0x8d,0x07},
	{0x90,0x01},
	{0x92,STARTY},//Don't Change Here!!!
	{0x94,STARTX},//Don't Change Here!!!
	{0x95,0x07},
	{0x96,0x98},
	{0x97,0x0a},
	{0x98,0x20},
	
	/* Gain */
	{0x99,0x01},
	{0x9a,0x02},
	{0x9b,0x03},
	{0x9c,0x04},
	{0x9d,0x0d},
	{0x9e,0x15},
	{0x9f,0x1d},
	{0xb0,0x4b},
	{0xb1,0x01},
	{0xb2,0x00},
	{0xb6,0x00},
	
	/* BLK */
	{0x40,0x22},
	{0x4e,0x3c},//Don't Change Here!!!
	{0x4f,0x00},//Don't Change Here!!!
	{0x60,0x00},
	{0x61,0x80},
	{0xfe,0x02},
	{0xa4,0x30},
	{0xa5,0x00},
	
	/* Dark Sun */
	{0x40,0x96},
	{0x42,0x0f},
	{0x45,0xca},
	{0x47,0xff},
	{0x48,0xc8},
	
	/* DD */	
	{0x80,0x90},
	{0x81,0x50},
	{0x82,0x60},
	{0x84,0x20},
	{0x85,0x10},
	{0x86,0x04},
	{0x87,0x20},//20  2016.11.21   
	{0x88,0x10},
	{0x89,0x04},
	
	/*ASDE*/
	{0xa0,0x30},
	{0xa3,0xac},
	
	/* Degrid */
	{0x8a,0x00},
	
	/* MIPI */
	{0xfe,0x03},
	{0x10,0x00},	//Stream off	
	{0x01,0x07},
	{0x02,0x34},
	{0x03,0x13},
	{0x04,0x04},
	{0x05,0x00},
	{0x06,0x80},
	{0x11,0x2b},
	{0x12,0xa8},
	{0x13,0x0c},
	{0x15,0x00},
	{0x16,0x09},
	{0x18,0x01},
	{0x21,0x10},
	{0x22,0x05},
	{0x23,0x30},
	{0x24,0x10},
	{0x25,0x16},  //14
	{0x26,0x08},
	{0x29,0x06},  //05
	{0x2a,0x0a},
	{0x2b,0x08},
	{0x42,0x20},
	{0x43,0x0a},
	{0xfe,0x00},
};

static const SENSOR_REG_T gc5024_preview_setting[] = {
};

static const SENSOR_REG_T gc5024_snapshot_setting[] = {	
};

static SENSOR_REG_TAB_INFO_T s_gc5024_resolution_tab_raw[SENSOR_MODE_MAX] = {
	{ADDR_AND_LEN_OF_ARRAY(gc5024_init_setting), 0, 0, EX_MCLK,
	 SENSOR_IMAGE_FORMAT_RAW},
	{ADDR_AND_LEN_OF_ARRAY(gc5024_preview_setting),
	 PREVIEW_WIDTH, PREVIEW_HEIGHT, EX_MCLK,
	 SENSOR_IMAGE_FORMAT_RAW},
	{ADDR_AND_LEN_OF_ARRAY(gc5024_snapshot_setting),
	 SNAPSHOT_WIDTH, SNAPSHOT_HEIGHT, EX_MCLK,
	 SENSOR_IMAGE_FORMAT_RAW},
};

static SENSOR_TRIM_T s_gc5024_resolution_trim_tab[SENSOR_MODE_MAX] = {
	{0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0}},
	{0, 0, PREVIEW_WIDTH, PREVIEW_HEIGHT,
	 PREVIEW_LINE_TIME, PREVIEW_MIPI_PER_LANE_BPS, PREVIEW_FRAME_LENGTH,
	 {0, 0, PREVIEW_WIDTH, PREVIEW_HEIGHT}},
	{0, 0, SNAPSHOT_WIDTH, SNAPSHOT_HEIGHT,
	 SNAPSHOT_LINE_TIME, SNAPSHOT_MIPI_PER_LANE_BPS, SNAPSHOT_FRAME_LENGTH,
	 {0, 0, SNAPSHOT_WIDTH, SNAPSHOT_HEIGHT}},
};

static const SENSOR_REG_T s_gc5024_preview_size_video_tab[SENSOR_VIDEO_MODE_MAX][1] = {
	/*video mode 0: ?fps */
	{
	 {0xffff, 0xff}
	 },
	/* video mode 1:?fps */
	{
	 {0xffff, 0xff}
	 },
	/* video mode 2:?fps */
	{
	 {0xffff, 0xff}
	 },
	/* video mode 3:?fps */
	{
	 {0xffff, 0xff}
	 }
};

static const SENSOR_REG_T s_gc5024_capture_size_video_tab[SENSOR_VIDEO_MODE_MAX][1] = {
	/*video mode 0: ?fps */
	{
	 {0xffff, 0xff}
	 },
	/* video mode 1:?fps */
	{
	 {0xffff, 0xff}
	 },
	/* video mode 2:?fps */
	{
	 {0xffff, 0xff}
	 },
	/* video mode 3:?fps */
	{
	 {0xffff, 0xff}
	 }
};

static SENSOR_VIDEO_INFO_T s_gc5024_video_info[SENSOR_MODE_MAX] = {
	{{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, PNULL},
	{{{30, 30, 270, 90}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
	 (SENSOR_REG_T **) s_gc5024_preview_size_video_tab},
	{{{2, 5, 167, 1000}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
	 (SENSOR_REG_T **) s_gc5024_capture_size_video_tab},
};

/*==============================================================================
 * Description:
 * set video mode
 *
 *============================================================================*/
static uint32_t gc5024_set_video_mode(uint32_t param)
{
	SENSOR_REG_T_PTR sensor_reg_ptr;
	uint16_t i = 0x00;
	uint32_t mode;

	if (param >= SENSOR_VIDEO_MODE_MAX)
		return 0;

	if (SENSOR_SUCCESS != Sensor_GetMode(&mode)) {
		SENSOR_PRINT("fail.");
		return SENSOR_FAIL;
	}

	if (PNULL == s_gc5024_video_info[mode].setting_ptr) {
		SENSOR_PRINT("fail.");
		return SENSOR_FAIL;
	}

	sensor_reg_ptr = (SENSOR_REG_T_PTR) & s_gc5024_video_info[mode].setting_ptr[param];
	if (PNULL == sensor_reg_ptr) {
		SENSOR_PRINT("fail.");
		return SENSOR_FAIL;
	}

	for (i = 0x00; (0xffff != sensor_reg_ptr[i].reg_addr)
	     || (0xff != sensor_reg_ptr[i].reg_value); i++) {
		Sensor_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
	}

	return 0;
}

/*==============================================================================
 * Description:
 * sensor all info
 * please modify this variable acording your spec
 *============================================================================*/
SENSOR_INFO_T g_gc5024_mipi_raw_info = {
	/* salve i2c write address */
	(I2C_SLAVE_ADDR >> 1),
	/* salve i2c read address */
	(I2C_SLAVE_ADDR >> 1),
	/*bit0: 0: i2c register value is 8 bit, 1: i2c register value is 16 bit */
	SENSOR_I2C_REG_8BIT | SENSOR_I2C_VAL_8BIT | SENSOR_I2C_FREQ_400,
	/* bit2: 0:negative; 1:positive -> polarily of horizontal synchronization signal
	 * bit4: 0:negative; 1:positive -> polarily of vertical synchronization signal
	 * other bit: reseved
	 */
	SENSOR_HW_SIGNAL_PCLK_P | SENSOR_HW_SIGNAL_VSYNC_P | SENSOR_HW_SIGNAL_HSYNC_P,
	/* preview mode */
	SENSOR_ENVIROMENT_NORMAL | SENSOR_ENVIROMENT_NIGHT,
	/* image effect */
	SENSOR_IMAGE_EFFECT_NORMAL |
	    SENSOR_IMAGE_EFFECT_BLACKWHITE |
	    SENSOR_IMAGE_EFFECT_RED |
	    SENSOR_IMAGE_EFFECT_GREEN | SENSOR_IMAGE_EFFECT_BLUE | SENSOR_IMAGE_EFFECT_YELLOW |
	    SENSOR_IMAGE_EFFECT_NEGATIVE | SENSOR_IMAGE_EFFECT_CANVAS,

	/* while balance mode */
	0,
	/* bit[0:7]: count of step in brightness, contrast, sharpness, saturation
	 * bit[8:31] reseved
	 */
	7,
	/* reset pulse level */
	SENSOR_LOW_PULSE_RESET,
	/* reset pulse width(ms) */
	20,
	/* 1: high level valid; 0: low level valid */
	SENSOR_HIGH_LEVEL_PWDN,
	/* count of identify code */
	1,
	/* supply two code to identify sensor.
	 * for Example: index = 0-> Device id, index = 1 -> version id
	 * customer could ignore it.
	 */
	{{GC5024_PID_ADDR, GC5024_PID_VALUE}
	 ,
	 {GC5024_VER_ADDR, GC5024_VER_VALUE}
	 }
	,
	/* voltage of avdd */
	SENSOR_AVDD_2800MV,
	/* max width of source image */
	SNAPSHOT_WIDTH,
	/* max height of source image */
	SNAPSHOT_HEIGHT,
	/* name of sensor */
	SENSOR_NAME,
	/* define in SENSOR_IMAGE_FORMAT_E enum,SENSOR_IMAGE_FORMAT_MAX
	 * if set to SENSOR_IMAGE_FORMAT_MAX here,
	 * image format depent on SENSOR_REG_TAB_INFO_T
	 */
	SENSOR_IMAGE_FORMAT_RAW,
	/*  pattern of input image form sensor */
	SENSOR_IMAGE_PATTERN_RAWRGB_R,
	/* point to resolution table information structure */
	s_gc5024_resolution_tab_raw,
	/* point to ioctl function table */
	&s_gc5024_ioctl_func_tab,
	/* information and table about Rawrgb sensor */
	&s_gc5024_mipi_raw_info_ptr,
	/* extend information about sensor
	 * like &g_gc5024_ext_info
	 */
	NULL,
	/* voltage of iovdd */
	SENSOR_AVDD_1800MV,
	/* voltage of dvdd */
	SENSOR_AVDD_1500MV,
	/* skip frame num before preview */
	3,										//      3
	/* skip frame num before capture */
	1,
	/* deci frame num during preview */
	0,
	/* deci frame num during video preview */
	0,
	0,
	0,
	0,
	0,
	0,
	{SENSOR_INTERFACE_TYPE_CSI2, LANE_NUM, RAW_BITS, 0}
	,
	0,
	/* skip frame num while change setting */
	1,
	/* horizontal  view angle*/
	65,
	/* vertical view angle*/
	60
};

#if defined(CONFIG_CAMERA_ISP_VERSION_V3) || defined(CONFIG_CAMERA_ISP_VERSION_V4)

#define param_update(x1,x2) sprintf(name,"/data/misc/cameraserver/gc5024_%s.bin",x1);\
				if(0==access(name,R_OK))\
				{\
					FILE* fp = NULL;\
					SENSOR_PRINT("param file %s exists",name);\
					if( NULL!=(fp=fopen(name,"rb")) ){\
						fread((void*)x2,1,sizeof(x2),fp);\
						fclose(fp);\
					}else{\
						SENSOR_PRINT("param open %s failure",name);\
					}\
				}else{\
					SENSOR_PRINT("access %s failure",name);\
				}\
				memset(name,0,sizeof(name))

static uint32_t gc5024_InitRawTuneInfo(void)
{
	uint32_t rtn=0x00;

	isp_raw_para_update_from_file(&g_gc5024_mipi_raw_info,0);

	struct sensor_raw_info* raw_sensor_ptr=s_gc5024_mipi_raw_info_ptr;
	struct isp_mode_param* mode_common_ptr = raw_sensor_ptr->mode_ptr[0].addr;
	int i;
	char name[100] = {'\0'};

	for (i=0; i<mode_common_ptr->block_num; i++) {
		struct isp_block_header* header = &(mode_common_ptr->block_header[i]);
		uint8_t* data = (uint8_t*)mode_common_ptr + header->offset;
		switch (header->block_id)
		{
		case	ISP_BLK_PRE_WAVELET_V1: {
				/* modify block data */
				struct sensor_pwd_param* block = (struct sensor_pwd_param*)data;

				static struct sensor_pwd_level pwd_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/pwd_param.h"
				};

				param_update("pwd_param",pwd_param);

				block->param_ptr = pwd_param;
			}
			break;

		case	ISP_BLK_BPC_V1: {
				/* modify block data */
				struct sensor_bpc_param_v1* block = (struct sensor_bpc_param_v1*)data;

				static struct sensor_bpc_level bpc_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/bpc_param.h"
				};

				param_update("bpc_param",bpc_param);

				block->param_ptr = bpc_param;
			}
			break;

		case	ISP_BLK_BL_NR_V1: {
				/* modify block data */
				struct sensor_bdn_param* block = (struct sensor_bdn_param*)data;

				static struct sensor_bdn_level bdn_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/bdn_param.h"
				};

				param_update("bdn_param",bdn_param);

				block->param_ptr = bdn_param;
			}
			break;

		case	ISP_BLK_GRGB_V1: {
				/* modify block data */
				struct sensor_grgb_v1_param* block = (struct sensor_grgb_v1_param*)data;
				static struct sensor_grgb_v1_level grgb_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/grgb_param.h"
				};

				param_update("grgb_param",grgb_param);

				block->param_ptr = grgb_param;

			}
			break;

		case	ISP_BLK_NLM: {
				/* modify block data */
				struct sensor_nlm_param* block = (struct sensor_nlm_param*)data;

				static struct sensor_nlm_level nlm_param[32] = {
					#include "NR/nlm_param.h"
				};

				param_update("nlm_param",nlm_param);

				static struct sensor_vst_level vst_param[32] = {
					#include "NR/vst_param.h"
				};

				param_update("vst_param",vst_param);

				static struct sensor_ivst_level ivst_param[32] = {
					#include "NR/ivst_param.h"
				};

				param_update("ivst_param",ivst_param);

				static struct sensor_flat_offset_level flat_offset_param[32] = {
					#include "NR/flat_offset_param.h"
				};

				param_update("flat_offset_param",flat_offset_param);

				block->param_nlm_ptr = nlm_param;
				block->param_vst_ptr = vst_param;
				block->param_ivst_ptr = ivst_param;
				block->param_flat_offset_ptr = flat_offset_param;
			}
			break;

		case	ISP_BLK_CFA_V1: {
				/* modify block data */
				struct sensor_cfa_param_v1* block = (struct sensor_cfa_param_v1*)data;
				static struct sensor_cfae_level cfae_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/cfae_param.h"
				};

				param_update("cfae_param",cfae_param);

				block->param_ptr = cfae_param;
			}
			break;

		case	ISP_BLK_RGB_PRECDN: {
				/* modify block data */
				struct sensor_rgb_precdn_param* block = (struct sensor_rgb_precdn_param*)data;

				static struct sensor_rgb_precdn_level precdn_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/rgb_precdn_param.h"
				};

				param_update("rgb_precdn_param",precdn_param);

				block->param_ptr = precdn_param;
			}
			break;

		case	ISP_BLK_YUV_PRECDN: {
				/* modify block data */
				struct sensor_yuv_precdn_param* block = (struct sensor_yuv_precdn_param*)data;

				static struct sensor_yuv_precdn_level yuv_precdn_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/yuv_precdn_param.h"
				};

				param_update("yuv_precdn_param",yuv_precdn_param);

				block->param_ptr = yuv_precdn_param;
			}
			break;

		case	ISP_BLK_PREF_V1: {
				/* modify block data */
				struct sensor_prfy_param* block = (struct sensor_prfy_param*)data;

				static struct sensor_prfy_level prfy_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/prfy_param.h"
				};

				param_update("prfy_param",prfy_param);

				block->param_ptr = prfy_param;
			}
			break;

		case	ISP_BLK_UV_CDN: {
				/* modify block data */
				struct sensor_uv_cdn_param* block = (struct sensor_uv_cdn_param*)data;

				static struct sensor_uv_cdn_level uv_cdn_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/yuv_cdn_param.h"
				};

				param_update("yuv_cdn_param",uv_cdn_param);

				block->param_ptr = uv_cdn_param;
			}
			break;

		case	ISP_BLK_EDGE_V1: {
				/* modify block data */
				struct sensor_ee_param* block = (struct sensor_ee_param*)data;

				static struct sensor_ee_level edge_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/edge_param.h"
				};

				param_update("edge_param",edge_param);

				block->param_ptr = edge_param;
			}
			break;

		case	ISP_BLK_UV_POSTCDN: {
				/* modify block data */
				struct sensor_uv_postcdn_param* block = (struct sensor_uv_postcdn_param*)data;

				static struct sensor_uv_postcdn_level uv_postcdn_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/yuv_postcdn_param.h"
				};

				param_update("yuv_postcdn_param",uv_postcdn_param);

				block->param_ptr = uv_postcdn_param;
			}
			break;

		case	ISP_BLK_IIRCNR_IIR: {
				/* modify block data */
				struct sensor_iircnr_param* block = (struct sensor_iircnr_param*)data;

				static struct sensor_iircnr_level iir_cnr_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/iircnr_param.h"
				};

				param_update("iircnr_param",iir_cnr_param);

				block->param_ptr = iir_cnr_param;
			}
			break;

		case	ISP_BLK_IIRCNR_YRANDOM: {
				/* modify block data */
				struct sensor_iircnr_yrandom_param* block = (struct sensor_iircnr_yrandom_param*)data;
				static struct sensor_iircnr_yrandom_level iir_yrandom_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/iir_yrandom_param.h"
				};

				param_update("iir_yrandom_param",iir_yrandom_param);

				block->param_ptr = iir_yrandom_param;
			}
			break;

		case  ISP_BLK_UVDIV_V1: {
				/* modify block data */
				struct sensor_cce_uvdiv_param_v1* block = (struct sensor_cce_uvdiv_param_v1*)data;

				static struct sensor_cce_uvdiv_level cce_uvdiv_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/cce_uv_param.h"
				};

				param_update("cce_uv_param",cce_uvdiv_param);

				block->param_ptr = cce_uvdiv_param;
			}
			break;
		case ISP_BLK_YIQ_AFM:{
			/* modify block data */
			struct sensor_y_afm_param *block = (struct sensor_y_afm_param*)data;

			static struct sensor_y_afm_level y_afm_param[SENSOR_SMART_LEVEL_NUM] = {
					#include "NR/y_afm_param.h"
				};

				param_update("y_afm_param",y_afm_param);

				block->param_ptr = y_afm_param;
			}
			break;

		default:
			break;
		}
	}


	return rtn;
}
#endif

/*==============================================================================
 * Description:
 * get default frame length
 *
 *============================================================================*/
static uint32_t gc5024_get_default_frame_length(uint32_t mode)
{
	return s_gc5024_resolution_trim_tab[mode].frame_line;
}


/*==============================================================================
 * Description:
 * read gain from sensor registers
 * please modify this function acording your spec
 *============================================================================*/
static uint16_t gc5024_read_gain(void)
{
	return s_sensor_ev_info.preview_gain;
}

/*==============================================================================
 * Description:
 * write gain to sensor registers
 * please modify this function acording your spec
 *============================================================================*/
 
#define ANALOG_GAIN_1 64   // 1.000x
#define ANALOG_GAIN_2 88   // 1.375x
#define ANALOG_GAIN_3 121  // 1.891x 
#define ANALOG_GAIN_4 168  // 2.625x
#define ANALOG_GAIN_5 239  // 3.734x
#define ANALOG_GAIN_6 336  // 5.250x
#define ANALOG_GAIN_7 481  // 7.516x


static void gc5024_write_gain(uint32_t gain)
{
	uint16_t temp=0x00;
	
	Sensor_WriteReg(0xfe,0x00);
    if (SENSOR_MAX_GAIN < gain)
        gain = SENSOR_MAX_GAIN;
	if(gain < 0x40)
		gain = 0x40;
	if((ANALOG_GAIN_1<= gain)&&(gain < ANALOG_GAIN_2))
	{	
		Sensor_WriteReg(0xfe, 0x00);		
		Sensor_WriteReg(0x29, 0x1c); 
		Sensor_WriteReg(0xd8, 0x10); 
		Sensor_WriteReg(0xe8, 0x02); 
		Sensor_WriteReg(0xe9, 0x01); 
		Sensor_WriteReg(0xec, 0x02); 
		Sensor_WriteReg(0xed, 0x01); 	
		//analog gain

		Sensor_WriteReg(0xb6, 0x00);
		temp = gain;
		Sensor_WriteReg(0xb1, temp>>6);
		Sensor_WriteReg(0xb2, (temp<<2)&0xfc);
	}
	else if((ANALOG_GAIN_2<= gain)&&(gain < ANALOG_GAIN_3))
	{	
		Sensor_WriteReg(0xfe, 0x00);		
		Sensor_WriteReg(0x29, 0x1f); 
		Sensor_WriteReg(0xd8, 0x10); 
		Sensor_WriteReg(0xe8, 0x02); 
		Sensor_WriteReg(0xe9, 0x01); 
		Sensor_WriteReg(0xec, 0x02); 
		Sensor_WriteReg(0xed, 0x01); 	
		//analog gain

		Sensor_WriteReg(0xb6, 0x01);
		temp = 64*gain/ANALOG_GAIN_2;
		Sensor_WriteReg(0xb1, temp>>6);
		Sensor_WriteReg(0xb2, (temp<<2)&0xfc);
	}
	else if((ANALOG_GAIN_3<= gain)&&(gain < ANALOG_GAIN_4))
	{
		Sensor_WriteReg(0xfe, 0x00);		
		Sensor_WriteReg(0x29, 0x22); 
		Sensor_WriteReg(0xd8, 0x10); 
		Sensor_WriteReg(0xe8, 0x02); 
		Sensor_WriteReg(0xe9, 0x02); 
		Sensor_WriteReg(0xec, 0x02); 
		Sensor_WriteReg(0xed, 0x02); 	
		//analog gain
		Sensor_WriteReg(0xb6, 0x02);
		temp = 64*gain/ANALOG_GAIN_3;
		Sensor_WriteReg(0xb1, temp>>6);
		Sensor_WriteReg(0xb2, (temp<<2)&0xfc);
	}
	else if((ANALOG_GAIN_4<= gain)&&(gain < ANALOG_GAIN_5))
	{
		Sensor_WriteReg(0xfe, 0x00);
		Sensor_WriteReg(0x29, 0x3b); 
		Sensor_WriteReg(0xd8, 0x18); 
		Sensor_WriteReg(0xe8, 0x03); 
		Sensor_WriteReg(0xe9, 0x02); 
		Sensor_WriteReg(0xec, 0x03); 
		Sensor_WriteReg(0xed, 0x02); 
		//analog gain
		Sensor_WriteReg(0xb6, 0x03);
		temp = 64*gain/ANALOG_GAIN_4;
		Sensor_WriteReg(0xb1, temp>>6);
		Sensor_WriteReg(0xb2, (temp<<2)&0xfc);
	}
	else if((ANALOG_GAIN_5<= gain)&&(gain < ANALOG_GAIN_6))
	{
		Sensor_WriteReg(0xfe, 0x00);		
		Sensor_WriteReg(0x29, 0x3b); 
		Sensor_WriteReg(0xd8, 0x18); 
		Sensor_WriteReg(0xe8, 0x03); 
		Sensor_WriteReg(0xe9, 0x02); 
		Sensor_WriteReg(0xec, 0x03); 
		Sensor_WriteReg(0xed, 0x02); 
		//analog gain
		Sensor_WriteReg(0xb6, 0x04);
		temp = 64*gain/ANALOG_GAIN_5;
		Sensor_WriteReg(0xb1, temp>>6);
		Sensor_WriteReg(0xb2, (temp<<2)&0xfc);
	}
	else if((ANALOG_GAIN_6<= gain)&&(gain < ANALOG_GAIN_7))
	{
		Sensor_WriteReg(0xfe, 0x00);		
		Sensor_WriteReg(0x29, 0x3b); 
		Sensor_WriteReg(0xd8, 0x18); 
		Sensor_WriteReg(0xe8, 0x03); 
		Sensor_WriteReg(0xe9, 0x02); 
		Sensor_WriteReg(0xec, 0x03); 
		Sensor_WriteReg(0xed, 0x02); 
		//analog gain
		Sensor_WriteReg(0xb6, 0x05);
		temp = 64*gain/ANALOG_GAIN_6;
		Sensor_WriteReg(0xb1, temp>>6);
		Sensor_WriteReg(0xb2, (temp<<2)&0xfc);
	}
	else
	{
		Sensor_WriteReg(0xfe, 0x00);		
		Sensor_WriteReg(0x29, 0x3b); 
		Sensor_WriteReg(0xd8, 0x18); 
		Sensor_WriteReg(0xe8, 0x03); 
		Sensor_WriteReg(0xe9, 0x02); 
		Sensor_WriteReg(0xec, 0x03); 
		Sensor_WriteReg(0xed, 0x02); 
		//analog gain
		Sensor_WriteReg(0xb6, 0x06);
		temp = 64*gain/ANALOG_GAIN_7;
		Sensor_WriteReg(0xb1, temp>>6);
		Sensor_WriteReg(0xb2, (temp<<2)&0xfc);
	}
}

/*==============================================================================
 * Description:
 * read frame length from sensor registers
 * please modify this function acording your spec
 *============================================================================*/
static uint16_t gc5024_read_frame_length(void)
{
	uint16_t frame_len_h = 0;
	uint16_t frame_len_l = 0;

//	frame_len_h = Sensor_ReadReg(0xYYYY) & 0xff;
//	frame_len_l = Sensor_ReadReg(0xYYYY) & 0xff;

	return ((frame_len_h << 8) | frame_len_l);
}

/*==============================================================================
 * Description:
 * write frame length to sensor registers
 * please modify this function acording your spec
 *============================================================================*/
static void gc5024_write_frame_length(uint32_t frame_len)
{
//	Sensor_WriteReg(0xYYYY, (frame_len >> 8) & 0xff);
//	Sensor_WriteReg(0xYYYY, frame_len & 0xff);
}

/*==============================================================================
 * Description:
 * read shutter from sensor registers
 * please modify this function acording your spec
 *============================================================================*/
static uint32_t gc5024_read_shutter(void)
{
	return s_sensor_ev_info.preview_shutter;
}

/*==============================================================================
 * Description:
 * write shutter to sensor registers
 * please pay attention to the frame length
 * please modify this function acording your spec
 *============================================================================*/
static void gc5024_write_shutter(uint32_t shutter)
{
	if(shutter < 1) shutter = 1;
	if(shutter > 16383) shutter = 16383;//2	 ^ 14

	if(shutter<=20)
	{
		Sensor_WriteReg(0xfe, 0x00);
		Sensor_WriteReg(0x32, 0x09);
		Sensor_WriteReg(0xb0, 0x53);
    }
	else
	{
		Sensor_WriteReg(0xfe, 0x00);
		Sensor_WriteReg(0x32, 0x49);
		Sensor_WriteReg(0xb0, 0x4b);
	}
		
	//Update Shutter	
	Sensor_WriteReg(0xfe, 0x00);
	Sensor_WriteReg(0x04, (shutter) & 0xFF);
	Sensor_WriteReg(0x03, (shutter >> 8) & 0x3F);	
}

/*==============================================================================
 * Description:
 * write exposure to sensor registers and get current shutter
 * please pay attention to the frame length
 * please don't change this function if it's necessary
 *============================================================================*/
static uint16_t gc5024_update_exposure(uint32_t shutter,uint32_t dummy_line)
{
	uint32_t dest_fr_len = 0;
	uint32_t cur_fr_len = 0;
	uint32_t fr_len = s_current_default_frame_length;
	uint32_t dummyline = 0;
	
	if (1 == SUPPORT_AUTO_FRAME_LENGTH)
		goto write_sensor_shutter;

	dummyline = dummy_line > FRAME_OFFSET? dummy_line:FRAME_OFFSET;
	dest_fr_len = ((shutter + dummyline) > fr_len) ? (shutter + dummyline) : fr_len;

	cur_fr_len = gc5024_read_frame_length();

	if (shutter < SENSOR_MIN_SHUTTER)
		shutter = SENSOR_MIN_SHUTTER;

	if (dest_fr_len != cur_fr_len)
		gc5024_write_frame_length(dest_fr_len);
write_sensor_shutter:
	/* write shutter to sensor registers */
	gc5024_write_shutter(shutter);
	#ifdef GAIN_DELAY_1_FRAME
	usleep(dest_fr_len*s_gc5024_resolution_trim_tab[cur_mode].line_time/10);
	#endif
	return shutter;
}

/*==============================================================================
 * Description:
 * sensor power on
 * please modify this function acording your spec
 *============================================================================*/
static uint32_t gc5024_power_on(uint32_t power_on)
{
	SENSOR_AVDD_VAL_E dvdd_val = g_gc5024_mipi_raw_info.dvdd_val;
	SENSOR_AVDD_VAL_E avdd_val = g_gc5024_mipi_raw_info.avdd_val;
	SENSOR_AVDD_VAL_E iovdd_val = g_gc5024_mipi_raw_info.iovdd_val;
	BOOLEAN power_down = g_gc5024_mipi_raw_info.power_down_level;
	BOOLEAN reset_level = g_gc5024_mipi_raw_info.reset_pulse_level;

	if (SENSOR_TRUE == power_on) {
		Sensor_PowerDown(power_down);
		Sensor_SetResetLevel(reset_level);
		usleep(1 * 500);     //1000
		Sensor_SetIovddVoltage(iovdd_val);
		usleep(1 * 500);   //1000
		Sensor_SetDvddVoltage(dvdd_val);
		usleep(1 * 500); // 1000
		Sensor_SetAvddVoltage(avdd_val);
		usleep(1 * 500);  //1000
		Sensor_SetMCLK(SENSOR_DEFALUT_MCLK);
		Sensor_PowerDown(!power_down);
		Sensor_SetResetLevel(!reset_level);

		#ifndef CONFIG_CAMERA_AUTOFOCUS_NOT_SUPPORT
		Sensor_SetMonitorVoltage(SENSOR_AVDD_2800MV);
		usleep(5 * 1000);
		#else
		Sensor_SetMonitorVoltage(SENSOR_AVDD_CLOSED);
		#endif

	} else {

		#ifndef CONFIG_CAMERA_AUTOFOCUS_NOT_SUPPORT
		Sensor_SetMonitorVoltage(SENSOR_AVDD_CLOSED);
		#endif

		Sensor_PowerDown(power_down);
		Sensor_SetResetLevel(reset_level);
		usleep(1 * 500);    //1000
		Sensor_SetMCLK(SENSOR_DISABLE_MCLK);
		Sensor_SetAvddVoltage(SENSOR_AVDD_CLOSED);
		Sensor_SetDvddVoltage(SENSOR_AVDD_CLOSED);
		Sensor_SetIovddVoltage(SENSOR_AVDD_CLOSED);
		//Sensor_PowerDown(!power_down);
		
	}
	SENSOR_PRINT("(1:on, 0:off): %d", power_on);
	return SENSOR_SUCCESS;
}

#ifdef FEATURE_OTP
/*==============================================================================
 * Description:
 * get  parameters from otp
 * please modify this function acording your spec
 *============================================================================*/
static int gc5024_get_otp_info(struct otp_info_t *otp_info)
{
	uint32_t ret = SENSOR_FAIL;
	uint32_t i = 0x00;

	//identify otp information
	for (i = 0; i < NUMBER_OF_ARRAY(s_gc5024_raw_param_tab); i++) {
		SENSOR_PRINT("identify module_id=0x%x",s_gc5024_raw_param_tab[i].param_id);

		if(PNULL!=s_gc5024_raw_param_tab[i].identify_otp){
			//set default value;
			memset(otp_info, 0x00, sizeof(struct otp_info_t));

			if(SENSOR_SUCCESS==s_gc5024_raw_param_tab[i].identify_otp(otp_info)){
				//if (s_gc5024_raw_param_tab[i].param_id== otp_info->module_id) {
					SENSOR_PRINT("identify otp sucess! module_id=0x%x",s_gc5024_raw_param_tab[i].param_id);
					ret = SENSOR_SUCCESS;
					break;
				//}
				//else{
				//	SENSOR_PRINT("identify module_id failed! table module_id=0x%x, otp module_id=0x%x",s_gc5024_raw_param_tab[i].param_id,otp_info->module_id);
				//}
			}
			else{
				SENSOR_PRINT("identify_otp failed!");
			}
		}
		else{
			SENSOR_PRINT("no identify_otp function!");
		}
	}

	if (SENSOR_SUCCESS == ret)
		return i;
	else
		return -1;
}

/*==============================================================================
 * Description:
 * apply otp parameters to sensor register
 * please modify this function acording your spec
 *============================================================================*/
static uint32_t gc5024_apply_otp(struct otp_info_t *otp_info, int id)
{
	uint32_t ret = SENSOR_FAIL;
	//apply otp parameters
	SENSOR_PRINT("otp_table_id = %d", id);
	if (PNULL != s_gc5024_raw_param_tab[id].cfg_otp) {

		if(SENSOR_SUCCESS==s_gc5024_raw_param_tab[id].cfg_otp(otp_info)){
			SENSOR_PRINT("apply otp parameters sucess! module_id=0x%x",s_gc5024_raw_param_tab[id].param_id);
			ret = SENSOR_SUCCESS;
		}
		else{
			SENSOR_PRINT("update_otp failed!");
		}
	}else{
		SENSOR_PRINT("no update_otp function!");
	}

	return ret;
}

/*==============================================================================
 * Description:
 * cfg otp setting
 * please modify this function acording your spec
 *============================================================================*/
static uint32_t gc5024_cfg_otp(uint32_t param)
{
	uint32_t ret = SENSOR_FAIL;
	struct otp_info_t otp_info={0x00};
	int table_id = 0;

	table_id = gc5024_get_otp_info(&otp_info);
	if (-1 != table_id)
		ret = gc5024_apply_otp(&otp_info, table_id);

	//checking OTP apply result
	if (SENSOR_SUCCESS != ret) {//disable lsc
		//Sensor_WriteReg(0xYYYY,0xYY);
	}
	else{//enable lsc
		//Sensor_WriteReg(0xYYYY,0xYY);
	}

	return ret;
}
#endif
/*==============================================================================
 * Description:
 * cfg otp setting
 * please modify this function acording your spec
 *============================================================================*/
static uint32_t gc5024_InitExifInfo(void)
{
	EXIF_SPEC_PIC_TAKING_COND_T *exif_ptr = &s_gc5024_exif_info;

	memset(&s_gc5024_exif_info, 0, sizeof(EXIF_SPEC_PIC_TAKING_COND_T));

	SENSOR_PRINT("Start");

	/*aperture = numerator/denominator */
	/*fnumber = numerator/denominator */
	exif_ptr->valid.FNumber=1;
	exif_ptr->FNumber.numerator=14;
	exif_ptr->FNumber.denominator=5;
	
	exif_ptr->valid.ApertureValue=1;
	exif_ptr->ApertureValue.numerator=14;
	exif_ptr->ApertureValue.denominator=5;
	exif_ptr->valid.MaxApertureValue=1;
	exif_ptr->MaxApertureValue.numerator=14;
	exif_ptr->MaxApertureValue.denominator=5;
	exif_ptr->valid.FocalLength=1;
	exif_ptr->FocalLength.numerator=289;
	exif_ptr->FocalLength.denominator=100;

	return SENSOR_SUCCESS;
}

static uint32_t  gc5024_get_exif_info(unsigned long param)
{
	return (unsigned long) & s_gc5024_exif_info;
}

static uint8_t gc5024_get_module_id(){
	uint8_t info_flag,module_id = 0;

	Sensor_WriteReg(0xfe, 0x00);
	Sensor_WriteReg(0xf7, 0x01);
	Sensor_WriteReg(0xf8, 0x11);
	Sensor_WriteReg(0xf9, 0xae);
	Sensor_WriteReg(0xfc, 0xae);
	Sensor_WriteReg(0xfa, 0x94);
	Sensor_WriteReg(0xd4, 0x80);	
	//Sensor_WriteReg(0xd5, 0x00);	
	//Sensor_WriteReg(0xf3, 0x20);		
	info_flag = Sensor_ReadReg(0xd7);

	if(0x01 == (info_flag>>4)&0x03)
	{
		Sensor_WriteReg(0xd5, 0x08);	
	}
	else
	{
		Sensor_WriteReg(0xd5, 0x48);	
	}
	
	Sensor_WriteReg(0xf3, 0x20);		
	module_id = Sensor_ReadReg(0xd7);	
      SENSOR_PRINT("gc5024_get_module_id module_id = %d",module_id);
	return module_id;
}



/*==============================================================================
 * Description:
 * identify sensor id
 * please modify this function acording your spec
 *============================================================================*/
static uint32_t gc5024_identify(uint32_t param)
{
	uint16_t pid_value = 0x00;
	uint16_t ver_value = 0x00;
	uint32_t ret_value = SENSOR_FAIL;
       static uint8_t module_id = 0;

	SENSOR_PRINT("mipi raw identify");

	pid_value = Sensor_ReadReg(GC5024_PID_ADDR);

     // module_id = gc5024_get_module_id();
		   module_id = 0x01;
	if (GC5024_PID_VALUE == pid_value && module_id == 0x01) {
		ver_value = Sensor_ReadReg(GC5024_VER_ADDR);
		SENSOR_PRINT("Identify: PID = %x, VER = %x", pid_value, ver_value);
		if (GC5024_VER_VALUE == ver_value) {
			SENSOR_PRINT_HIGH("this is gc5024 sensor");
			
			#if 0//def FEATURE_OTP
			/*if read otp info failed or module id mismatched ,identify failed ,return SENSOR_FAIL ,exit identify*/
			if(PNULL!=s_gc5024_raw_param_tab_ptr->identify_otp){
				SENSOR_PRINT("identify module_id=0x%x",s_gc5024_raw_param_tab_ptr->param_id);
				//set default value
				memset(s_gc5024_otp_info_ptr, 0x00, sizeof(struct otp_info_t));
				ret_value = s_gc5024_raw_param_tab_ptr->identify_otp(s_gc5024_otp_info_ptr);
				
				if(SENSOR_SUCCESS == ret_value ){
					SENSOR_PRINT("identify otp sucess! module_id=0x%x, module_name=%s",s_gc5024_raw_param_tab_ptr->param_id,MODULE_NAME);
				} else{
					SENSOR_PRINT("identify otp fail! exit identify");
					return ret_value;
				}
			} else{
			SENSOR_PRINT("no identify_otp function!");
			}

			#endif
			
			gc5024_InitExifInfo();
			#if defined(CONFIG_CAMERA_ISP_VERSION_V3) || defined(CONFIG_CAMERA_ISP_VERSION_V4)
			gc5024_InitRawTuneInfo();
			#endif
			ret_value = SENSOR_SUCCESS;
			SENSOR_PRINT_HIGH("this is gc5024 sensor");
		} else {
			SENSOR_PRINT_HIGH("Identify this is %x%x sensor", pid_value, ver_value);
		}
	} else {
		SENSOR_PRINT_HIGH("identify fail, pid_value = %x", pid_value);
	}

	return ret_value;
}

/*==============================================================================
 * Description:
 * get resolution trim
 *
 *============================================================================*/
static unsigned long gc5024_get_resolution_trim_tab(uint32_t param)
{
	return (unsigned long) s_gc5024_resolution_trim_tab;
}

/*==============================================================================
 * Description:
 * before snapshot
 * you can change this function if it's necessary
 *============================================================================*/
static uint32_t gc5024_before_snapshot(uint32_t param)
{
	uint32_t cap_shutter = 0;
	uint32_t prv_shutter = 0;
	uint32_t gain = 0;
	uint32_t cap_gain = 0;
	uint32_t capture_mode = param & 0xffff;
	uint32_t preview_mode = (param >> 0x10) & 0xffff;

	uint32_t prv_linetime = s_gc5024_resolution_trim_tab[preview_mode].line_time;
	uint32_t cap_linetime = s_gc5024_resolution_trim_tab[capture_mode].line_time;

	s_current_default_frame_length = gc5024_get_default_frame_length(capture_mode);
	SENSOR_PRINT("capture_mode = %d", capture_mode);

	if (preview_mode == capture_mode) {
		cap_shutter = s_sensor_ev_info.preview_shutter;
		cap_gain = s_sensor_ev_info.preview_gain;
		goto snapshot_info;
	}

	prv_shutter = s_sensor_ev_info.preview_shutter;	//gc5024_read_shutter();
	gain = s_sensor_ev_info.preview_gain;	//gc5024_read_gain();

	Sensor_SetMode(capture_mode);
	Sensor_SetMode_WaitDone();

	cap_shutter = prv_shutter * prv_linetime / cap_linetime * BINNING_FACTOR;

	while (gain >= (2 * SENSOR_BASE_GAIN)) {
		if (cap_shutter * 2 > s_current_default_frame_length)
			break;
		cap_shutter = cap_shutter * 2;
		gain = gain / 2;
	}

	cap_shutter = gc5024_update_exposure(cap_shutter,0);
	cap_gain = gain;
	gc5024_write_gain(cap_gain);
	SENSOR_PRINT("preview_shutter = 0x%x, preview_gain = 0x%x",
		     s_sensor_ev_info.preview_shutter, s_sensor_ev_info.preview_gain);

	SENSOR_PRINT("capture_shutter = 0x%x, capture_gain = 0x%x", cap_shutter, cap_gain);
snapshot_info:
	s_hdr_info.capture_shutter = cap_shutter; //gc5024_read_shutter();
	s_hdr_info.capture_gain = cap_gain; //gc5024_read_gain();
	/* limit HDR capture min fps to 10;
	 * MaxFrameTime = 1000000*0.1us;
	 */
	//s_hdr_info.capture_max_shutter = 1000000/ cap_linetime;

	Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_EXPOSURETIME, cap_shutter);

	return SENSOR_SUCCESS;
}

/*==============================================================================
 * Description:
 * get the shutter from isp
 * please don't change this function unless it's necessary
 *============================================================================*/
static uint32_t gc5024_write_exposure(uint32_t param)
{
	uint32_t ret_value = SENSOR_SUCCESS;
	uint16_t exposure_line = 0x00;
	uint16_t dummy_line = 0x00;
	uint16_t mode = 0x00;

	exposure_line = param & 0xffff;
	dummy_line = (param >> 0x10) & 0xfff; /*for cits frame rate test*/
	mode = (param >> 0x1c) & 0x0f;
	#ifdef GAIN_DELAY_1_FRAME
	cur_mode = mode;
	#endif

	SENSOR_PRINT("current mode = %d, exposure_line = %d, dummy_line=%d", mode, exposure_line,dummy_line);
	s_current_default_frame_length = gc5024_get_default_frame_length(mode);

	s_sensor_ev_info.preview_shutter = gc5024_update_exposure(exposure_line,dummy_line);

	return ret_value;
}

/*==============================================================================
 * Description:
 * get the parameter from isp to real gain
 * you mustn't change the funcion !
 *============================================================================*/
static uint32_t isp_to_real_gain(uint32_t param)
{
	uint32_t real_gain = 0;

	
#if defined(CONFIG_CAMERA_ISP_VERSION_V3) || defined(CONFIG_CAMERA_ISP_VERSION_V4)
	real_gain=param;
#else
	real_gain = ((param & 0xf) + 16) * (((param >> 4) & 0x01) + 1);
	real_gain = real_gain * (((param >> 5) & 0x01) + 1) * (((param >> 6) & 0x01) + 1);
	real_gain = real_gain * (((param >> 7) & 0x01) + 1) * (((param >> 8) & 0x01) + 1);
	real_gain = real_gain * (((param >> 9) & 0x01) + 1) * (((param >> 10) & 0x01) + 1);
	real_gain = real_gain * (((param >> 11) & 0x01) + 1);
#endif

	return real_gain;
}

/*==============================================================================
 * Description:
 * write gain value to sensor
 * you can change this function if it's necessary
 *============================================================================*/
static uint32_t gc5024_write_gain_value(uint32_t param)
{
	uint32_t ret_value = SENSOR_SUCCESS;
	uint32_t real_gain = 0;

	real_gain = isp_to_real_gain(param);

	real_gain = real_gain * SENSOR_BASE_GAIN / ISP_BASE_GAIN;

	SENSOR_PRINT("real_gain = 0x%x", real_gain);

	s_sensor_ev_info.preview_gain = real_gain;
	gc5024_write_gain(real_gain);

	return ret_value;
}

#ifndef CONFIG_CAMERA_AUTOFOCUS_NOT_SUPPORT
/*==============================================================================
 * Description:
 * get otp of vcm
 * please add your VCM function to this function
 *============================================================================*/
static uint32_t gc5024_get_motor_pos(uint16_t *pos)
{
	
	*pos = cur_pos;	
	 return dw9714_get_motor_pos(pos);// af driver new version  use
	/*
	uint32_t ret_value = 0;
	*pos = cur_pos;	
	return ret_value;
	*/
}

/*==============================================================================
 * Description:
 * vcm init
 * please add your VCM function to this function
 *============================================================================*/
static uint32_t gc5024_set_motor_bestmode()
{
	return dw9714_set_motor_bestmode(2); // af driver new version  use
    //return dw9714_init(2);
	
}

/*==============================================================================
 * Description:
 * get otp of vcm
 * please inform with your module vendor
 *============================================================================*/
static uint32_t gc5024_get_otp(uint16_t *inf,uint16_t *macro)
{
	#ifdef FEATURE_OTP
	// no need this af otp
	//*inf = s_gc5024_otp_info_ptr->vcm_dac_inifity;
	//*macro = s_gc5024_otp_info_ptr->vcm_dac_macro;
	#endif
	
	return 0;
}

/*==============================================================================
 * Description:
 * write parameter to vcm
 * please add your VCM function to this function
 *============================================================================*/
static uint32_t gc5024_write_af(uint32_t param)
{
	SENSOR_PRINT("dw9714_set_pos param = %d", param);
	
	cur_pos = param;
	return dw9714_set_pos(param);// af driver new version  use
	/*
	cur_pos = param;
	return dw9714_write_af(param);
	*/
	
}
#endif

/*==============================================================================
 * Description:
 * increase gain or shutter for hdr
 *
 *============================================================================*/
static void gc5024_increase_hdr_exposure(uint8_t ev_multiplier)
{
	uint32_t shutter_multiply = s_hdr_info.capture_max_shutter / s_hdr_info.capture_shutter;
	uint32_t gain = 0;

	if (0 == shutter_multiply)
		shutter_multiply = 1;

	if (shutter_multiply >= ev_multiplier) {
		gc5024_update_exposure(s_hdr_info.capture_shutter * ev_multiplier,0);
		gc5024_write_gain(s_hdr_info.capture_gain);
	} else {
		gain = s_hdr_info.capture_gain * ev_multiplier / shutter_multiply;

		gc5024_update_exposure(s_hdr_info.capture_shutter * shutter_multiply,0);
		gc5024_write_gain(gain);
	}
}

/*==============================================================================
 * Description:
 * decrease gain or shutter for hdr
 *
 *============================================================================*/
static void gc5024_decrease_hdr_exposure(uint8_t ev_divisor)
{
	uint16_t gain_multiply = 0;
	uint32_t shutter = 0;
	gain_multiply = s_hdr_info.capture_gain / SENSOR_BASE_GAIN;

	if (gain_multiply >= ev_divisor) {
		gc5024_update_exposure(s_hdr_info.capture_shutter,0);
		gc5024_write_gain(s_hdr_info.capture_gain / ev_divisor);

	} else {
		shutter = s_hdr_info.capture_shutter * gain_multiply / ev_divisor;
		gc5024_update_exposure(shutter,0);
		gc5024_write_gain(s_hdr_info.capture_gain / gain_multiply);
	}
}

/*==============================================================================
 * Description:
 * set hdr ev
 * you can change this function if it's necessary
 *============================================================================*/
static uint32_t gc5024_set_hdr_ev(unsigned long param)
{
	uint32_t ret = SENSOR_SUCCESS;
	SENSOR_EXT_FUN_PARAM_T_PTR ext_ptr = (SENSOR_EXT_FUN_PARAM_T_PTR) param;

	uint32_t ev = ext_ptr->param;
	uint8_t ev_divisor, ev_multiplier;

	switch (ev) {
	case SENSOR_HDR_EV_LEVE_0:
		ev_divisor = 2;
		gc5024_decrease_hdr_exposure(ev_divisor);
		break;
	case SENSOR_HDR_EV_LEVE_1:
		ev_multiplier = 1;
		gc5024_increase_hdr_exposure(ev_multiplier);
		break;
	case SENSOR_HDR_EV_LEVE_2:
		ev_multiplier = 2;
		gc5024_increase_hdr_exposure(ev_multiplier);
		break;
	default:
		break;
	}
	return ret;
}

/*==============================================================================
 * Description:
 * extra functoin
 * you can add functions reference SENSOR_EXT_FUNC_CMD_E which from sensor_drv_u.h
 *============================================================================*/
static uint32_t gc5024_ext_func(unsigned long param)
{
	uint32_t rtn = SENSOR_SUCCESS;
	SENSOR_EXT_FUN_PARAM_T_PTR ext_ptr = (SENSOR_EXT_FUN_PARAM_T_PTR) param;

	SENSOR_PRINT("ext_ptr->cmd: %d", ext_ptr->cmd);
	switch (ext_ptr->cmd) {
	case SENSOR_EXT_EV:
		rtn = gc5024_set_hdr_ev(param);
		break;
	default:
		break;
	}

	return rtn;
}

/*==============================================================================
 * Description:
 * mipi stream on
 * please modify this function acording your spec
 *============================================================================*/
static uint32_t gc5024_stream_on(uint32_t param)
{
	SENSOR_PRINT("E");
	usleep(100*1000);
	Sensor_WriteReg(0xfe, 0x03);
	Sensor_WriteReg(0x10, 0x91);
	Sensor_WriteReg(0xfe, 0x00);
	/*delay*/
	usleep(20*1000);

	return 0;
}

/*==============================================================================
 * Description:
 * mipi stream off
 * please modify this function acording your spec
 *============================================================================*/
static uint32_t gc5024_stream_off(uint32_t param)
{
	SENSOR_PRINT("E");
	usleep(20 * 1000);
	Sensor_WriteReg(0xfe, 0x03);
	Sensor_WriteReg(0x10, 0x00);
	Sensor_WriteReg(0xfe, 0x00);
	/*delay*/
	usleep(100*1000);

	return 0;
}


/*==============================================================================
 * Description:
 * all ioctl functoins
 * you can add functions reference SENSOR_IOCTL_FUNC_TAB_T from sensor_drv_u.h
 *
 * add ioctl functions like this:
 * .power = gc5024_power_on,
 *============================================================================*/
static SENSOR_IOCTL_FUNC_TAB_T s_gc5024_ioctl_func_tab = {
	.power = gc5024_power_on,
	.identify = gc5024_identify,
	.get_trim = gc5024_get_resolution_trim_tab,
	.before_snapshort = gc5024_before_snapshot,
	.write_ae_value = gc5024_write_exposure,
	.write_gain_value = gc5024_write_gain_value,
	#ifndef CONFIG_CAMERA_AUTOFOCUS_NOT_SUPPORT
	.af_enable = gc5024_write_af,
	.set_pos = gc5024_write_af,
	//.get_otp = gc5024_get_otp,
	.get_motor_pos = gc5024_get_motor_pos,
	.set_motor_bestmode = gc5024_set_motor_bestmode,
	#endif
	.get_exif = gc5024_get_exif_info,
	.set_focus = gc5024_ext_func,
	//.set_video_mode = gc5024_set_video_mode,
	.stream_on = gc5024_stream_on,
	.stream_off = gc5024_stream_off,
	#ifdef FEATURE_OTP
	.cfg_otp=gc5024_cfg_otp,
	#endif	
};
