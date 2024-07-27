// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CAMERA_STRUCT_DEFINE_H
#define CAMERA_STRUCT_DEFINE_H

#include <time.h>

typedef enum
{
	CAM_PARAM_NULL,
	CAM_GET_CHIP_TYPE,
	CAM_PARAM_STATE_GET,
	CAM_PARAM_ALL_SET,
	CAM_PARAM_ALL_GET,
	CAM_DEC_DATA_PUSH,					// 发送音频解码数据
	CAM_DEC_DATA_CLEAR,					// 清空音频解码数据
	CAM_DEC_FILE_PUSH,					// 发送音频解码文件
	CAM_MOTION_CALLBACK_REG,			// 注册移动侦测回调
	CAM_MOTION_CALLBACK_UNREG,			// 注销移动侦测回调
	CAM_VENC_FRAMERATE_SET,
	CAM_VENC_BITRATE_SET,
	CAM_VENC_GOP_SET,
	CAM_VENC_CVBR_SET,
	CAM_VENC_FORCEIDR,
	CAM_VENC_MIRROR_SET,
	CAM_VENC_NTSC_PAL_SET,
	CAM_VENC_NTSC_PAL_GET,
	CAM_AENC_PARAM_GET,
	CAM_AENC_VOLUME_SET,
	CAM_AENC_MUTE_SET,
	CAM_AENC_SAMPLERATE_SET,
	CAM_AENC_BITRATE_SET,
	CAM_AENC_CHANNLES_SET,
	CAM_AENC_AEC_SET,
	CAM_AENC_AGC_SET,
	CAM_AENC_ANR_SET,
	CAM_ADEC_VOLUME_SET,
	CAM_ADEC_SAMPLERATE_SET,
	CAM_ADEC_BITRATE_SET,
	CAM_ADEC_CHANNLES_SET,
	CAM_OSD_TIMEENABLE_SET,
	CAM_SNAP_PATH_SET,
	CAM_SNAP_ONCE,
	CAM_MOTION_PARAM_GET,
	CAM_MOTION_PARAM_SET,
	CAM_YUV_BUFFER_START,
	CAM_YUV_BUFFER_STOP,
	CAM_MOTION_SENSITIVITY_SET,
	CAM_MOTION_AREA_SET,
	CAM_GPIO_CTRL_SET,
	CAM_GPIO_CTRL_GET,
	CAM_ADC_CTRL_GET,
	CAM_ISP_CTRL_MODE_SET,
	CAM_RINGBUFFER_CREATE,
	CAM_RINGBUFFER_DATA_GET,
	CAM_RINGBUFFER_DATA_SYNC,
	CAM_RINGBUFFER_DESTORY,
	CAM_VENC_ALL_PARAM_GET, // 获取编码所有通道的参数
	CAM_VENC_CHN_PARAM_GET, // 获取单个编码通道的参数
	CAM_GET_VENC_CHN_STATUS,
	CAM_GET_RAW_FRAME,
	CAM_GET_YUV_FRAME,
	CAM_JPEG_SNAP,
	CAM_START_RECORDER,
	CAM_STOP_RECORDER,
}CAM_PARAM_E;

#define CAM_ACODEC_FILE		"/dev/acodec"
#define CAM_CONF_PATH		"/mnt/config/"
#define CAM_CONF_FILE		CAM_CONF_PATH"camera.json"
#define CAM_CONF_DEFAULT	CAM_CONF_PATH"camera.json.default"

#define VENC_MAX_CHANNLES	(2)
typedef int (*cam_motion_effect_call)(int type, time_t time, int duration);
typedef enum
{
	CAM_AENC_TYPE_G711A		=	19,
	CAM_AENC_TYPE_AAC		=	37,
	CAM_VENC_TYPE_H264		= 	96,
	CAM_VENC_TYPE_H265		=	265,
	CAM_VENC_TYPE_MJPEG		=	1002,
}CAM_PAYLOAD_TYPE_E;

typedef enum
{
	CAM_VENC_RC_CBR			=	0,
	CAM_VENC_RC_VBR			=	1,
	CAM_VENC_RC_BUTT,
}CAM_RC_TYPE_E;

typedef struct{
	int			enable;
	int			channle;	// 通道号
	int			type;		// 编码类型 H264 H265
	int			mirror;		// 旋转
	int			flip;		// 镜像
	
	float		framerate;	// 帧率
	int			bitrate;	// 码率或最大码率
	int			stream_buf_size; // 编码器设置的buf size
	int			profile;	// H264的base main high; H265 high
	int			gop;		// I帧间隔
	int			width;		// 分辨率宽
	int 		height;		// 分辨率高
	int			cvbr;		// 码率控制方式 定码率 变码率
	int			minqp;
	int			maxqp;
}venc_info_t;

typedef struct{
	int			channles;
	int			framerate;	// sensor vi 帧率
	int			ntsc_pal;	// 0:NTSC 1:PAL
	venc_info_t	infos[VENC_MAX_CHANNLES];
}venc_infos_t;

typedef enum  
{ 
    CAM_AUDIO_SAMPLE_RATE_8000   = 8000,    /* 8K samplerate*/
    CAM_AUDIO_SAMPLE_RATE_12000  = 12000,   /* 12K samplerate*/    
    CAM_AUDIO_SAMPLE_RATE_11025  = 11025,   /* 11.025K samplerate*/
    CAM_AUDIO_SAMPLE_RATE_16000  = 16000,   /* 16K samplerate*/
    CAM_AUDIO_SAMPLE_RATE_22050  = 22050,   /* 22.050K samplerate*/
    CAM_AUDIO_SAMPLE_RATE_24000  = 24000,   /* 24K samplerate*/
    CAM_AUDIO_SAMPLE_RATE_32000  = 32000,   /* 32K samplerate*/
    CAM_AUDIO_SAMPLE_RATE_44100  = 44100,   /* 44.1K samplerate*/
    CAM_AUDIO_SAMPLE_RATE_48000  = 48000,   /* 48K samplerate*/
    CAM_AUDIO_SAMPLE_RATE_64000  = 64000,   /* 64K samplerate*/
    CAM_AUDIO_SAMPLE_RATE_96000  = 96000,   /* 96K samplerate*/
    CAM_AUDIO_SAMPLE_RATE_BUTT,
} CAM_AUDIO_SAMPLE_RATE_E; 

typedef enum 
{
    CAM_AUDIO_SOUND_MODE_MONO   =0,/*mono*/
    CAM_AUDIO_SOUND_MODE_STEREO =1,/*stereo*/
    CAM_AUDIO_SOUND_MODE_BUTT    
} CAM_AUDIO_SOUND_MODE_E;

typedef struct{
	int			enable;		// 开关
	int			channle;	// 通道号
	int			type;		// 编码类型
	int			mic_input;	// mic输入类型0: mic 1:line	
	int			mic_mute;	// 输入静音
	
	int			channles;	// 声道数
	int			bitrate;	// 码率
	int			samplerate;	// 采样率
	int			volume;		// 输入音量

	int			aec;		// 回声抵消算法
	int			agc;		// 自动增益算法
	int			anr;		// 语音降噪
}aenc_info_t;

typedef struct{
	int			enable;
	int			channle;
	int			type;		// 编码类型 同aenc
	
	int			channles;	// 声道数
	int			bitrate;	// 码率
	int			samplerate;	// 采样率
	int			volume;		// 输出音量
}adec_info_t;

typedef struct
{
	int				enable;
	int				layer;			/* OVERLAY region layer range:[0,7]*/

	int				width;			//区域宽
	int				height;			//区域高
	int				point_x;		//point坐标 /* X:[0,4096],align:4,Y:[0,4096],align:4 */
	int				point_y;
	int				alpha_fg;		//前景透明度	range:[0,128]
	int				alpha_bg;		//背景透明度	range:[0,128]

	unsigned int	color_bg;		//背景颜色		PIXEL_FORMAT_RGB_1555
	unsigned short	color_fg;		//前景颜色		PIXEL_FORMAT_RGB_1555
	unsigned short	color_bd;		//边界颜色
	unsigned short	color_dg;		//阴影颜色
	int				font;			//字体大小		//支持 32 24 8
	int				space;			//空格占长		建议14
	//反色处理???
}osd_region_t;

typedef struct{
	int				enable;
	int				vchn;
	
	osd_region_t	time[VENC_MAX_CHANNLES];
//  notic
//  logo
}osd_info_t;

typedef struct{					//约定 venc的第一路视频参数做抓拍 绑定到venc chn0的vpss chn
	int		enable;		
	int		channle;			// 通道号		// venc channle
	int		width;				// 分辨率宽
	int 	height;				// 分辨率高
	char	path[64];			// 存储位置
}snap_info_t;

#define MAX_MD_RECT		16		// 将整体区域分为16个均等的小块，每个小块可以设置是否使能
typedef struct{
	int		x;
	int		y;
	int		w;
	int		h;
}md_rect;

typedef struct{
	int 	enable;
	int 	channle;				// 通道号		
	int 	vbblkcnt;				// 海思VB BLOCK COUNT
	int		width;					// 分辨率宽
	int 	height;					// 分辨率高
	float	framerate;
	int		sensitivity;
	int		rect_enable;			// 16个位分别对应16个小块是否使能
	int		sustain;				// 一个移动侦测持续时长
	md_rect	rect[MAX_MD_RECT];
	cam_motion_effect_call	func;	// 发送移动时状态回调
}motion_info_t;

typedef struct
{
	int 		enable;
	int			frequency;				// 频率，默认0
	int			mode;					// 0:普通 1:自动
}ae_antiflicker_t;					

typedef struct
{
	ae_antiflicker_t antiflicker;		// 抗闪烁
}ae_info_t;

typedef struct
{
	int		enable;						
	int		op_type;					// 0:自动曝光 1:手动曝光
	int		interval;					// AE 算法运行的间隔，取值范围为[1,255], 建议1
	
	ae_info_t		st_auto;
//	me_info_t		st_manual;
}isp_exposure_info_t;

typedef struct
{
	int		enable;
	char	ini[64];

	isp_exposure_info_t exposure;
}isp_info_t;

typedef struct
{
	isp_info_t		isp_info;
	venc_infos_t	venc_info;
	aenc_info_t		aenc_info;
	adec_info_t		adec_info;
	osd_info_t		osd_info;
	snap_info_t		snap_info;
	motion_info_t	motion_info;
}camera_info_t;

////////////////////////////////////////////////////////////////////
typedef struct
{
	int 	channle;
	int		count;
}cam_forceidr_t;

typedef struct
{
	int		ch;
	int		max_frame;
	int		max_size;
	char	name[64];
}cam_yuv_buffer_t;

typedef struct
{
	int			 	type;  //GPIO_CTRL_NAME
	int				val;
}cam_gpoi_ctrl_t;

typedef struct
{
	int			 	ch;  
	int				val;
}cam_adc_ctrl_t;

////////////////////////////////////////////////////////////////////

#endif
