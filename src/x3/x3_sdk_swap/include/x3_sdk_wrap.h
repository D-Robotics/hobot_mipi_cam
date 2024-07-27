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

#ifndef X3_SDK_WRAP_H_
#define X3_SDK_WRAP_H_

// SDK 提供的接口都经过这里封装好向上提供

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include "vio/hb_vin_api.h"
#include "vio/hb_mipi_api.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_sys.h"
#include "vio/hb_comm_venc.h"
#include "vio/hb_comm_vdec.h"
#include "vio/hb_common_vot.h"
#include "vio/hb_vps_api.h"
#ifdef __cplusplus
};
#endif /* __cplusplus */

typedef struct {
	/* gdc 配置文件 */
	char m_gdc_config[128];
	ROTATION_E m_rotation;
} x3_gdc_info_t;

// 以下两个结构体用来描述模块使用的内存buff信息
typedef struct vp_param {
    uint64_t mmz_paddr[32];
    char *mmz_vaddr[32];
    int mmz_cnt;
    int mmz_size;
} vp_param_t;

typedef struct av_param {
    int count;
    int videoIndex;
    int bufSize;
    int firstPacket;
} av_param_t;


// 一个vps 通道的属性配置
typedef struct{
	int m_chn_id; /* vps 输出通道 */
	int m_chn_enable; /* 是否使能通道 */
	/* 通道输出尺寸限制请查阅《X3J3平台AIOT媒体系统接口手册.pdf》5.3 功能描述 */
	VPS_CHN_ATTR_S m_chn_attr; /* 通道配置 */
} x3_vps_chn_attr_t;

// 一个ipu group的配置，包含一个输入配置和7个输出通道
typedef struct {
	/* VPS 输入配置*/
	int m_vps_grp_id;
	VPS_GRP_ATTR_S 	m_vps_grp_attr;
	/* 以下是vps输出通道配置，最多支持7个通道，第7个通道需要从通道2 online给到pym */
	int m_chn_num; // 使能几个通道
	x3_vps_chn_attr_t m_vps_chn_attrs[7];
} x3_vps_info_t;

// 多个VPS group的配置
typedef struct {
	/* VPS group 数量*/
	int m_group_num;
	x3_vps_info_t m_vps_info[8];
} x3_vps_infos_t;

typedef struct x3_vin_info {
     /*定义 sensor   初始化的属性信息 */
    MIPI_SENSOR_INFO_S snsinfo;
     /*定义 mipi 初始化参数信息 */
    MIPI_ATTR_S mipi_attr;
     /*定义 dev 初始化的属性信息 */
    VIN_DEV_ATTR_S devinfo;
     /*定义 pipe 属性信息 */
    VIN_PIPE_ATTR_S pipeinfo;
     /*定义 DIS(gdc) 属性信息 */
    VIN_DIS_ATTR_S disinfo;
     /*定义 LDC 属性信息 */
    VIN_LDC_ATTR_S ldcinfo;

    int dev_id;       /* 通路索引，范围 0~7 */
    int pipe_id;      /* PipeLine 号， 对应每路输入，范围 0~7 */

    /* 配置vin到vps是online还是offline模式 */
    SYS_VIN_VPS_MODE_E vin_vps_mode;

	/* dev扩展属性 */
	int enable_dev_attr_ex;
    VIN_DEV_ATTR_EX_S devexinfo;
} x3_vin_info_t;


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void print_debug_infos(void);

int x3_vps_init_wrap(x3_vps_info_t *vps_info);
void x3_vps_uninit_wrap(x3_vps_info_t *vps_info);

#ifdef __cplusplus
};
#endif /* __cplusplus */
#endif // X3_SDK_WRAP_H_

