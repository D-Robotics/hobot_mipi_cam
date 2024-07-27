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

#include "x3_preparam.h"
#include "sensor_f37_config.h"
#include "sensor_imx415_config.h"
#include "sensor_imx586_config.h"
#include "sensor_gc4c33_config.h"
#include "sensor_gc4663_config.h"
#include "sensor_imx219_config.h"
#include "sensor_imx477_config.h"
#include "sensor_ov5647_config.h"
#include "x3_sdk_wrap.h"

/******************************* F37 方案 **********************************/
int f37_linear_vin_param_init(x3_vin_info_t* vin_info) {
  vin_info->snsinfo = SENSOR_1LANE_F37_30FPS_10BIT_LINEAR_INFO;
  vin_info->mipi_attr = MIPI_1LANE_SENSOR_F37_30FPS_10BIT_LINEAR_ATTR;
  vin_info->devinfo = DEV_ATTR_F37_LINEAR_BASE;
  vin_info->pipeinfo = PIPE_ATTR_F37_LINEAR_BASE;
  vin_info->disinfo = DIS_ATTR_F37_LINEAR_BASE;
  vin_info->ldcinfo = LDC_ATTR_F37_LINEAR_BASE;
  vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;

  // 单目的使用dev_id 和 pipe_id 都设置成0
  vin_info->dev_id = 0;
  vin_info->pipe_id = 0;
  vin_info->enable_dev_attr_ex = 0;

  return 0;
}

int f37_dol2_vin_param_init(x3_vin_info_t* vin_info) {
  vin_info->snsinfo = SENSOR_1LANE_F37_30FPS_10BIT_DOL2_INFO;
  vin_info->mipi_attr = MIPI_1LANE_SENSOR_F37_30FPS_10BIT_DOL2_ATTR;
  vin_info->devinfo = DEV_ATTR_F37_DOL2_BASE;
  vin_info->pipeinfo = PIPE_ATTR_F37_DOL2_BASE;
  vin_info->disinfo = DIS_ATTR_F37_DOL2_BASE;
  vin_info->ldcinfo = LDC_ATTR_F37_DOL2_BASE;
  vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;

  // 单目的使用dev_id 和 pipe_id 都设置成0
  vin_info->dev_id = 0;
  vin_info->pipe_id = 0;
  vin_info->enable_dev_attr_ex = 0;
  return 0;
}

/******************************** IMX415 4K 方案 ******************************/
int imx415_linear_vin_param_init(x3_vin_info_t* vin_info) {
  vin_info->snsinfo = SENSOR_4LANE_IMX415_30FPS_10BIT_LINEAR_INFO;
  vin_info->mipi_attr = MIPI_4LANE_SENSOR_IMX415_30FPS_10BIT_LINEAR_ATTR;
  vin_info->devinfo = DEV_ATTR_IMX415_LINEAR_BASE;
  vin_info->pipeinfo = PIPE_ATTR_IMX415_LINEAR_BASE;
  vin_info->disinfo = DIS_ATTR_IMX415_BASE;
  vin_info->ldcinfo = LDC_ATTR_IMX415_BASE;
  vin_info->vin_vps_mode = VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE;

  // 单目的使用dev_id 和 pipe_id 都设置成0
  vin_info->dev_id = 0;
  vin_info->pipe_id = 0;
  vin_info->enable_dev_attr_ex = 0;
  return 0;
}
/******************************* IMX586 方案 **********************************/
int imx586_linear_vin_param_init(x3_vin_info_t* vin_info) {
  vin_info->snsinfo = SENSOR_IMX586_25FPS_10BIT_LINEAR_INFO;
  vin_info->mipi_attr = MIPI_SENSOR_IMX586_25FPS_10BIT_LINEAR_ATTR;
  vin_info->devinfo = DEV_ATTR_IMX586_LINEAR_BASE;
  vin_info->pipeinfo = PIPE_ATTR_IMX586_LINEAR_BASE;
  vin_info->disinfo = DIS_ATTR_IMX586_BASE;
  vin_info->ldcinfo = LDC_ATTR_IMX586_BASE;
  vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;  // VIN_OFFLINE_VPS_OFFINE;
  // 单目的使用dev_id 和 pipe_id 都设置成0
  vin_info->dev_id = 0;
  vin_info->pipe_id = 0;
  vin_info->enable_dev_attr_ex = 0;
  return 0;
}


/******************************* GC4C33 方案 **********************************/
int gc4c33_linear_vin_param_init(x3_vin_info_t* vin_info) {
  vin_info->snsinfo = SENSOR_GC4C33_30FPS_10BIT_LINEAR_INFO;
  vin_info->mipi_attr = MIPI_SENSOR_GC4C33_30FPS_10BIT_LINEAR_ATTR;
  vin_info->devinfo = DEV_ATTR_GC4C33_LINEAR_BASE;
  vin_info->pipeinfo = PIPE_ATTR_GC4C33_LINEAR_BASE;
  vin_info->disinfo = DIS_ATTR_GC4C33_BASE;
  vin_info->ldcinfo = LDC_ATTR_GC4C33_BASE;
  vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;  // VIN_OFFLINE_VPS_OFFINE;
  // 单目的使用dev_id 和 pipe_id 都设置成0
  vin_info->dev_id = 0;
  vin_info->pipe_id = 0;
  vin_info->enable_dev_attr_ex = 0;
  return 0;
}

/******************************* GC4663 方案 **********************************/
int gc4663_linear_vin_param_init(x3_vin_info_t* vin_info) {
  vin_info->snsinfo = SENSOR_GC4663_30FPS_1440P_LINEAR_INFO;
  vin_info->mipi_attr = MIPI_SENSOR_GC4663_30FPS_1440P_LINEAR_ATTR;
  vin_info->devinfo = DEV_ATTR_GC4663_LINEAR_BASE;
  vin_info->pipeinfo = PIPE_ATTR_GC4663_LINEAR_BASE;
  vin_info->disinfo = DIS_ATTR_GC4663_BASE;
  vin_info->ldcinfo = LDC_ATTR_GC4663_BASE;
  vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;  // VIN_OFFLINE_VPS_OFFINE;
  // 单目的使用dev_id 和 pipe_id 都设置成0
  vin_info->dev_id = 0;
  vin_info->pipe_id = 0;
  vin_info->enable_dev_attr_ex = 0;
  return 0;
}

/******************************* IMX219 方案 **********************************/
int imx219_linear_vin_param_init(x3_vin_info_t* vin_info) {
  vin_info->snsinfo = SENSOR_2LANE_IMX219_30FPS_10BIT_LINEAR_INFO;
  vin_info->mipi_attr = MIPI_2LANE_SENSOR_IMX219_30FPS_10BIT_LINEAR_ATTR;
  vin_info->devinfo = DEV_ATTR_IMX219_LINEAR_BASE;
  vin_info->pipeinfo = PIPE_ATTR_IMX219_LINEAR_BASE;
  vin_info->disinfo = DIS_ATTR_IMX219_LINEAR_BASE;
  vin_info->ldcinfo = LDC_ATTR_IMX219_LINEAR_BASE;
  vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;  // VIN_OFFLINE_VPS_OFFINE;

  // 单目的使用dev_id 和 pipe_id 都设置成0
  vin_info->dev_id = 0;
  vin_info->pipe_id = 0;
  vin_info->enable_dev_attr_ex = 0;

  return 0;
}

/******************************* IMX477 方案 **********************************/
int imx477_linear_vin_param_init(x3_vin_info_t* vin_info) {
  vin_info->snsinfo = SENSOR_2LANE_IMX477_50FPS_12BIT_LINEAR_INFO;
  vin_info->mipi_attr = MIPI_2LANE_SENSOR_IMX477_50FPS_12BIT_LINEAR_ATTR;
  vin_info->devinfo = DEV_ATTR_IMX477_LINEAR_BASE;
  vin_info->pipeinfo = PIPE_ATTR_IMX477_LINEAR_BASE;
  vin_info->disinfo = DIS_ATTR_IMX477_LINEAR_BASE;
  vin_info->ldcinfo = LDC_ATTR_IMX477_LINEAR_BASE;
  vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;  // VIN_OFFLINE_VPS_OFFINE;

  // 单目的使用dev_id 和 pipe_id 都设置成0
  vin_info->dev_id = 0;
  vin_info->pipe_id = 0;
  vin_info->enable_dev_attr_ex = 0;

  return 0;
}

/******************************* OV5647 方案 **********************************/
int ov5647_linear_vin_param_init(x3_vin_info_t* vin_info) {
  vin_info->snsinfo = SENSOR_2LANE_OV5647_30FPS_10BIT_LINEAR_INFO;
  vin_info->mipi_attr = MIPI_2LANE_SENSOR_OV5647_30FPS_10BIT_LINEAR_ATTR;
  vin_info->devinfo = DEV_ATTR_OV5647_LINEAR_BASE;
  vin_info->pipeinfo = PIPE_ATTR_OV5647_LINEAR_BASE;
  vin_info->disinfo = DIS_ATTR_OV5647_LINEAR_BASE;
  vin_info->ldcinfo = LDC_ATTR_OV5647_LINEAR_BASE;
  vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;  // VIN_OFFLINE_VPS_OFFINE;

  // 单目的使用dev_id 和 pipe_id 都设置成0
  vin_info->dev_id = 0;
  vin_info->pipe_id = 0;
  vin_info->enable_dev_attr_ex = 0;

  return 0;
}

// vps的输入参数
int vps_grp_param_init(x3_vps_info_t *vps_info, int width, int height) {
  // 默认group id 0
  vps_info->m_vps_grp_id = 0;
  vps_info->m_vps_grp_attr.maxW = width;
  vps_info->m_vps_grp_attr.maxH = height;
  /* vps grp的framedepth是gdc的buf深度，如果只用groupRotate，配成1就行 */
  vps_info->m_vps_grp_attr.frameDepth = 8;  // 原先是 1
  /*.pixelFormat 像素格式只支持nv12 */
  /*vps_info->m_vps_grp_attr.pixelFormat = HB_PIXEL_FORMAT_NV12;*/
  return 0;
}

// vps的通道参数，为什么要grp和chn分开两个调用，目的是为了更加灵活的配置一个group的输出
int vps_chn_param_init(x3_vps_chn_attr_t *vps_chn_attr,
                       int chn_id, int width, int height, int fps) {
  vps_chn_attr->m_chn_id = chn_id;  // vps 通道，只有chn2 和 chn5 支持4K输出
  vps_chn_attr->m_chn_enable = 1;
  vps_chn_attr->m_chn_attr.width = width;
  vps_chn_attr->m_chn_attr.height = height;
  vps_chn_attr->m_chn_attr.enMirror = 0;
  vps_chn_attr->m_chn_attr.enFlip = 0;
  vps_chn_attr->m_chn_attr.enScale = 1;
  vps_chn_attr->m_chn_attr.frameDepth = 8;  // 1;/* 图像队列长度, 最大32 */
  vps_chn_attr->m_chn_attr.frameRate.srcFrameRate = fps;
  vps_chn_attr->m_chn_attr.frameRate.dstFrameRate = fps;
  return 0;
}

