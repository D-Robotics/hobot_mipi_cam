/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 D-Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_VIO_VIN_H_
#define X3_VIO_VIN_H_

#include "stdint.h"
#include "stddef.h"
#include "x3_sdk_wrap.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include "vio/hb_isp_api.h"
int x3_vin_init(x3_vin_info_t *vin_info);
int x3_vin_start(x3_vin_info_t *vin_info);
void x3_vin_stop(x3_vin_info_t *vin_info);
void x3_vin_deinit(x3_vin_info_t *vin_info);
int x3_vin_feedback(int pipeId, hb_vio_buffer_t *feedback_buf);
int x3_vin_get_ouput(int pipeId, hb_vio_buffer_t *buffer);
int x3_vin_output_release(int pipeId, hb_vio_buffer_t *buffer);
int x3_vin_sif_raw_dump(int pipeId, char *file_name);
int x3_vin_isp_yuv_dump(int pipeId, char *file_name);

#ifdef __cplusplus
};
#endif /* __cplusplus */
#endif // X3_VIO_VIN_H_
