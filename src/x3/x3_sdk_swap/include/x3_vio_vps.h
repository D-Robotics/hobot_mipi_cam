/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 D-Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_VIO_VPS_H_
#define x3_VIO_VPS_H_
#include "vio/hb_vps_api.h"
#include "vio/hb_vio_interface.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void print_vps_chn_attr(VPS_CHN_ATTR_S *chn_attr);

int x3_vps_group_init(int vps_grp_id, VPS_GRP_ATTR_S *vps_grp_attr);
int x3_setpu_gdc(int vps_grp_id, char *gdc_config_file, ROTATION_E enRotation);
int x3_vps_chn_init(int vps_grp_id, int vps_chn_id, VPS_CHN_ATTR_S *chn_attr);
int x3_vps_start(uint32_t vpsGrpId);
void x3_vps_stop(int vpsGrpId);
void x3_vps_deinit(int vpsGrpId);
int x3_vps_input(uint32_t vpsGrpId, hb_vio_buffer_t *buffer);
int x3_vps_get_output(uint32_t vpsGrpId, int channel, hb_vio_buffer_t *buffer);
int x3_vps_output_release(uint32_t vpsGrpId, int channel,
        hb_vio_buffer_t *buffer);
void x3_normal_buf_info_print(hb_vio_buffer_t * buf);
int x3_dump_nv12(char *filename, char *srcBuf, char *srcBuf1,
		unsigned int size, unsigned int size1);
int x3_dump_vio_buf_to_nv12(char *filename, hb_vio_buffer_t *vio_buf);
int x3_save_jpeg(char *filename, char *srcBuf, unsigned int size);
int x3_dumpToFile(char *filename, char *srcBuf, unsigned int size);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif // X3_VIO_VPS_H_
