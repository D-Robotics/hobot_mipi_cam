/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 D-Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_VIO_BIND_H_
#define X3_VIO_BIND_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
int x3_vin_bind_vps(int pipeId, int vpsGrp, int vin_vps_mode);
int x3_vin_unbind_vps(int pipeId, int vpsGrp, int vin_vps_mode);
int x3_vps_bind_vps(int vpsGrp, int vpsChn, int dstVpsGrp);
int x3_vps_bind_vot(int vpsGrp, int vpsChn, int votChn);
int x3_vps_bind_venc(int vpsGrp, int vpsChn, int vencChn);
int x3_vps_unbind_vps(int vpsGrp, int vpsChn, int dstVpsGrp);
int x3_vps_unbind_vot(int vpsGrp, int vpsChn, int votChn);
int x3_vps_unbind_venc(int vpsGrp, int vpsChn, int vencChn);

int x3_votwb_bind_venc(int venc_chn);
int x3_votwb_bind_vps(int vps_grp, int vps_chn);
int x3_votwb_unbind_venc(int venc_chn);
int x3_votwb_unbind_vps(int vps_grp, int vps_chn);

int x3_vdec_bind_vps(int vdecChn, int vpsGrp, int vpsChn);
int x3_vdec_bind_vot(int vdecChn, int votChn);
int x3_vdec_bind_venc(int vdecChn, int vencChn);
int x3_vdec_unbind_vps(int vdecChn, int vpsGrp, int vpsChn);
int x3_vdec_unbind_vot(int vdecChn, int votChn);
int x3_vdec_unbind_venc(int vdecChn, int vencChn);

#ifdef __cplusplus
};
#endif /* __cplusplus */
#endif // X3_VIO_BIND_H_