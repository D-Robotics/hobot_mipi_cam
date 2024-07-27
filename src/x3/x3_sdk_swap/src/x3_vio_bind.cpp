/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 D-Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

extern "C" {
#include "vio/hb_sys.h"
}


#include "logging.h"

#include "x3_vio_bind.h"
#include "x3_sdk_wrap.h"
#include <rclcpp/rclcpp.hpp>

int x3_vps_bind_vps(int vpsGrp, int vpsChn, int dstVpsGrp)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = dstVpsGrp;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_Bind failed\n");
    return s32Ret;
}

int x3_vps_bind_vot(int vpsGrp, int vpsChn, int votChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VOT;
	dst_mod.s32DevId = 0;
	dst_mod.s32ChnId = votChn;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_Bind failed\n");
    return s32Ret;
}

int x3_vps_bind_venc(int vpsGrp, int vpsChn, int vencChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VENC;
	dst_mod.s32DevId = vencChn;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_Bind failed\n");

    return s32Ret;
}

int x3_vin_bind_vps(int pipeId, int vpsGrp, int vin_vps_mode)
{
	int ret = 0;
	struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VIN;
	src_mod.s32DevId = pipeId;
	if (vin_vps_mode == VIN_ONLINE_VPS_ONLINE ||
		vin_vps_mode == VIN_OFFLINE_VPS_ONLINE||
		vin_vps_mode == VIN_SIF_ONLINE_DDR_ISP_ONLINE_VPS_ONLINE||
		vin_vps_mode == VIN_SIF_OFFLINE_ISP_OFFLINE_VPS_ONLINE ||
		vin_vps_mode == VIN_FEEDBACK_ISP_ONLINE_VPS_ONLINE ||
		vin_vps_mode == VIN_SIF_VPS_ONLINE)
		src_mod.s32ChnId = 1;
	else
		src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vpsGrp;
	dst_mod.s32ChnId = 0; /* vps 只有一个输入通道，所以只能是0 */
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
	  "[%s]->src s32DevId:%d src s32ChnId:%d dst s32DevId:%d dst s32ChnId:%d.\n",__func__,
		src_mod.s32DevId, src_mod.s32ChnId, dst_mod.s32DevId, dst_mod.s32ChnId);
	ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
		  "[x3_vin_bind_vps]->HB_SYS_Bind failed, ret:%d.\n",__func__, ret);

	return ret;
}

int x3_vps_unbind_vps(int vpsGrp, int vpsChn, int dstVpsGrp)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = dstVpsGrp;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_UnBind failed\n");

    return s32Ret;
}


int x3_vin_unbind_vps(int pipeId, int vpsGrp, int vin_vps_mode)
{
	int ret = 0;
	struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VIN;
	src_mod.s32DevId = pipeId;
	if (vin_vps_mode == VIN_ONLINE_VPS_ONLINE ||
		vin_vps_mode == VIN_OFFLINE_VPS_ONLINE||
		vin_vps_mode == VIN_SIF_ONLINE_DDR_ISP_ONLINE_VPS_ONLINE||
		vin_vps_mode == VIN_SIF_OFFLINE_ISP_OFFLINE_VPS_ONLINE ||
		vin_vps_mode == VIN_FEEDBACK_ISP_ONLINE_VPS_ONLINE ||
		vin_vps_mode == VIN_SIF_VPS_ONLINE)
		src_mod.s32ChnId = 1;
	else
		src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vpsGrp;
	dst_mod.s32ChnId = 0;
	ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_UnBind failed\n");

	return ret;
}

int x3_votwb_bind_vps(int vps_grp, int vps_chn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VOT;
	src_mod.s32DevId = 0;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vps_grp;
	dst_mod.s32ChnId = vps_chn;
	ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_Bind failed\n");

    return ret;
}

int x3_votwb_bind_venc(int venc_chn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
    src_mod.enModId = HB_ID_VOT;
    src_mod.s32DevId = 0;
    src_mod.s32ChnId = 0;
    dst_mod.enModId = HB_ID_VENC;
    dst_mod.s32DevId = venc_chn;
    dst_mod.s32ChnId = venc_chn;
    ret = HB_SYS_Bind(&src_mod, &dst_mod);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_Bind failed\n");
      return -1;
    }

    return 0;
}

int x3_votwb_unbind_vps(int vps_grp, int vps_chn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VOT;
	src_mod.s32DevId = 0;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vps_grp;
	dst_mod.s32ChnId = vps_chn;
	ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_Bind failed\n");

    return ret;
}

int x3_votwb_unbind_venc(int venc_chn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
    src_mod.enModId = HB_ID_VOT;
    src_mod.s32DevId = 0;
    src_mod.s32ChnId = 0;
    dst_mod.enModId = HB_ID_VENC;
    dst_mod.s32DevId = venc_chn;
    dst_mod.s32ChnId = venc_chn;
    ret = HB_SYS_UnBind(&src_mod, &dst_mod);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_Bind failed\n");
      return -1;
    }

    return 0;
}

int x3_vdec_bind_vot(int vdecChn, int votChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VOT;
	dst_mod.s32DevId = 0;
	dst_mod.s32ChnId = votChn;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_Bind failed\n");

    return s32Ret;
}

int x3_vdec_bind_vps(int vdecChn, int vpsGrp, int vpsChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vpsGrp;
	dst_mod.s32ChnId = vpsChn;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_Bind failed\n");

    return s32Ret;
}

int x3_vdec_bind_venc(int vdecChn, int vencChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VENC;
	dst_mod.s32DevId = vencChn;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_Bind failed\n");

    return s32Ret;
}

int x3_vdec_unbind_vot(int vdecChn, int votChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VOT;
	dst_mod.s32DevId = 0;
	dst_mod.s32ChnId = votChn;
	s32Ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_Bind failed\n");

    return s32Ret;
}

int x3_vdec_unbind_vps(int vdecChn, int vpsGrp, int vpsChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vpsGrp;
	dst_mod.s32ChnId = vpsChn;
	s32Ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_UnBind failed\n");

    return s32Ret;
}

int x3_vdec_unbind_venc(int vdecChn, int vencChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VENC;
	dst_mod.s32DevId = vencChn;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "HB_SYS_UnBind failed\n");

    return s32Ret;
}

