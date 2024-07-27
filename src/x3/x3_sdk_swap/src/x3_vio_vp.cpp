/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 D-Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "stdint.h"
#include <stdio.h>
#include <unistd.h>

//#include "utils/utils_log.h"

#include <cstring>
#include "vio/hb_sys.h"
#include "vio/hb_vp_api.h"

#include "x3_vio_vp.h"
#include <rclcpp/rclcpp.hpp>

int x3_vp_init() {
    VP_CONFIG_S struVpConf;
    memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
    struVpConf.u32MaxPoolCnt = 32; // 整个系统中可以容纳缓冲池的个数
    HB_VP_SetConfig(&struVpConf);

    int ret = HB_VP_Init();
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
      "[%s]->ret %d.", __func__, ret);
    if (!ret) {
       RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
         "hb_vp_init success.\n");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "hb_vp_init failed, ret: %d.\n", ret);
    }
    return ret;
}

int x3_vp_alloc(vp_param_t *param) {
	int i = 0, ret = 0;
    vp_param_t *vp_param = (vp_param_t *)param;

    for (i = 0; i < vp_param->mmz_cnt; i++) {
        ret = HB_SYS_Alloc(&vp_param->mmz_paddr[i],
            (void **)&vp_param->mmz_vaddr[i], vp_param->mmz_size);
        if (!ret) {
            RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
                    "mmzAlloc paddr = 0x%x, vaddr = 0x%x, %d/%d , %d",
                    vp_param->mmz_paddr[i], vp_param->mmz_vaddr[i], i,
                    vp_param->mmz_cnt, vp_param->mmz_size);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
              "hb_vp_alloc failed, ret: %d", ret);
            return -1;
        }
    }
    return 0;
}

int x3_vp_free(vp_param_t *param) {
	int i = 0, ret = 0;
    vp_param_t * vp_param = (vp_param_t *)param;
    for (i = 0; i < vp_param->mmz_cnt; i++) {
        ret = HB_SYS_Free(vp_param->mmz_paddr[i], vp_param->mmz_vaddr[i]);
        if (ret == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
                "mmzFree paddr = 0x%x, vaddr = 0x%x i = %d",
                vp_param->mmz_paddr[i], vp_param->mmz_vaddr[i], i);
        }
    }
    return 0;
}

int x3_vp_deinit() {
    int ret = HB_VP_Exit();
    if (ret != 0) {
        RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
          "hb_vp_deinit success\n");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "hb_vp_deinit failed, ret: %d\n", ret);
    }
    return ret;
}
