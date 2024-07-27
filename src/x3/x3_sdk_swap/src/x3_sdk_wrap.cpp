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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>

//#include "utils/utils_log.h"

#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_vio_vp.h"
#include "x3_sdk_wrap.h"
#include <rclcpp/rclcpp.hpp>

// 打印 vin isp vpu venc等模块的调试信息
void print_debug_infos(void)
{
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
	  "========================= SIF ==========================\n");
	system("cat /sys/devices/platform/soc/a4001000.sif/cfg_info");
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
	  "========================= ISP ==========================\n");
	system("cat /sys/devices/platform/soc/b3000000.isp/isp_status");
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
	  "========================= IPU PIPE enable ==============\n");
	system("cat /sys/devices/platform/soc/a4040000.ipu/info/enabled_pipeline");
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
	  "========================= IPU PIPE config ==============\n");
	system("cat /sys/devices/platform/soc/a4040000.ipu/info/pipeline0_info");
	system("cat /sys/devices/platform/soc/a4040000.ipu/info/pipeline1_info");
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
	  "========================= VENC =========================\n");
	system("cat /sys/kernel/debug/vpu/venc");
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
	  "========================= VDEC =========================\n");
	system("cat /sys/kernel/debug/vpu/vdec");
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
	  "========================= JENC =========================\n");
	system("cat /sys/kernel/debug/jpu/jenc");
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
	  "========================= IAR ==========================\n");
	system("cat /sys/kernel/debug/iar");
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
	  "========================= ION ==========================\n");
	system("cat /sys/kernel/debug/ion/heaps/carveout");
	system("cat /sys/kernel/debug/ion/heaps/ion_cma");
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
	  "========================= END ===========================\n");
}


/* 默认只使能一个输出通道，需要多输出通道的场景，另外再调用x3_vps_init_chn初始化和使能 */
int x3_vps_init_wrap(x3_vps_info_t *vps_info)
{
	int ret = 0;
	int i = 0;
	// 创建group
	ret = x3_vps_group_init(vps_info->m_vps_grp_id, &vps_info->m_vps_grp_attr);
	if (ret) return ret;
#if 0
	// 初始化gdc
	if (vps_info->m_need_gdc) {
		ret = x3_setpu_gdc(vps_info->m_vps_grp_id, vps_info->m_gdc_config, vps_info->m_rotation);
		if (ret) {
			HB_VPS_DestroyGrp(vps_info->m_vps_grp_id);
			return ret;
		}
	}
#endif
	// 初始化配置的vps channal
	for (i = 0; i < vps_info->m_chn_num; i++){
		if (vps_info->m_vps_chn_attrs[i].m_chn_enable) {
			ret = x3_vps_chn_init(vps_info->m_vps_grp_id, vps_info->m_vps_chn_attrs[i].m_chn_id,
				&vps_info->m_vps_chn_attrs[i].m_chn_attr);
			RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
			    "[%s]->vps chn%d/%d init ret=%d.\n",
				__func__, vps_info->m_vps_chn_attrs[i].m_chn_id, vps_info->m_chn_num,ret);
			if (ret) {
				HB_VPS_DestroyGrp(vps_info->m_vps_grp_id);
				return ret;
			}
		}
	}
	return ret;

}

void x3_vps_uninit_wrap(x3_vps_info_t *vps_info)
{
	x3_vps_deinit(vps_info->m_vps_grp_id);
}
