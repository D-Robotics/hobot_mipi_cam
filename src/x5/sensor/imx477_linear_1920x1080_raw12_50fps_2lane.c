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

#include "vp_sensors.h"

#define SENSOR_WIDTH  1920
#define SENSOR_HEIGHT  1080
#define SENSOE_FPS 50
#define RAW12 0x2C

static mipi_config_t imx477_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 2,
		.datatype = RAW12,
		.fps = SENSOE_FPS,
		.mclk = 24,
		.mipiclk = 2250,
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
		.linelenth = 2976,//3008
		.framelenth = 1313,//1314
		.settle = 30,
		.channel_num = 1,
		.channel_sel = {0},
	},
	.rx_ex_mask = 0x40,
	.rx_attr_ex = {
		.stop_check_instart = 1,
	}
};

static camera_config_t imx477_camera_config = {
	.name = "imx477",
	.addr = 0x1a,
	.sensor_mode = 1,
	.fps = SENSOE_FPS,
	.format = RAW12,
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.gpio_enable_bit = 0x01,
	.gpio_level_bit = 0x00,
	.mipi_cfg = &imx477_mipi_config,
	.calib_lname = "/usr/hobot/bin/imx477_1920x1080_tuning.json",
};

static vin_node_attr_t imx477_vin_node_attr = {
	.cim_attr = {
		.mipi_rx = 2,
		.vc_index = 0,
		.ipi_channel = 1,
		.cim_isp_flyby = 1,
		.func = {
			.enable_frame_id = 1,
			.set_init_frame_id = 0,
			.hdr_mode = NOT_HDR,
			.time_stamp_en = 0,
		},

	},
};

static vin_attr_ex_t vin_attr_ex = {
	.vin_attr_ex_mask = 0x80,
	.mclk_ex_attr = {
		.mclk_freq = 24000000,
	},
};

static vin_ichn_attr_t imx477_vin_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.format = RAW12,
};

static vin_ochn_attr_t imx477_vin_ochn_attr = {
	.ddr_en = 1,
	.ochn_attr_type = VIN_BASIC_ATTR,
	.vin_basic_attr = {
		.format = RAW12,
		// 硬件 stride 跟格式匹配，通过行像素根据raw数据bit位数计算得来
		// 8bit：x1, 10bit: x2 12bit: x2 16bit: x2,例raw10，1920 x 2 = 3840
		.wstride = (SENSOR_WIDTH) * 2,
	},
};

static isp_attr_t imx477_isp_attr = {
	.input_mode = 1, // 0: online, 1: mcm, 类似offline
	.sensor_mode= ISP_NORMAL_M,
	.crop = {
		.x = 0,
		.y = 0,
		.h = SENSOR_HEIGHT,
		.w = SENSOR_WIDTH,
	},
};

static isp_ichn_attr_t imx477_isp_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.fmt = FRM_FMT_RAW,
	.bit_width = 12,
};

static isp_ochn_attr_t imx477_isp_ochn_attr = {
	.ddr_en = 1,
	.fmt = FRM_FMT_NV12,
	.bit_width = 8,
};

vp_sensor_config_t imx477_linear_1920x1080_raw12_50fps_2lane = {
	.chip_id_reg = 0x0016,
	.chip_id = 0x0477,
	.sensor_i2c_addr_list = {0x1A},
	.sensor_name = "imx477-1920x1080-50fps",
	.config_file = "linear_1920x1080_raw12_50fps_2lane.c",
	.camera_config = &imx477_camera_config,
	.vin_ichn_attr = &imx477_vin_ichn_attr,
	.vin_node_attr = &imx477_vin_node_attr,
	.vin_attr_ex   = &vin_attr_ex,
	.vin_ochn_attr = &imx477_vin_ochn_attr,
	.isp_attr      = &imx477_isp_attr,
	.isp_ichn_attr = &imx477_isp_ichn_attr,
	.isp_ochn_attr = &imx477_isp_ochn_attr,
};
