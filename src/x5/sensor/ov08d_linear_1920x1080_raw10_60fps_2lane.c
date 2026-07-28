#include "vp_sensors.h"

#define SENSOR_WIDTH  1920
#define SENSOR_HEIGHT  1080
#define SENSOR_FPS 60
#define RAW10 0x2B

static mipi_config_t ov08d_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 2,
		.datatype = RAW10,
		.fps = SENSOR_FPS,
		.mclk = 24,
		.mipiclk = 1764,
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
		.linelenth = 2517,
		.framelenth = 1168,
		.settle = 20,
		.channel_num = 1,
		.channel_sel = {0},
	},
	.rx_ex_mask = 0x40,
	.rx_attr_ex = {
		.stop_check_instart = 1,
	}
};

static camera_config_t ov08d_camera_config = {
	.name = "ov08d",
	.addr = 0x10,
	.sensor_mode = 1,
	.fps = SENSOR_FPS,
	.format = RAW10,
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.gpio_enable_bit = 0x01,
	.gpio_level_bit = 0x00,
	.mipi_cfg = &ov08d_mipi_config,
	.calib_lname = "/usr/hobot/lib/sensor/ov08d_1920x1080_tuning.json",
};

static vin_node_attr_t ov08d_vin_node_attr = {
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
	.vin_attr_ex_mask = 0x00,
	.mclk_ex_attr = {
		.mclk_freq = 24000000,
	},
};

static vin_ichn_attr_t ov08d_vin_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.format = RAW10,
};

static vin_ochn_attr_t ov08d_vin_ochn_attr = {
	.ddr_en = 1,
	.ochn_attr_type = VIN_BASIC_ATTR,
	.vin_basic_attr = {
		.format = RAW10,
		.wstride = (SENSOR_WIDTH) * 2,
	},
};

static isp_attr_t ov08d_isp_attr = {
	.input_mode = 1,
	.sensor_mode= ISP_NORMAL_M,
	.crop = {
		.x = 0,
		.y = 0,
		.h = SENSOR_HEIGHT,
		.w = SENSOR_WIDTH,
	},
};

static isp_ichn_attr_t ov08d_isp_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.fmt = FRM_FMT_RAW,
	.bit_width = 10,
};

static isp_ochn_attr_t ov08d_isp_ochn_attr = {
	.ddr_en = 1,
	.fmt = FRM_FMT_NV12,
	.bit_width = 8,
};

vp_sensor_config_t ov08d_linear_1920x1080_raw10_60fps_2lane = {
	.chip_id_reg = 0x0000,
	.chip_id = 0x5608,
	.sensor_i2c_addr_list = {0x10},
	.sensor_name = "ov08d-1920x1080-60fps",
	.config_file = "linear_1920x1080_raw10_60fps_2lane.c",
	.camera_config = &ov08d_camera_config,
	.vin_ichn_attr = &ov08d_vin_ichn_attr,
	.vin_node_attr = &ov08d_vin_node_attr,
	.vin_attr_ex   = &vin_attr_ex,
	.vin_ochn_attr = &ov08d_vin_ochn_attr,
	.isp_attr      = &ov08d_isp_attr,
	.isp_ichn_attr = &ov08d_isp_ichn_attr,
	.isp_ochn_attr = &ov08d_isp_ochn_attr,
	.ex_chip_id = 0x5647,
};
