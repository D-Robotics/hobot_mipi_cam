#include "vp_sensors.h"

#define SENSOR_WIDTH  3840
#define SENSOR_HEIGHT  2160
#define SENSOE_FPS 30
#define RAW10 0x2B

static mipi_config_t imx415_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 4,
		.datatype = RAW10,
		.fps = SENSOE_FPS,
		.mclk = 24,
		.mipiclk = 2880,
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
		.linelenth = 4400,
		.framelenth = 2700,
		.settle = 10,
		.channel_num = 1,
		.channel_sel = {0},
		.hsdTime = 0,
		.hsaTime = 0,
		.hbpTime = 0,
	},
	.rx_ex_mask = 0x40,
	.rx_attr_ex = {
		.stop_check_instart = 1,
	}
};

static camera_config_t imx415_camera_config = {
	.name = "imx415",
	.addr = 0x1a,
	.sensor_mode = 1,
	.fps = SENSOE_FPS,
	.format = RAW10,
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.gpio_enable_bit = 0x01,
	.gpio_level_bit = 0x00,
	.mipi_cfg = &imx415_mipi_config,
	.calib_lname = "disable",
};

static vin_node_attr_t imx415_vin_node_attr = {
	.cim_attr = {
		.mipi_rx = 0,
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

static vin_ichn_attr_t imx415_vin_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.format = RAW10,
};

static vin_ochn_attr_t imx415_vin_ochn_attr = {
	.ddr_en = 1,
	.ochn_attr_type = VIN_BASIC_ATTR,
	.vin_basic_attr = {
		.format = RAW10,
		// 硬件 stride 跟格式匹配，通过行像素根据raw数据bit位数计算得来
		// 8bit：x1, 10bit: x2 12bit: x2 16bit: x2,例raw10，1920 x 2 = 3840
		.wstride = (SENSOR_WIDTH) * 2,
	},
};

static isp_attr_t imx415_isp_attr = {
	.input_mode = 1, // 0: online, 1: mcm, 类似offline
	.sensor_mode= ISP_NORMAL_M,
	.crop = {
		.x = 0,
		.y = 0,
		.h = SENSOR_HEIGHT,
		.w = SENSOR_WIDTH,
	},
};

static isp_ichn_attr_t imx415_isp_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.fmt = FRM_FMT_RAW,
	.bit_width = 10,
};

static isp_ochn_attr_t imx415_isp_ochn_attr = {
	.ddr_en = 1,
	.fmt = FRM_FMT_NV12,
	.bit_width = 8,
};

vp_sensor_config_t imx415_linear_3480x2160_raw10_30fps_4lane = {
	.chip_id_reg = 0x4001,
	.chip_id = 0x03,
	.sensor_i2c_addr_list = {0x1A},
	.sensor_name = "imx415-30fps",
	.config_file = "linear_3840x2160_raw10_30fps_4lane.c",
	.camera_config = &imx415_camera_config,
	.vin_ichn_attr = &imx415_vin_ichn_attr,
	.vin_node_attr = &imx415_vin_node_attr,
	.vin_ochn_attr = &imx415_vin_ochn_attr,
	.isp_attr      = &imx415_isp_attr,
	.isp_ichn_attr = &imx415_isp_ichn_attr,
	.isp_ochn_attr = &imx415_isp_ochn_attr,
};
