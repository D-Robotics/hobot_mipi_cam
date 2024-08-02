#include "vp_sensors.h"

#define SENSOR_WIDTH  640
#define SENSOR_HEIGHT  480
#define SENSOE_FPS 30
#define RAW10 0x2B

static mipi_config_t sc035hgs_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 1,
		.datatype = RAW10,
		.fps = SENSOE_FPS,
		.mclk = 1,
		.mipiclk = 600,
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
		.linelenth = 1600,
		.framelenth = 1250,
		.settle = 20,
		.channel_num = 1,
		.channel_sel = {0},
	},
};

static camera_config_t sc035hgs_camera_config = {
	.name = "sc035hgs",
	.addr = 0x30,
	.sensor_mode = 1,
	.fps = SENSOE_FPS,
	.format = RAW10,
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.gpio_enable_bit = 0x01,
	.gpio_level_bit = 0x00,
	.mipi_cfg = &sc035hgs_mipi_config,
	.calib_lname = "disable",
};

static vin_node_attr_t sc035hgs_vin_node_attr = {
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
	.lpwm_attr = {
		.enable = 1,
		.lpwm_chn_attr = {
			{	.trigger_source = 0,
				.trigger_mode = 0,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
			{	.trigger_source = 0,
				.trigger_mode = 0,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
			{	.trigger_source = 0,
				.trigger_mode = 0,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
			{	.trigger_source = 0,
				.trigger_mode = 0,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
		},
	},
};

static vin_ichn_attr_t sc035hgs_vin_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.format = RAW10,
};

static vin_ochn_attr_t sc035hgs_vin_ochn_attr = {
	// 使能数据输出至DDR
	.ddr_en = 0,
	.ochn_attr_type = VIN_BASIC_ATTR,
	.vin_basic_attr = {
		.format = RAW10,
		// 硬件 stride 跟格式匹配，通过行像素根据raw数据bit位数计算得来
		// 8bit：x1, 10bit: x2 12bit: x2 16bit: x2,例raw10，1920 x 2 = 3840
		.wstride = (SENSOR_WIDTH) * 2,
	},
};

static isp_attr_t sc035hgs_isp_attr = {
	.input_mode = 0,
	.sensor_mode= ISP_NORMAL_M,
	.crop = {
		.x = 0,
		.y = 0,
		.h = SENSOR_HEIGHT,
		.w = SENSOR_WIDTH,
	},
};

static isp_ichn_attr_t sc035hgs_isp_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.fmt = FRM_FMT_RAW,
	.bit_width = 10,
};

static isp_ochn_attr_t sc035hgs_isp_ochn_attr = {
	.ddr_en = 1,
	.fmt = FRM_FMT_NV12,
	.bit_width = 8,
};

vp_sensor_config_t sc035hgs_linear_640x480_raw10_30fps_1lane = {
	.chip_id_reg = 0x3107,
	.chip_id = 0x0035,
	.sensor_name = "sc035hgs",
	.config_file = "linear_640x480_raw10_30fps_1lane.c",
	.camera_config = &sc035hgs_camera_config,
	.vin_ichn_attr = &sc035hgs_vin_ichn_attr,
	.vin_node_attr = &sc035hgs_vin_node_attr,
	.vin_ochn_attr = &sc035hgs_vin_ochn_attr,
	.isp_attr      = &sc035hgs_isp_attr,
	.isp_ichn_attr = &sc035hgs_isp_ichn_attr,
	.isp_ochn_attr = &sc035hgs_isp_ochn_attr,
};
