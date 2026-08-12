#include "vp_sensors.h"

#define SENSOR_WIDTH  1280
#define SENSOR_HEIGHT  720
#define SENSOE_FPS 60
#define RAW10 0x2B

static mipi_config_t cv4006_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 2,
		.datatype = RAW10,
		.fps = SENSOE_FPS,
		.mclk = 24,
		//.mipiclk = 3500,
		//.mipiclk = 1700*2, // 24/2*0x27/2 = 468/2
        .mipiclk = 1020*2, // 24/2*0x27/2 = 468/2
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
		//.linelenth = 4400,
		//.framelenth = 2700,
		//.linelenth = 0x270*4,
		//.framelenth = 0x960,
        //
		//.linelenth = 0xbb8, 
	//	.framelenth = 0x320,
		.linelenth = 0x2c8*2, 
		.framelenth = 0x954,
		//.settle = 0,
		//.settle = 31,
		.settle = 22,
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

static camera_config_t cv4006_camera_config = {
	.name = "cv4006",
	.addr = 0x35, //shaokc..
	.sensor_mode = 1,
	.fps = SENSOE_FPS,
	.format = RAW10,
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.gpio_enable_bit = 0x01,
	.gpio_level_bit = 0x00,
	.mipi_cfg = &cv4006_mipi_config,
	.calib_lname = "/usr/hobot/lib/sensor/cv4006_tuning.json",
	.config_index = 0, //2lane
};

static vin_node_attr_t cv4006_vin_node_attr = {
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

static vin_attr_ex_t cv4006_vin_attr_ex = {
	.vin_attr_ex_mask = 0x80,
	.mclk_ex_attr = {
		.mclk_freq = 24000000,
	},
};

static vin_ichn_attr_t cv4006_vin_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.format = RAW10,
};

static vin_ochn_attr_t cv4006_vin_ochn_attr = {
	.ddr_en = 1,
	.ochn_attr_type = VIN_BASIC_ATTR,
	.vin_basic_attr = {
		.format = RAW10,
		// 硬件 stride 跟格式匹配，通过行像素根据raw数据bit位数计算得来
		// 8bit：x1, 10bit: x2 12bit: x2 16bit: x2,例raw10，1920 x 2 = 3840
		.wstride = (SENSOR_WIDTH) * 2,
	},
};

static isp_attr_t cv4006_isp_attr = {
	//.input_mode = 1, // 0: online, 1: mcm, 类似offline
	.input_mode = 2, // 0: online, 1: mcm, 类似offline
	.sensor_mode= ISP_NORMAL_M,
	.crop = {
		.x = 0,
		.y = 0,
		.h = SENSOR_HEIGHT,
		.w = SENSOR_WIDTH,
	},
};

static isp_ichn_attr_t cv4006_isp_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.fmt = FRM_FMT_RAW,
	.bit_width = 10,
};

static isp_ochn_attr_t cv4006_isp_ochn_attr = {
	.ddr_en = 1,
	.fmt = FRM_FMT_NV12,
	.bit_width = 8,
};

vp_sensor_config_t cv4006_linear_1280x720_raw10_60fps_2lane = {
	.chip_id_reg = 0x3002,
	.chip_id = 0x40,
	.sensor_i2c_addr_list = {0x35},
	.sensor_name = "cv4006-60fps-2lane",
	.config_file = "cv4006_linear_1280x720_raw10_60fps_2lane.c",
	.camera_config = &cv4006_camera_config,
	.vin_ichn_attr = &cv4006_vin_ichn_attr,
	.vin_node_attr = &cv4006_vin_node_attr,
	.vin_attr_ex   = &cv4006_vin_attr_ex,
	.vin_ochn_attr = &cv4006_vin_ochn_attr,
	.isp_attr      = &cv4006_isp_attr,
	.isp_ichn_attr = &cv4006_isp_ichn_attr,
	.isp_ochn_attr = &cv4006_isp_ochn_attr,
};
