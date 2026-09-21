#include "vp_sensors.h"

#define SENSOR_WIDTH  1920
#define SENSOR_HEIGHT 1200
#define SENSOR_FPS    30
#define RAW10         0x2B

#define TRG_HZ        30
#define TRG_PERIOD    (1000000 / (TRG_HZ))

/*
 * sc233hgs behind an HSMT deserializer (vendor reference: NS660x):
 * one MIPI RX carries up to 4 virtual channels (one per SerDes TX).
 *
 * Only ONE template config is registered. The per-camera VC selection is
 * assigned at runtime in HobotMipiCapIml::gsml_init():
 *   - vin cim_attr.vc_index       = pipeline_num % 4
 *   - camera_config->extra_mode   = pipeline_num % 4
 * libsc233hgs_hsmt.so reads extra_mode to decide which TX (tx0..tx3) to
 * route the sensor I2C traffic to, so extra_mode must stay equal to vc_index.
 *
 * The HSMT sensor driver manages the deserializer itself over I2C tunneling,
 * so no deserial node is needed in vflow (deserial_attr = NULL).
 */

static mipi_config_t sc233hgs_hsmt_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 4,               /* deserializer 4-lane output to SoC */
		.datatype = RAW10,
		.fps = SENSOR_FPS,
		.mclk = 24,
		.mipiclk = 2160,
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
		.linelenth = 2250,
		.framelenth = 1600,
		.settle = 0,
		.channel_num = 4,        /* 4 virtual channels on the aggregated MIPI RX */
		.channel_sel = {0, 1, 2, 3},
	},

	.rx_ex_mask = 0x41,
	.rx_attr_ex = {
		.stop_check_instart = 1,
		.nocheck = 1,
	},

	.end_flag = MIPI_CONFIG_END_FLAG,
};

static camera_config_t sc233hgs_hsmt_camera_config = {
	.name = "sc233hgs_hsmt",       /* matches libsc233hgs_hsmt.so */
	.addr = 0x30,
	.eeprom_addr = 0x50,   /* 实测EEPROM在7-bit 0x50（8-bit 0xA0），框架按此地址做CAMERA_EEPROM_REG访问 */
	.serial_addr = 0x40,
	.sensor_mode = 6,              /* SLAVE_M: external-trigger slave mode */
	.fps = SENSOR_FPS,
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.format = RAW10,
	.gpio_enable = 0x00,  /* HSMT: sensor GPIOs are behind the deserializer, not on SoC vcon */
	.gpio_level = 0x00,
	.extra_mode = 0,               /* template value; overwritten at runtime to the VC index
	                                   (kept equal to vc_index, consumed by libsc233hgs_hsmt.so) */
	.config_index = 0,
	.calib_lname = "/usr/hobot/lib/sensor/lib_sc233hgs_linear.so",
	.mipi_cfg = &sc233hgs_hsmt_mipi_config,
	.end_flag = CAMERA_CONFIG_END_FLAG,
};

static isp_cfg_t sc233hgs_hsmt_isp_cfg = {
	.isp_attr = {
		.channel = {
			.hw_id = 3,
			.slot_id = 4,
			.ctx_id = -1,
		},
		.sched_mode = 1,
		.work_mode = 0,
		.hdr_mode = 0,
		.size = {
			.width = SENSOR_WIDTH,
			.height = SENSOR_HEIGHT,
		},
		.frame_rate = SENSOR_FPS,
		.isp_combine = {
			.isp_channel_mode = 0,
			.bind_channel = {
				.bind_hw_id = 0,
				.bind_slot_id = 0,
			},
		},
		.isp_sw_ctrl = {
			.ae_stat_buf_en = 1,
			.awb_stat_buf_en = 1,
			.ae5bin_stat_buf_en = 1,
			.ctx_buf_en = 0,
			.pixel_consistency_en = 0,
		},
		.algo_state = 1,
		.clear_record = 0,
	},
	.ochn_attr = {
		.stream_output_mode = 1,
		.axi_output_mode = 9,       /* COVERT_AXI_OUTPUT_MODE_YUV420 */
		.output_crop_cfg = {
			.enable = 0,
			.rect = {
				.x = 0,
				.y = 0,
				.width = 0,
				.height = 0,
			},
		},
		.output_raw_level = 0,
		.out_buf_noinvalid = 1,
		.out_buf_noncached = 0,
		.buf_num = 3,
	},
	.ichn_attr = {
		.input_crop_cfg = {
			.enable = 0,
			.rect = {
				.x = 0,
				.y = 0,
				.width = 0,
				.height = 0,
			},
		},
		.in_buf_noclean = 1,
		.in_buf_noncached = 0,
	},
};

static vin_attr_t sc233hgs_hsmt_vin_attr = {
	.vin_node_attr = {
		.vcon_attr = {
			.bus_main = 2,
			.bus_second = 2,
		},

		.cim_attr = {
			.mipi_en = 1,
			.cim_isp_flyby = 0,
			.cim_pym_flyby = 0,
			.mipi_rx = 0,
			.vc_index = 0,             /* template value; overwritten at runtime to pipeline_num % 4 */
			.ipi_channels = 1,
			.y_uv_swap = 0,
			.func = {
				.enable_frame_id = 1,
				.set_init_frame_id = 1,
				.enable_pattern = 0,
				.lpwm_trig_sel = 0,   /* 关联LPWM通道0做触发时间戳（对应X5的time_stamp_mode=TS_IPI_TRIGGER语义） */
			},
			.rdma_input = {
				.rdma_en = 0,
				.stride = 0,
				.pack_mode = 1,
				.buff_num = 6,
			},
		},
		.lpwm_attr = {
			.lpwm_chn_attr = {
				{ .enable = 1, .trigger_source = 0, .trigger_mode = 0,
				  .period = TRG_PERIOD, .offset = 10, .duty_time = 100,
				  .threshold = 0, .adjust_step = 0, },
				{ .enable = 1, .trigger_source = 0, .trigger_mode = 0,
				  .period = TRG_PERIOD, .offset = 10, .duty_time = 100,
				  .threshold = 0, .adjust_step = 0, },
				{ .enable = 1, .trigger_source = 0, .trigger_mode = 0,
				  .period = TRG_PERIOD, .offset = 10, .duty_time = 100,
				  .threshold = 0, .adjust_step = 0, },
				{ .enable = 1, .trigger_source = 0, .trigger_mode = 0,
				  .period = TRG_PERIOD, .offset = 10, .duty_time = 100,
				  .threshold = 0, .adjust_step = 0, },
			},
		},
		.magicNumber = MAGIC_NUMBER,
	},

	.vin_ichn_attr = {
		.format = RAW10,
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
	},

	.vin_attr_ex = {
		.cim_static_attr = {
			.water_level_mark = 0,
		},
	},

	.vin_ochn_attr = {
		[VIN_MAIN_FRAME] = {
			.ddr_en = 1,
			.pingpong_ring = 1,
			.vin_basic_attr = {
				.format = RAW10,
				.wstride = 0,
				.vstride = 0,
				.pack_mode = 1,
			},
			.roi_en = 0,
			.roi_attr = {
				.roi_x = 1280,
				.roi_y = 720,
				.roi_width = 64,
				.roi_height = 64,
			},
			.rawds_en = 0,
			.rawds_attr = {
				.rawds_mode = 0,
			},
			.magicNumber = MAGIC_NUMBER,
		},
	},
	.magicNumber = MAGIC_NUMBER,
};

static struct ynr_init_attr sc233hgs_hsmt_ynr_attr = {
	.work_mode = 1,
	.slot_id = 4,

	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.nr_static_switch = 0b11, // (nr3d_en << 1) | (nr2d_en);
	.in_stride = {
		SENSOR_WIDTH, SENSOR_HEIGHT
	},
	.nr2d_en = 1,
	.nr3d_en = 0,   /* 厂商样例未经过YNR验证；先按sc132gs已验证组合(仅2DNR)打通链路 */

	.dma_output_en = 1, // nr3d_en

	.debug_en = 0,
};

static pym_cfg_t sc233hgs_hsmt_pym_config = {
	.hw_id = 3,
	.pym_mode = 1,
	.slot_id = 4,
	.axi_burst_len = 0,
	.in_linebuff_watermark = 0,
	.out_buf_noinvalid = 1,
	.out_buf_noncached = 0,
	.in_buf_noclean = 1,
	.in_buf_noncached = 0,
	.buf_consecutive = 0,
	.pingpong_ring = 0,
	.output_buf_num = 6,
	.timeout = 0,
	.threshold_time = 0,
	.layer_num_trans_next = 0,
	.layer_num_share_prev = -1,
	.chn_ctrl = {
		.pixel_num_before_sol = 2,
		.invalid_head_lines = 0,
		.src_in_width = SENSOR_WIDTH,
		.src_in_height = SENSOR_HEIGHT,
		.src_in_stride_y = SENSOR_WIDTH,
		.src_in_stride_uv = SENSOR_WIDTH,
		.suffix_hb_val = 100,
		.prefix_hb_val = 2,
		.suffix_vb_val = 10,
		.prefix_vb_val = 0,
		.bl_max_layer_en = 5,
		.ds_roi_en = 1,
		.ds_roi_uv_bypass = 0,
		.ds_roi_info = {
			[0] = {
				.start_top = 0,
				.start_left = 0,
				.region_width = SENSOR_WIDTH,
				.region_height = SENSOR_HEIGHT,
				.wstride_uv = SENSOR_WIDTH,
				.wstride_y = SENSOR_WIDTH,
				.vstride = SENSOR_HEIGHT,
				.step_v = 0,
				.step_h = 0,
				.out_width = SENSOR_WIDTH,
				.out_height = SENSOR_HEIGHT,
				.phase_y_v = 0,
				.phase_y_h = 0,
			},
		},
	},
	.fb_buf_num = 2,
	.magicNumber = MAGIC_NUMBER,
};

vp_sensor_config_t hsmt_sc233hgs_linear_1920x1200_raw10_30fps_4lane = {
	.chip_id_reg = 0x3107,
	.chip_id = 0xcb61,
	.sensor_i2c_addr_list = {0x30},
	.sensor_type = SENSOR_TYPE_HSMT_RAW,
	.sensor_name = "sc233hgs_hsmt",
	.config_file = "hsmt_sc233hgs_linear_1920x1200_raw10_30fps_4lane.c",
	.camera_config = &sc233hgs_hsmt_camera_config,
	.camera_slave_config = NULL,
	.deserial_attr = NULL,
	.deserial_slave_attr = NULL,
	.vin_attr = &sc233hgs_hsmt_vin_attr,
	.isp_cfg = &sc233hgs_hsmt_isp_cfg,
	.ynr_attr = &sc233hgs_hsmt_ynr_attr,
	.pym_cfg = &sc233hgs_hsmt_pym_config,
};
