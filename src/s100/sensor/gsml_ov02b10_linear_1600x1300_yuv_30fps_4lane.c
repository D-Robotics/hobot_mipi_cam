#include "vp_sensors.h"
// #include "vp_pym.h"

#define SENSOR_FPS 30

#define SENSOR_WIDTH 1600
#define SENSOR_HEIGHT 1300
#define RAW10 30
#define SETTLE 16
#define MIPILINE  2000
#define MIPIFRAME 2673
#define MAGIC_NUMBER 0x12345678

#if 0
//0820c 的sample 配置
static mipi_config_t ov02b10std_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 4,
		.datatype = 30,
		.fps = 30,
		.mclk = 24,
		.mipiclk = 810,
		.width = 1600,
		.height = 1300,
		.linelenth = 2149,
		.framelenth = 1125 * 2,
		.settle = 22,
		.channel_num = 2,
		.channel_sel[0] = 0,
		.channel_sel[1] = 1,
	},
	.end_flag = MIPI_CONFIG_END_FLAG,
};
#endif

static camera_config_t gsml_ov02b10std_camera_config = {
    .name = "ov02b10std",
    .addr =0x48,
    .eeprom_addr = 0x53,
    .serial_addr = 0x40,
    .sensor_mode = 0x05,
    .fps = SENSOR_FPS,
    .width = SENSOR_WIDTH,
    .height = SENSOR_HEIGHT,
    // .format = RAW10,
    .extra_mode = 1,
    .config_index = 2560,
    // .mipi_cfg = &g_mipi_config, // MIPI配置,NULL自动获取
    .calib_lname = "disable",
    .end_flag = CAMERA_CONFIG_END_FLAG,	
};

static camera_config_t gsml_ov02b10std_camera_config_slave = {
    .name = "dummystd",
    .addr = 0xff,
    .eeprom_addr = 0xff,
    .serial_addr = 0xff,
    .sensor_mode = 0x5,
    .fps = SENSOR_FPS,
    .width = SENSOR_WIDTH,
    .height = SENSOR_HEIGHT,
    //.format = 10,	//Notice
    .extra_mode = 22,
    .config_index = 0,
    //.mipi_cfg = &g_mipi_config,
    .calib_lname = "disable",
    .sensor_param = 
    "{"
        "\"tuning_data\": {"
            "\"enable\": 1,"
            "\"VMAX\": 5400,"
            "\"HMAX\": 0,"
            "\"gain_max\": 104856,"
            "\"analog_gain_max\": 104856,"
            "\"digital_gain_max\": 0,"
            "\"exposure_time_min\": 1,"
            "\"exposure_time_max\": 4000,"
            "\"exposure_time_long_max\": 4000,"
            "\"lines_per_second\": 162000,"
            "\"turning_type\": 6,"
            "\"conversion\": 0,"
            "\"data_width\": 10,"
            "\"bayer_start\": 3,"
            "\"bayer_pattern\": 0,"
            "\"exposure_max_bit_width\": 12"
        "}"
    "}",
    .end_flag = CAMERA_CONFIG_END_FLAG,	
};

static poc_config_t g_poc_cfg[] = {
	[0] = {
		/* 0 */
		.addr = 0x28,
		.poc_map = 0x1320,
		.end_flag = POC_CONFIG_END_FLAG,
	},
    [1] = {
		/* 0 */
		.addr = 0x28,
		.poc_map = 0x3210,
		.end_flag = POC_CONFIG_END_FLAG,
	},
};

static deserial_config_t gsml_ov02b10std_deserial_config = {
    .name = "max96712",
    .link_desp[0] = "ov02b10std:0@512",
    .link_desp[1] = "ov02b10std:0@512",
    .link_desp[2] = "ov02b10std:0@512",
    .link_desp[3] = "ov02b10std:0@512",
    .gpio_mfp[CAMERA_DES_GPIO_TRIG0] = 0x5,
    .gpio_mfp[CAMERA_DES_GPIO_TRIG1] = 0x5,
    .gpio_mfp[CAMERA_DES_GPIO_TRIG2] = 0x5,
    .gpio_mfp[CAMERA_DES_GPIO_TRIG3] = 0x5,
    .addr = 0x29,
    .poc_cfg = &g_poc_cfg[0],
    .end_flag = DESERIAL_CONFIG_END_FLAG,
};

static deserial_config_t gsml_ov02b10std_deserial_config_slave = {
        .name = "max96712",
        .link_desp[0] = "ov02b10std:0@512",
        .link_desp[1] = "ov02b10std:0@512",
        .link_desp[2] = "ov02b10std:0@512",
        .link_desp[3] = "ov02b10std:0@512",
        .gpio_mfp[CAMERA_DES_GPIO_TRIG0] = 0x5,
        .gpio_mfp[CAMERA_DES_GPIO_TRIG1] = 0x5,
        .gpio_mfp[CAMERA_DES_GPIO_TRIG2] = 0x5,
        .gpio_mfp[CAMERA_DES_GPIO_TRIG3] = 0x5,
        .addr = 0x29,
        .poc_cfg = &g_poc_cfg[1],
        .end_flag = DESERIAL_CONFIG_END_FLAG,        
};

static vin_attr_t gsml_ov02b10std_vin_attr = {
    .vin_node_attr = {
        .vcon_attr = {
            .bus_main =3,
            .bus_second = 3,
        },

        .cim_attr = {
            .mipi_en = 1,
            .cim_isp_flyby = 0,
            .cim_pym_flyby = 0,
            .mipi_rx = 1,
            .vc_index = 0,
            .ipi_channels = 1,
            .y_uv_swap = 0, //(uint32_t)vpf_get_json_value(p_node_mipi, "y_uv_swap");
            .func = {
                .enable_frame_id = 1,
                .set_init_frame_id = 1,
                .enable_pattern = 0,
                .lpwm_trig_sel = (int32_t)LPWM_CHN_INVALID,
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
                {	.enable = 1,
                    .trigger_source = 0,
                    .trigger_mode = 1,
                    .period = 33333,
                    .offset = 10,
                    .duty_time = 100,
                    .threshold = 0,
                    .adjust_step = 0,
                },
                {	.enable = 1,
                    .trigger_source = 10,
                    .trigger_mode = 1,
                    .period = 33333,
                    .offset = 10,
                    .duty_time = 100,
                    .threshold = 0,
                    .adjust_step = 0,
                },
                {	.enable = 1,
                    .trigger_source = 10,
                    .trigger_mode = 1,
                    .period = 33333,
                    .offset = 10,
                    .duty_time = 100,
                    .threshold = 0,
                    .adjust_step = 0,
                },
                {	.enable = 1,
                    .trigger_source = 10,
                    .trigger_mode = 1,
                    .period = 33333,
                    .offset = 10,
                    .duty_time = 100,
                    .threshold = 0,
                    .adjust_step = 0,
                },
            },
        },
        .magicNumber = MAGIC_NUMBER,
    },

    .vin_ichn_attr = {
        .width =  SENSOR_WIDTH,
        .height = SENSOR_HEIGHT,
        .format = RAW10,
    },

    .vin_attr_ex = {
        .cim_static_attr = {
            .water_level_mark = 0,
        },
    },

    .vin_ochn_attr = {
        [VIN_MAIN_FRAME] = { //vin_ochn0_attr
            .ddr_en = 1,
            .vin_basic_attr = {
                .format = RAW10,
                .wstride = 0,
                .vstride = 0,
                .pack_mode = 1,
            },
            .pingpong_ring = 1,
            .roi_en = 0,
            .roi_attr = {
                .roi_x = 0,
                .roi_y = 0,
                .roi_width = SENSOR_WIDTH,
                .roi_height = SENSOR_HEIGHT,
            },
            .rawds_en = 0,
            .rawds_attr = {
                .rawds_mode = 0,
            },
            .magicNumber = MAGIC_NUMBER,
        },
    },
    .vin_ochn_buff_attr = {
        [VIN_MAIN_FRAME] = { //vin_ochn0_buff_attr
            .buffers_num = 6,
        },
        [VIN_EMB] = { //vin_ochn3_buff_attr
            .buffers_num = 6,
        },
        [VIN_ROI] = { //vin_ochn4_buff_attr
            .buffers_num = 6,
        },
	},
    .magicNumber = MAGIC_NUMBER,
};

static isp_cfg_t gsml_ov02b10std_isp_cfg = {
    .isp_attr = {
        .channel = {
            .hw_id = 1,
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
            .pixel_consistency_en = 0
        },
        .algo_state = 1,
        .clear_record = 0,
    },
    .ochn_attr = {
        .stream_output_mode = 0,
        .axi_output_mode = AXI_OUTPUT_MODE_YUV420,
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
        .buf_num = 6,
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

struct ynr_init_attr gsml_ov02b10std_ynr_attr = {
	.work_mode = 1,
	.slot_id = 4,

	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.nr_static_switch = 0b11, // (nr3d_en << 1) | (nr2d_en);
	.in_stride = {
		SENSOR_WIDTH, SENSOR_HEIGHT
	},
	.nr2d_en = 1,
	.nr3d_en = 0,

	.dma_output_en = 1, // nr3d_en

	.debug_en = 0,    
};

pym_cfg_t gsml_ov02b10std_pym_common_config = {
    .hw_id = 1,
    .pym_mode = 1,
    .slot_id = 5,
    .pingpong_ring = 0,
    .output_buf_num = 6,
    .fb_buf_num = 2,
    .timeout = 0,
    .threshold_time = 0,
    .layer_num_trans_next = 0,
    .layer_num_share_prev = -1,
    .out_buf_noinvalid = 1,
    .out_buf_noncached = 0,
    .in_buf_noclean = 1,
    .in_buf_noncached = 0,
    .chn_ctrl = {
        //.pixel_num_before_sol = DEF_PIX_NUM_BF_SOL,
        .invalid_head_lines = 0,
        .src_in_width = SENSOR_WIDTH,
        .src_in_height = SENSOR_HEIGHT,
        .src_in_stride_y = SENSOR_WIDTH,
        .src_in_stride_uv = SENSOR_WIDTH,
        .suffix_hb_val = 68,
        .prefix_hb_val = 2,
        .suffix_vb_val = 20,
        .prefix_vb_val = 2,
        .ds_roi_en = 1,
        .bl_max_layer_en = 5,
        .ds_roi_uv_bypass = 0,
        .ds_roi_sel = {
            [0] = 0,
        },
        .ds_roi_layer = {
            [0] = 0,
        },
        .ds_roi_info = {
            [0] = {
                .start_left = 0,
                .start_top = 0,
                .region_width = SENSOR_WIDTH,
                .region_height = SENSOR_HEIGHT,
                .wstride_uv = SENSOR_WIDTH,
                .wstride_y = SENSOR_WIDTH,
                .out_width = SENSOR_WIDTH,
                .out_height = SENSOR_HEIGHT,
                .vstride = SENSOR_HEIGHT, //.out_height,
            },
        },
    },
.magicNumber = MAGIC_NUMBER,
};


vp_sensor_config_t gsml_ov02b10std_linear_1600x13000_yuv_30fps_4lane = {
	.chip_id_reg = 0,
	.chip_id = 0x031,
	.sensor_i2c_addr_list = {0x1a},
    .sensor_type = SENSOR_TYPE_GMSL_YUV,
	.sensor_name = "ov02b10-1300p25",
	.config_file = "linear_1600x13000_yuv_30fps_4lane.c",
	.camera_config = &gsml_ov02b10std_camera_config,
    .camera_slave_config = &gsml_ov02b10std_camera_config_slave,
    .deserial_attr = &gsml_ov02b10std_deserial_config,
    .deserial_slave_attr = &gsml_ov02b10std_deserial_config_slave,
	.vin_attr = &gsml_ov02b10std_vin_attr,
	.isp_cfg  = NULL,
    .ynr_attr = NULL,
	.pym_cfg = NULL,
};
