#include "vp_sensors.h"

#define SENSOR_WIDTH  1920
#define SENSOR_HEIGHT  1080
#define SENSOE_FPS 30
#define RAW10 0x2B

// imx219 的sample配置
static mipi_config_t imx219_mipi_config = {
    .rx_enable = 1,
    .rx_attr = {
        .phy = 0,
        .lane = 2,
        .datatype = 0x12b,
        .fps = 30,
        .mclk = 24,
        .mipiclk = 1728,
        .width = 0,
        .height = 0,
        .linelenth = 0,
        .framelenth = 0,
        .settle = 0,
        .channel_num = 0,
        .channel_sel = {0},
    },

    .rx_ex_mask = 0x40,
    .rx_attr_ex = {
        .stop_check_instart = 1,
    },

    .end_flag = MIPI_CONFIG_END_FLAG,
};

static camera_config_t imx219_camera_config = {
        /* 0 */
        .name = "imx219",
        .addr = 0x10,
        .eeprom_addr = 0x51,
        .serial_addr = 0x40,
        .sensor_mode = 1,
        .fps = 30,
        .width = 1920,
        .height = 1080,
        .format = 0x12b,
        .extra_mode = 0,
        .config_index = 0,
        .mipi_cfg = &imx219_mipi_config, // MIPI配置,NULL自动获取
        .end_flag = CAMERA_CONFIG_END_FLAG,
        .calib_lname = "lib_imx219_linear.so",
};

static isp_cfg_t imx219_isp_config = {
    .isp_attr = {
        .channel = {
            .hw_id = 1,
            .slot_id = 4,
            .ctx_id = -1, //#define AUTO_ALLOC_ID -1
        },
        .work_mode = 0,
        .hdr_mode = 1,
        .size = {
            .width = 1920,
            .height = 1080,
        },
        .frame_rate = 30,
        .sched_mode = 1,
        .algo_state = 1,
        .isp_combine = {
            .isp_channel_mode = 0, //ISP_CHANNEL_MODE_NORMAL
            .bind_channel = {
                .bind_hw_id = 0,
                .bind_slot_id = 0,
            },
        },
        .clear_record = 0, //json和代码中未拿到，设置为0
        .isp_sw_ctrl = {
            .ae_stat_buf_en = 1,
            .awb_stat_buf_en = 1,
            .ae5bin_stat_buf_en = 1,
            .ctx_buf_en = 0,
            .pixel_consistency_en = 0,
        },
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
    .ochn_attr = {
        .output_crop_cfg = {
            .enable = 0,
            .rect = {
                .x = 0,
                .y = 0,
                .width = 0,
                .height = 0,
            },
        },
        .out_buf_noinvalid = 1,
        .out_buf_noncached = 0,
        .output_raw_level = 0, //ISP_OUTPUT_RAW_LEVEL_SENSOR_DATA
        .stream_output_mode = 1, //convert_isp_stream_output(1),
        .axi_output_mode = 9, //convert_isp_axi_output(0),
        .buf_num = 3,
    }
};

static vin_attr_t imx219_vin_attr = {
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
        .magicNumber = MAGIC_NUMBER,
    },

    .vin_ichn_attr = {
        .width =  1920,
        .height = 1080,
        .format = 43,
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
                .format = 43,
                .wstride = 0,
                .pack_mode = 1,
            },
            .pingpong_ring = 1,
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

struct ynr_init_attr imx219_ynr_attr = {
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


pym_cfg_t pym_common_config = {
        .hw_id = 1,
        .pym_mode = 1,
        .slot_id = 4,
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
            .src_in_width = 1920,
            .src_in_height = 1080,
            .src_in_stride_y = 1920,
            .src_in_stride_uv = 1920,
            .suffix_hb_val = 100,
            .prefix_hb_val = 2,
            .suffix_vb_val = 10,
            .prefix_vb_val = 0,
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
                    .region_width = 1920,
                    .region_height = 1080,
                    .wstride_uv = 1920,
                    .wstride_y = 1920,
                    .out_width = 1920,
                    .out_height = 1080,
                    .vstride = 1080, //.out_height,
                },
            },
        },
    .magicNumber = MAGIC_NUMBER,
};

vp_sensor_config_t imx219_linear_1920x1080_raw10_30fps_2lane = {
	.chip_id_reg = 0x0000,
	.chip_id = 0x0219,
	.sensor_i2c_addr_list = {0x10},
	.sensor_name = "imx219-30fps",
	.config_file = "linear_1920x1080_raw10_30fps_2lane.c",
	.camera_config = &imx219_camera_config,
	.vin_attr = &imx219_vin_attr,
	.isp_cfg      = &imx219_isp_config,
    .ynr_attr      = &imx219_ynr_attr,
	.pym_cfg = &pym_common_config,
};
