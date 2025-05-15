#include "vp_sensors.h"

#define SENSOR_WIDTH  1088
#define SENSOR_HEIGHT  1280
#define SENSOE_FPS 30
#define RAW10 0x2B

// sc132gs 的sample配置
static mipi_config_t sc132gs_mipi_config = {
    .rx_enable = 1,
    .rx_attr = {
        .phy = 0,
        .lane = 1,
        .datatype = RAW10,
        .fps = SENSOE_FPS,
        .mclk = 1,
        .mipiclk = 1200,
        .width = 0,
        .height = 0,
        .linelenth = 0,
        .framelenth = 0,
        .settle = 0,
        .channel_num = 0,
        .channel_sel = {0},
    },

    .rx_ex_mask = 0x1,
    .rx_attr_ex = {
        .nocheck = 1,
    },

    .end_flag = MIPI_CONFIG_END_FLAG,
};

static camera_config_t sc132gs_camera_config = {
        /* 0 */
        .name = "sc132gs",
        .addr = 0x33,
        .eeprom_addr = 0x51,
        .serial_addr = 0x40,
        .sensor_mode = 1,
        .fps = SENSOE_FPS,
        .width = SENSOR_WIDTH,
        .height = SENSOR_HEIGHT,
        .extra_mode = 0,
        .config_index = 0,
        .mipi_cfg = &sc132gs_mipi_config, // MIPI配置,NULL自动获取
        .end_flag = CAMERA_CONFIG_END_FLAG,
        .calib_lname = "lib_sc132gs_linear.so",
};

static isp_cfg_t sc132gs_isp_config = {
    .isp_attr = {
        .channel = {
            .hw_id = 1,
            .slot_id = 4,
            .ctx_id = -1, //#define AUTO_ALLOC_ID -1
        },
        .work_mode = 0,
        .hdr_mode = 0,
        .size = {
            .width = SENSOR_WIDTH,
            .height = SENSOR_HEIGHT,
        },
        .frame_rate = 30,
        .sched_mode = 1,
        .algo_state = 1,
        .isp_combine = {
            .isp_channel_mode = 0, //ISP_CHANNEL_MODE_NORMAL
            .bind_channel = {
                .bind_hw_id = 1,
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

static vin_attr_t sc132gs_vin_attr = {
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
        .width =  SENSOR_WIDTH,
        .height = SENSOR_HEIGHT,
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
                .wstride = 1376,
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

ynr_info_t sc132gs_ynr_config = {
        /* 0 */
        .hw_id = 1,
        .link_mode = 1,
        .slot_id = 4,
        .ch_img_width = SENSOR_WIDTH,
        .ch_img_height = SENSOR_HEIGHT,
        .nr2d_en = 1,
        .nr3d_en = 1,
        .debug_en = 0,
};

pym_cfg_t sc132gs_pym_common_config = {
        .hw_id = 1,
        .pym_mode = 3,
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

vp_sensor_config_t sc132gs_linear_1088x1280_raw10_30fps_1lane = {
	.chip_id_reg = 0x3107,
	.chip_id = 0x0132,
	.sensor_i2c_addr_list = {0x30, 0x32, 0x33},
	.sensor_name = "sc132gs",
	.config_file = "linear_1088x1280_raw10_30fps_1lane.c",
	.camera_config = &sc132gs_camera_config,
	.vin_attr = &sc132gs_vin_attr,
	.isp_cfg      = &sc132gs_isp_config,
    .ynr_cfg      = &sc132gs_ynr_config,
	.pym_cfg = &sc132gs_pym_common_config,
};
