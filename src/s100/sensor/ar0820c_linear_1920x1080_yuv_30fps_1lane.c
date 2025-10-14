#include "vp_sensors.h"
// #include "vp_pym.h"

#define SENSOR_FPS 30

#define SENSOR_WIDTH 1920
#define SENSOR_HEIGHT 1080


static camera_config_t ar0820std_camera_config = {
		/* 0 */
		.name = "ar0820std",
		.addr = 0x13,
		.eeprom_addr = 0x53,
		.serial_addr = 0x43,
		.sensor_mode = 0x05,
		.fps = 30,
		.width = 1920,
		.height = 1080,
		.extra_mode = 5,
		.config_index = 512,
		// .mipi_cfg = &ar0820std_mipi_config, // MIPI配置,NULL自动获取
		.end_flag = CAMERA_CONFIG_END_FLAG,
		.calib_lname = "disable",
};

static poc_config_t g_poc_cfg[] = {
	{
		/* 0 */
		.addr = 0x28,
		.poc_map = 0x1320,
		.end_flag = POC_CONFIG_END_FLAG,
	},
};

static deserial_config_t ar0820std_deserial_config = {
	.name = "max96712",
	.link_desp[0] = "ar0820std:5@512",
	.link_desp[1] = "ar0820std:5@512",
	.link_desp[2] = "ar0820std:5@512",
	.link_desp[3] = "ar0820std:5@512",
	.addr = 0x29,
	.poc_cfg = &g_poc_cfg[0],
	.end_flag = DESERIAL_CONFIG_END_FLAG,
};

static vin_attr_t ar0820std_vin_attr = {
    .vin_node_attr = {
        .vcon_attr = {
            .bus_main = 3,
            .bus_second = 3,
        },

        .cim_attr = {
            .mipi_en = 1,
            .cim_isp_flyby = 0,
            .cim_pym_flyby = 0,
            .mipi_rx = 4,
            .vc_index = 3,
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
        .format = 30,
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
                .format = 30,
                .wstride = 0,
                .vstride = 0,
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


static pym_cfg_t pym_common_config = {
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

vp_sensor_config_t ar0820std_linear_1920x1080_yuv_30fps_1lane = {
	.chip_id_reg = 0,
	.chip_id = 0x0820,
	.sensor_i2c_addr_list = {0x10},
    .sensor_type = SENSOR_TYPE_GMSL_YUV,
	.sensor_name = "ar0820std-1080p30",
	.config_file = "linear_1920x1080_yuv_30fps_1lane.c",
	.camera_config = &ar0820std_camera_config,
    .deserial_attr = &ar0820std_deserial_config,
	.vin_attr = &ar0820std_vin_attr,
	.isp_cfg      = NULL,
    .ynr_attr      = NULL,
	.pym_cfg = &pym_common_config,
};
