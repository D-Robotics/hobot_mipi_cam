#include "vp_sensors.h"

#define SENSOR_FPS 30
#define SENSOR_WIDTH 1920
#define SENSOR_HEIGHT 1536
#define MAGIC_NUMBER 0x12345678

/* ============= Camera config ============= */
/* base addr，代码会自动执行 addr += (1 + link_port) */
static camera_config_t gsml_shw3h_camera_config = {
    .name = "shw3h_shf3l_std",
    .addr = 0x10,        /* link_port=2 → 0x13 */
    .eeprom_addr = 0x50, /* link_port=2 → 0x53 */
    .serial_addr = 0x40, /* link_port=2 → 0x43 */
    .sensor_mode = 0x5,
    .fps = SENSOR_FPS,
    .width = SENSOR_WIDTH,
    .height = SENSOR_HEIGHT,
    .extra_mode = 0,
    .config_index = 512,
    /* mipi_cfg 不设，自动获取 */
    .calib_lname = "disable",
    .end_flag = CAMERA_CONFIG_END_FLAG,
};

/* ============= POC config ============= */
static poc_config_t gsml_shw3h_poc_cfg[] = {
    {
        .addr = 0x28,
        .poc_map = 0x1320,
        .end_flag = POC_CONFIG_END_FLAG,
    },
};

/* ============= Deserial config（传感器级，非全局） ============= */
static deserial_config_t gsml_shw3h_deserial_config = {
    .name = "max96712",
    .link_desp[0] = "shw3h_shf3l_std:0@512",
    .link_desp[1] = "shw3h_shf3l_std:0@512",
    .link_desp[2] = "shw3h_shf3l_std:0@512",
    .link_desp[3] = "shw3h_shf3l_std:0@512",
    .addr = 0x29,
    .poc_cfg = &gsml_shw3h_poc_cfg[0],
    .end_flag = DESERIAL_CONFIG_END_FLAG,
};

/* ============= 组合式 VIN attr ============= */
static vin_attr_t gsml_shw3h_vin_attr = {
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
            .vc_index = 0, /* 代码动态设为 link_port */
            .ipi_channels = 1,
            .y_uv_swap = 0,
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
        .width = SENSOR_WIDTH,
        .height = SENSOR_HEIGHT,
        .format = 30,
    },

    .vin_attr_ex = {
        .cim_static_attr = {
            .water_level_mark = 0,
        },
    },

    .vin_ochn_attr = {
        [VIN_MAIN_FRAME] = {
            .ddr_en = 1,
            .vin_basic_attr = {
                .format = 30,
                .wstride = 0,
                .pack_mode = 1,
            },
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
        [VIN_MAIN_FRAME] = {
            .buffers_num = 6,
        },
    },

    .magicNumber = MAGIC_NUMBER,
};

/* ============= PYM config ============= */
static pym_cfg_t gsml_shw3h_pym_config = {
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
                .vstride = SENSOR_HEIGHT,
            },
        },
    },
    .magicNumber = MAGIC_NUMBER,
};

/* ============= 主入口 ============= */
vp_sensor_config_t gsml_shw3h_linear_1920x1536_30fps_1lane = {
    .chip_id_reg = 0,
    .chip_id = 0x0231,
    .sensor_i2c_addr_list = {0x10},
    .sensor_type = SENSOR_TYPE_GMSL_YUV,
    .sensor_name = "shw3h_shf3l_std-30fps",
    .config_file = "gsml_shw3h_linear_1920x1536_30fps_1lane.c",
    .camera_config = &gsml_shw3h_camera_config,
    .camera_slave_config = NULL,
    .vin_attr = &gsml_shw3h_vin_attr,
    .isp_cfg = NULL,
    .ynr_attr = NULL,
    .pym_cfg = &gsml_shw3h_pym_config,
    .deserial_attr = &gsml_shw3h_deserial_config,
    .deserial_slave_attr = NULL,
};
