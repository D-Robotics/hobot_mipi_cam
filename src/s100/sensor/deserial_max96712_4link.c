#include "vp_sensors.h"

static poc_config_t g_poc_cfg[] = {
	[0] = {
		/* 0 */
		.addr = 0xff,
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

static deserial_config_t gsml_deserial_config = {
    .name = "max96712",
    .link_desp[0] = "",
    .link_desp[1] = "",
    .link_desp[2] = "",
    .link_desp[3] = "",
    .gpio_mfp[CAMERA_DES_GPIO_TRIG0] = 0x5,
    .gpio_mfp[CAMERA_DES_GPIO_TRIG1] = 0x5,
    .gpio_mfp[CAMERA_DES_GPIO_TRIG2] = 0x5,
    .gpio_mfp[CAMERA_DES_GPIO_TRIG3] = 0x5,
    .addr = 0x29,
    .poc_cfg = &g_poc_cfg[0],
    .end_flag = DESERIAL_CONFIG_END_FLAG,
};

static deserial_config_t gsml_deserial_config_slave = {
    .name = "max96712",
    .link_desp[0] = "ov02b10std:0@512",
    .link_desp[1] = "",
    .link_desp[2] = "ov02b10std:0@512",
    .link_desp[3] = "",
    .gpio_mfp[CAMERA_DES_GPIO_TRIG0] = 0x5,
    .gpio_mfp[CAMERA_DES_GPIO_TRIG1] = 0x5,
    .gpio_mfp[CAMERA_DES_GPIO_TRIG2] = 0x5,
    .gpio_mfp[CAMERA_DES_GPIO_TRIG3] = 0x5,
    .addr = 0x29,
    .poc_cfg = &g_poc_cfg[1],
    .end_flag = DESERIAL_CONFIG_END_FLAG,        
};

vp_deserial_config_t deserial_max96712_4link = {
	.chip_id_reg = 0,
	.chip_id = 0x0820,
	.sensor_i2c_addr_list = {0x29},
	.sensor_name = "max96712",
	.config_file = "deserial_max96712_4link.c",
    .deserial_attr = &gsml_deserial_config,
    .deserial_slave_attr = &gsml_deserial_config_slave,
};

vp_deserial_config_t deserial_max96712_4link_slave = {
	.chip_id_reg = 0,
	.chip_id = 0x0820,
	.sensor_i2c_addr_list = {0x29},
	.sensor_name = "max96712_slave",
	.config_file = "deserial_max96712_4link.c",
    .deserial_attr = &gsml_deserial_config_slave,
    .deserial_slave_attr = &gsml_deserial_config_slave,
};
