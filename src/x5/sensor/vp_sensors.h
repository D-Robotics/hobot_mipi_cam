#ifndef __VP_SENSORS_H__
#define __VP_SENSORS_H__

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "vin_cfg.h"
#include "isp_cfg.h"
#include "hb_camera_data_config.h"
#include "cam_def.h"

// Todo: remove define variable
#define MAGIC_NUMBER 0x12345678
#define AUTO_ALLOC_ID -1

#define VP_MAX_BUF_SIZE 256
#define VP_MAX_VCON_NUM 4

typedef struct vcon_properties {
	char device_path[VP_MAX_BUF_SIZE];
	char compatible[VP_MAX_BUF_SIZE];
	int32_t type;
	int32_t bus;
	int32_t rx_phy[2];
	char status[VP_MAX_BUF_SIZE];
	char pinctrl_names[VP_MAX_BUF_SIZE];
	int32_t pinctrl_0[8];
	int32_t gpio_oth[8];
} vcon_propertie_t;

typedef struct vp_sensor_config_s {
	int16_t chip_id_reg;
	int16_t chip_id;
	// Some sensors use a different set of i2c addresses
	uint32_t sensor_i2c_addr_list[8];
	char sensor_name[128];
	char config_file[128];
	camera_config_t *camera_config;
	vin_node_attr_t *vin_node_attr;
	vin_ichn_attr_t *vin_ichn_attr;
	vin_ochn_attr_t *vin_ochn_attr;
	isp_attr_t      *isp_attr;
	isp_ichn_attr_t *isp_ichn_attr;
	isp_ochn_attr_t *isp_ochn_attr;
} vp_sensor_config_t;

extern vp_sensor_config_t *vp_sensor_config_list[];

uint32_t vp_get_sensors_list_number();
void vp_show_sensors_list();
vp_sensor_config_t *vp_get_sensor_config_by_name(char *sensor_name);
int32_t vp_sensor_detect(char *sensor_list, int32_t *num_sensors);
int32_t vp_sensor_fixed_mipi_host(vp_sensor_config_t *sensor_config);
int32_t vp_sensor_fixed_mipi_host_1(int host, vp_sensor_config_t *sensor_config);
int copy_config(vp_sensor_config_t* dest, vp_sensor_config_t* src);
#ifdef __cplusplus
}
#endif
#endif // __VP_SENSORS_H__