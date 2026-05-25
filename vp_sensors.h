#ifndef __VP_SENSORS_H__
#define __VP_SENSORS_H__

#include <string.h>
#include "cJSON.h"
#include "vin_cfg.h"
#include "isp_cfg.h"
#include "ynr_cfg.h"
#include "hbn_pym_cfg.h"
#include "hb_camera_data_config.h"

#ifdef __cplusplus
extern "C" {
#endif

//TODO: hbre 的hb_camera_data_config.h 中定义
enum sensor_mode_e {
        NORMAL_M = 1,
        DOL2_M = 2,
        DOL3_M = 3,
        DOL4_M = 4,
        PWL_M = 5,
        SLAVE_M = 6,
        MONO_M = 7,
        INVALID_MOD,
};

#define AUTO_ALLOC_ID -1

#define VP_MAX_BUF_SIZE 256
#define VP_MAX_VCON_NUM 6

#define SENSOR_TYPE_NORMAL	0
#define SENSOR_TYPE_GMSL_RAW	1
#define SENSOR_TYPE_GMSL_YUV	2
#define SENSOR_TYPE_GMSL_RGBIR	3

#define SENSOR_TYPE_NORMAL	0

#define SENSOR_DATA_TYPE_RAW12 0x2C
#define SENSOR_DATA_TYPE_RAW10 0x2B
#define SENSOR_DATA_TYPE_YUV422 0x1E

#define MAGIC_NUMBER 0x12345678

typedef struct vp_csi_config_s{
	int mipi_rx;
	int sensor_addr;
	int mclk_is_not_configed;
} vp_csi_config_t;

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
	vin_attr_ex_t   *vin_attr_ex;
	isp_attr_t      *isp_attr;
	isp_ichn_attr_t *isp_ichn_attr;
	isp_ochn_attr_t *isp_ochn_attr;
	struct ynr_init_attr *ynr_attr;
	pym_cfg_t *pym_config;
	deserial_config_t *deserial_node_attr;
	mipi_config_t *mipi_cfg_attr;
	uint16_t sensor_type;
} vp_sensor_config_t;

extern vp_sensor_config_t *vp_sensor_config_list[];

void vp_show_sensors_list();
uint32_t vp_get_sensors_list_number();
vp_sensor_config_t *vp_get_sensor_config_by_name(char *sensor_name);

int32_t vp_sensor_multi_fixed_mipi_host(const vp_sensor_config_t *sensor_config, int *used_mipi_host, vp_csi_config_t* csi_config);

void vp_deserial_config_show();
const deserial_config_t* vp_deserial_config_get();
void vp_deserial_config_update(const camera_config_t *camera_config, int link_port);
void vp_update_camera_config(const camera_config_t *camera_config_in, camera_config_t *camera_config_out, int link_port);
vp_sensor_config_t *vp_get_sensor_config_by_mipi_host(int32_t mipi_host_index,
	vp_csi_config_t* csi_config,int sensor_height,int sensor_width,int sensor_fps);
#define VP_MAX_VCON_NUM 6

typedef struct {
	int index;
	int is_valid;
	int mclk_is_not_configed;
	int sensor_addr;
	char sensor_config_list[128];
} csi_info_t;
//保证 0-3 的信息分别存储到 csi_info中，即使这个CSI下没有摄像头
typedef struct{
	int valid_count;
	int max_count;
	csi_info_t csi_info[VP_MAX_VCON_NUM];
} csi_list_info_t;
void vp_sensor_detect_structed(csi_list_info_t *csi_list_info);
#ifdef __cplusplus
}
#endif
#endif // __VP_SENSORS_H__
