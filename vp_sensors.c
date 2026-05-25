#include "vp_sensors.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <ctype.h>


#include <stdbool.h>

extern vp_sensor_config_t imx219_linear_1920x1080_raw10_30fps_1lane;
extern vp_sensor_config_t sc1336_gmsl_linear_1280x720_raw10_30fps_2lane;
extern vp_sensor_config_t ar0820std_linear_3840x2160_30fps_1lane;
extern vp_sensor_config_t ar0820std_linear_1920x1080_yuv_30fps_1lane;
extern vp_sensor_config_t ovx3cstd_linear_1920x1280_raw12_30fps_1lane;
extern vp_sensor_config_t dummy_sensor_config;
extern vp_sensor_config_t sc230ai_linear_1920x1080_raw10_30fps_1lane;
extern vp_sensor_config_t sc132gs_linear_1088x1280_raw10_30fps_2lane;
extern vp_sensor_config_t s36_linear_1920x1536_30fps_4lane_vc0;
extern vp_sensor_config_t s36_linear_1920x1536_30fps_4lane_vc1;
extern vp_sensor_config_t dummystd_linear_1920x1536_30fps_4lane_vc1;
vp_sensor_config_t *vp_sensor_config_list[] = {
    &imx219_linear_1920x1080_raw10_30fps_1lane,
	&sc1336_gmsl_linear_1280x720_raw10_30fps_2lane,
	&ar0820std_linear_3840x2160_30fps_1lane,
	&ar0820std_linear_1920x1080_yuv_30fps_1lane,
	&ovx3cstd_linear_1920x1280_raw12_30fps_1lane,
	&dummy_sensor_config,
	&sc230ai_linear_1920x1080_raw10_30fps_1lane,
	&sc132gs_linear_1088x1280_raw10_30fps_2lane,
	&s36_linear_1920x1536_30fps_4lane_vc0,
	&s36_linear_1920x1536_30fps_4lane_vc1,
	&dummystd_linear_1920x1536_30fps_4lane_vc1,
};

/////////////////////////////////////////////////////////////////////////////////////////////////////
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

typedef struct mipi_properties {
	char device_path[VP_MAX_BUF_SIZE];
	char status[VP_MAX_BUF_SIZE];
	char pinctrl_names[VP_MAX_BUF_SIZE];
	int32_t pinctrl_0[8];
	int32_t pinctrl_1[8];
	int32_t snrclk_idx[8];
} mipi_propertie_t;

// Check system endianness
static int is_little_endian() {
	uint16_t test = 0x0001;
	return *((uint8_t*)(&test)) == 0x01;
}

// Endianness conversion for 32-bit integer
static int32_t convert_endianness_int32(int32_t value) {
	if (is_little_endian()) {
		// Convert from little endian to big endian
		return ((value >> 24) & 0x000000FF)
			| ((value >> 8) & 0x0000FF00)
			| ((value << 8) & 0x00FF0000)
			| ((value << 24) & 0xFF000000);
	} else {
		// Convert from big endian to little endian
		return value;
	}
}

#define GPIO_EXPORT_PATH "/sys/class/gpio/export"
#define GPIO_UNEXPORT_PATH "/sys/class/gpio/unexport"

// Function to export a GPIO
static int gpio_export(int gpio_number) {
	FILE *fp;
	fp = fopen(GPIO_EXPORT_PATH, "w");
	if (fp == NULL) {
		printf("Error opening GPIO export file for writing\n");
		return -1;
	}
	fprintf(fp, "%d", gpio_number);
	fclose(fp);
	return 0;
}

// Function to unexport a GPIO
static int gpio_unexport(int gpio_number) {
	FILE *fp;
	fp = fopen(GPIO_UNEXPORT_PATH, "w");
	if (fp == NULL) {
		printf("Error opening GPIO unexport file for writing\n");
		return -1;
	}
	fprintf(fp, "%d", gpio_number);
	fclose(fp);
	return 0;
}

// Function to set GPIO direction
static int gpio_set_direction(int gpio_number, const char *direction) {
	char filename[256];
	FILE *fp;
	snprintf(filename, sizeof(filename), "/sys/class/gpio/gpio%d/direction", gpio_number);
	fp = fopen(filename, "w");
	if (fp == NULL) {
		printf("Error opening GPIO direction file for writing\n");
		return -1;
	}
	fprintf(fp, "%s", direction);
	fclose(fp);
	return 0;
}

// Function to set GPIO value
static int gpio_set_value(int gpio_number, int value) {
	char filename[256];
	FILE *fp;
	snprintf(filename, sizeof(filename), "/sys/class/gpio/gpio%d/value", gpio_number);
	fp = fopen(filename, "w");
	if (fp == NULL) {
		printf("Error opening GPIO value file for writing\n");
		return -1;
	}
	fprintf(fp, "%d", value);
	fclose(fp);
	return 0;
}

static int enable_sensor_pin(int gpio_number, int active)
{
	// Export the GPIO
	if (gpio_export(gpio_number) != 0) {
		printf("Failed to export GPIO\n");
		return -1;
	}

	usleep(30 * 1000);

	// Set GPIO direction to output
	if (gpio_set_direction(gpio_number, "out") != 0) {
		printf("Failed to set GPIO direction\n");
		return -1;
	}

	usleep(30 * 1000);

	/* gpio level should be keep same with sensor driver power_on api */
	// Set GPIO value to active
	if (gpio_set_value(gpio_number, active) != 0) {
		printf("Failed to set GPIO value\n");
		return -1;
	}

	usleep(30 * 1000);

	// Set GPIO value to 1 - active
	if (gpio_set_value(gpio_number,  (1 - active)) != 0) {
		printf("Failed to set GPIO value\n");
		return -1;
	}

	usleep(30 * 1000);

	// Set GPIO value to active
	if (gpio_set_value(gpio_number, active) != 0) {
		printf("Failed to set GPIO value\n");
		return -1;
	}

	usleep(30 * 1000);

	// Unexport the GPIO
	if (gpio_unexport(gpio_number) != 0) {
		printf("Failed to unexport GPIO\n");
		return -1;
	}

	return 0;
}
//9
static void read_mipi_info_from_device_tree(const int device, struct mipi_properties *properties) {
	#define MIPI_DEVICE_COUNT 6
	const char *mipi_device_tree_node_suffixs [MIPI_DEVICE_COUNT]= {"0x37420000", "0x37620000","0x37820000","0x37A20000","0x37C20000","0x37C40000"};
	if(device > MIPI_DEVICE_COUNT){
		printf("Error device %d exceed max valud %d\n", device, MIPI_DEVICE_COUNT);
		return;
	}
	const char *node_suffix = mipi_device_tree_node_suffixs[device];
	memset(properties, 0, sizeof(struct mipi_properties));

	snprintf(properties->device_path, sizeof(properties->device_path),
		"/proc/device-tree/soc/mipi_host@%s", node_suffix);

	DIR *dir = opendir(properties->device_path);
	if (dir == NULL) {
		printf("Error opening directory: %s\n", properties->device_path);
		return;
	}
	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_type == DT_REG) { // Regular file
			char filename[VP_MAX_BUF_SIZE] = {0};
			int ret = snprintf(filename, sizeof(filename), "%s/%s", properties->device_path, entry->d_name);
			if (ret < 0 || ret >= sizeof(filename)) {
				printf("Error: Failed to set filename\n");
				return;
			}
			FILE *fp = fopen(filename, "rb");
			if (fp != NULL) {
				if (strcmp(entry->d_name, "status") == 0) {
					fread(&properties->status, sizeof(char), VP_MAX_BUF_SIZE, fp);
				}  else if (strcmp(entry->d_name, "pinctrl-names") == 0) {
					fread(&properties->pinctrl_names, sizeof(char), VP_MAX_BUF_SIZE, fp);
				}  else if (strcmp(entry->d_name, "pinctrl-0") == 0) {
					fread(&properties->pinctrl_0, sizeof(int32_t), sizeof(properties->pinctrl_0) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->pinctrl_0) / sizeof(int32_t); ++i)
						properties->pinctrl_0[i] = convert_endianness_int32(properties->pinctrl_0[i]);
				} else if (strcmp(entry->d_name, "pinctrl-1") == 0) {
					fread(&properties->pinctrl_1, sizeof(int32_t), sizeof(properties->pinctrl_1) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->pinctrl_1) / sizeof(int32_t); ++i)
						properties->pinctrl_1[i] = convert_endianness_int32(properties->pinctrl_1[i]);
				} else if (strcmp(entry->d_name, "snrclk-idx") == 0) {
					fread(&properties->snrclk_idx, sizeof(int32_t), sizeof(properties->snrclk_idx) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->snrclk_idx) / sizeof(int32_t); ++i)
						properties->snrclk_idx[i] = convert_endianness_int32(properties->snrclk_idx[i]);
				}

				// Close the file
				fclose(fp);
			}
		}
	}

	closedir(dir);
}
static void read_vcon_info_from_device_tree(const int device, struct vcon_properties *properties) {
	memset(properties, 0, sizeof(struct vcon_properties));

	snprintf(properties->device_path, sizeof(properties->device_path),
		"/proc/device-tree/soc/vcon@%d", device);

	DIR *dir = opendir(properties->device_path);
	if (dir == NULL) {
		printf("Error opening directory: %s\n", properties->device_path);
		return;
	}

	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_type == DT_REG) { // Regular file
			char filename[VP_MAX_BUF_SIZE] = {0};
			int ret = snprintf(filename, sizeof(filename), "%s/%s", properties->device_path, entry->d_name);
			if (ret < 0 || ret >= sizeof(filename)) {
				printf("Error: Failed to set filename\n");
				return;
			}

			FILE *fp = fopen(filename, "rb");
			if (fp != NULL) {
				if (strcmp(entry->d_name, "compatible") == 0) {
					fread(&properties->compatible, sizeof(char), VP_MAX_BUF_SIZE, fp);
				} else if (strcmp(entry->d_name, "status") == 0) {
					fread(&properties->status, sizeof(char), VP_MAX_BUF_SIZE, fp);
				}  else if (strcmp(entry->d_name, "pinctrl-names") == 0) {
					fread(&properties->pinctrl_names, sizeof(char), VP_MAX_BUF_SIZE, fp);
				} else if (strcmp(entry->d_name, "type") == 0) {
					fread(&properties->type, sizeof(int32_t), 1, fp);
					properties->type = convert_endianness_int32(properties->type);
				} else if (strcmp(entry->d_name, "bus") == 0) {
					fread(&properties->bus, sizeof(int32_t), 1, fp);
					properties->bus = convert_endianness_int32(properties->bus);
				} else if (strcmp(entry->d_name, "rx_phy") == 0) {
					fread(&properties->rx_phy, sizeof(int32_t), sizeof(properties->rx_phy) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->rx_phy) / sizeof(int32_t); ++i)
						properties->rx_phy[i] = convert_endianness_int32(properties->rx_phy[i]);
				} else if (strcmp(entry->d_name, "pinctrl-0") == 0) {
					fread(&properties->pinctrl_0, sizeof(int32_t), sizeof(properties->pinctrl_0) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->pinctrl_0) / sizeof(int32_t); ++i)
						properties->pinctrl_0[i] = convert_endianness_int32(properties->pinctrl_0[i]);
				} else if (strcmp(entry->d_name, "gpio_oth") == 0) {
					fread(&properties->gpio_oth, sizeof(int32_t), sizeof(properties->gpio_oth) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->gpio_oth) / sizeof(int32_t); ++i)
						properties->gpio_oth[i] = convert_endianness_int32(properties->gpio_oth[i]);
				}

				// Close the file
				fclose(fp);
			}
		}
	}

	closedir(dir);
}

static int32_t vp_i2c_read_reg16_data8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint8_t *value)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[2] = {0};
	uint8_t readbuf[1] = {0};
	struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};
	char filename[20];
	int file;

	// Open the I2C bus
	snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
	file = open(filename, O_RDWR);
	if (file < 0) {
		fprintf(stderr, "Open I2C bus %s failed:%s\n", filename, strerror(errno));
		return -1;
	}

	sendbuf[0] = (uint8_t)((reg_addr >> 8u) & 0xffu);
	sendbuf[1] = (uint8_t)(reg_addr & 0xffu);

	data.msgs = msgs; /*PRQA S 5118*/
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	data.msgs[1].len = 1;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 1;
	data.msgs[1].buf = readbuf;

	ret = ioctl(file, I2C_RDWR, (uint64_t)&data);
	if (ret < 0) {
		// perror("Failed to read from the I2C bus");
		*value = 0;
		close(file);
		return -1;
	}
	*value = readbuf[0];

	// Close the I2C bus
	close(file);

	return 0;
}


// Function to write frequency to MIPI host
static void write_mipi_host_freq(int mipi_host, int freq)
{
	char path[256];
	FILE *file;

	// Construct path to the file
	snprintf(path, 256, "/sys/class/vps/mipi_host%d/param/snrclk_freq", mipi_host);

	// Open the file for writing
	file = fopen(path, "w");
	if (file) {
		// Write frequency to the file
		fprintf(file, "%d", freq);
		fclose(file);
	}
}

// Function to enable MIPI host clock
static void enable_mipi_host_clock(int mipi_host, int enable)
{
	char path[256];
	FILE *file;

	// Construct path to the file
	snprintf(path, 256, "/sys/class/vps/mipi_host%d/param/snrclk_en", mipi_host);

	// Open the file for writing
	file = fopen(path, "w");
	if (file) {
		// Write enable value to the file
		fprintf(file, "%d", enable);
		fclose(file);
	}
}

static int check_mipi_host_status(int mipi_host) {
	char file_path[100];
	snprintf(file_path, sizeof(file_path), "/sys/class/vps/mipi_host%d/status/cfg", mipi_host);

	FILE *file = fopen(file_path, "r");
	if (file == NULL) {
		printf("Failed to open %s: %s\n", file_path, strerror(errno));
		return 0;
	}

	char first_line[256];
	if (fgets(first_line, sizeof(first_line), file) == NULL) {
		perror("Failed to read file");
		fclose(file);
		return 0;
	}

	fclose(file);

	first_line[strcspn(first_line, "\n")] = '\0';

	// 判断第一行内容是否为 "not inited"
	if (strcmp(first_line, "not inited") == 0) {
		return 1;
	} else {
		return 0;
	}
}
#if 0
static int get_board_id(char *data, size_t size)
{
	const char *board_id_file = "/sys/class/socinfo/board_id";
	FILE *fp = fopen(board_id_file, "r");
	if (fp == NULL) {
		printf("[ERROR] open file %s failed.\n", board_id_file);
		return -1;
	}

	if (fgets(data, size, fp) == NULL) {
		printf("[ERROR] read file %s failed.\n", board_id_file);
		fclose(fp);
		return -1;
	}
	fclose(fp);

	// Remove trailing newline
	size_t len = strlen(data);
	if (len > 0 && data[len - 1] == '\n') {
		data[len - 1] = '\0';
	}

	// Trim leading and trailing whitespace
	char *start = data;
	while (isspace((unsigned char)*start)) {
		start++;
	}

	char *end = data + strlen(data) - 1;
	while (end > start && isspace((unsigned char)*end)) {
		end--;
	}

	// Null-terminate the trimmed string
	*(end + 1) = '\0';

	// Move the trimmed string to the start of the buffer
	if (start != data) {
		memmove(data, start, end - start + 2);
	}

	return 0;
}
#endif

static int32_t vp_sensor_mipi_host_mclk_is_not_configed(int csi_index){
	int mclk_is_not_configed = 0;
	struct mipi_properties mipi_property;

	read_mipi_info_from_device_tree(csi_index, &mipi_property);
	if(strlen(mipi_property.pinctrl_names) == 0){
			mclk_is_not_configed = 1;
			printf("mipi mclk is not configed.\n");
		}else{

			printf("mipi mclk is configed.\n");
		}
	return mclk_is_not_configed;
}

static int32_t get_valid_sensor_addr(vcon_propertie_t vcon_props,
	const vp_sensor_config_t *sensor_config) {

	// Read sensor chip ID register
	int32_t chip_id = 0;
	uint32_t addr = -1;

	// Try reading chip ID using sensor_i2c_addr_list
	for (int i = 0; i < 8; i++) {
		addr = sensor_config->sensor_i2c_addr_list[i];
		if (addr == 0)
			continue;

		if (vp_i2c_read_reg16_data8(vcon_props.bus, addr, sensor_config->chip_id_reg, (uint8_t*)&chip_id) == 0) {
			if (sensor_config->chip_id == 0xA55A || // 如果有的 sensor 本身读不到ID，但是又想要使用它，就把 sensor 的 chip_id 设为 0xA55A
				((chip_id & 0xFF) == (sensor_config->chip_id >> 8 & 0xFF)) ||
				((chip_id & 0xFF) == (sensor_config->chip_id & 0xFF))) {
				return addr;
			} else {
				printf("WARN: Sensor Name: %s, Expected Chip ID: 0x%02X, Actual Chip ID Read: 0x%02X\n",
					sensor_config->sensor_name, sensor_config->chip_id & 0x0000FFFF, chip_id);
			}
		}
	}

	// If none of the addresses worked
	// printf("Failed to read sensor chip ID register\n");
	return -1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t vp_sensor_multi_fixed_mipi_host(const vp_sensor_config_t *sensor_config,
	int *used_mipi_host, vp_csi_config_t* csi_config)
{
	int32_t ret = -1;
	uint32_t frequency = 24000000;
	struct vcon_properties vcon_props_array[VP_MAX_VCON_NUM];
	const int vcon_ids[] = {0, 1, 2, 3, 4, 5};

	// Iterate over vcon@0 - 2
	for (int i = 0; i < VP_MAX_VCON_NUM; ++i) {

		// 跳过使用使用的mipi csi控制器，支持同时接入相同的摄像头
		int mipi_index = vcon_ids[i];

		if (*used_mipi_host & (1 << mipi_index))
			continue;

		if (check_mipi_host_status(mipi_index) == 0)
			continue;

		int mclk_is_not_configed = vp_sensor_mipi_host_mclk_is_not_configed(mipi_index);
		read_vcon_info_from_device_tree(mipi_index, &vcon_props_array[i]);

		printf("Searching camera sensor on device: %s ", vcon_props_array[i].device_path);
		printf("i2c bus: %d ", vcon_props_array[i].bus);
		printf("mipi rx phy: %d\n", vcon_props_array[i].rx_phy[1]);
		printf("mipi rx used phy: %08x\n", *used_mipi_host);

		// 如果该vcon使能了，检测该vcon上是否有连接 sensor
		if (vcon_props_array[i].status[0] == 'o') { // okay
			// 检测该vcon上连接的 sensor
			/*enable gpio_oth, enable camera sensor gpio, maybe pwd/reset gpio */
			for (int j = 0; j < 8; ++j) {
				if (vcon_props_array[i].gpio_oth[j] != 0) {
					if (sensor_config->camera_config->gpio_enable != 0) {
						// gpio_level should be from sensor config and sensor spec
						enable_sensor_pin(vcon_props_array[i].gpio_oth[j],
							(1 - sensor_config->camera_config->gpio_level));
					}
				}
			}

			if(!mclk_is_not_configed){
				/* enable mclk */
				write_mipi_host_freq(mipi_index, frequency);
				enable_mipi_host_clock(mipi_index, 1);
			}

			// 从指定的vcon关联的i2c bus上读取 vp_sensor_config_list 中指定的 chip_id_reg 对应的寄存器值
			int sensor_addr = get_valid_sensor_addr(vcon_props_array[i], sensor_config);
			if (sensor_addr > 0) {
				ret = 0;
				csi_config->sensor_addr = sensor_addr;
				csi_config->mipi_rx = vcon_props_array[i].rx_phy[1];
				csi_config->mclk_is_not_configed = mclk_is_not_configed;
				*used_mipi_host |= (1 << csi_config->mipi_rx);
				printf("INFO: Found sensor_name:%s on mipi rx csi %d, i2c addr 0x%x, config_file:%s\n",
					sensor_config->sensor_name, vcon_props_array[i].rx_phy[1],
					csi_config->sensor_addr, sensor_config->config_file);
				break;
			}
		}
	}

	return ret;
}


uint32_t vp_get_sensors_list_number() {
	return sizeof(vp_sensor_config_list) / sizeof(vp_sensor_config_list[0]);
}

void vp_show_sensors_list() {
	int num = 0;

	num = vp_get_sensors_list_number();
	for (int i = 0; i < num; i++) {
		printf("index: %d  sensor_name: %-16s \tconfig_file:%s\n",
		i, vp_sensor_config_list[i]->sensor_name,
		vp_sensor_config_list[i]->config_file);
	}
}

vp_sensor_config_t *vp_get_sensor_config_by_name(char *sensor_name)
{
	for (int i = 0; strlen(vp_sensor_config_list[i]->sensor_name) > 0; i++) {
		if (strcmp(vp_sensor_config_list[i]->sensor_name, sensor_name) == 0) {
			return vp_sensor_config_list[i];
		}
	}
	return NULL;
}

static poc_config_t g_poc_cfg[] = {
	{
		/* 0 */
		.addr = 0x28,
		.poc_map = 0x0213,
		.end_flag = POC_CONFIG_END_FLAG,
	},
};
static deserial_config_t g_gmsl_deserial_config = {
	.name = "max96712",
	.link_desp[0] = "sc1336_gmsl:0@256",
	.link_desp[1] = "sc1336_gmsl:0@256",
	.link_desp[2] = "sc1336_gmsl:0@256",
	.link_desp[3] = "sc1336_gmsl:0@256",
	.addr = 0x29,
	.gpio_mfp[CAMERA_DES_GPIO_TRIG0] = 0x5,
	.poc_cfg = &g_poc_cfg[0],
	.end_flag = DESERIAL_CONFIG_END_FLAG,
};

void vp_deserial_config_update(const camera_config_t *camera_config, int link_port){
	snprintf(g_gmsl_deserial_config.link_desp[link_port],
			sizeof(g_gmsl_deserial_config.link_desp[link_port]),
			"%.32s:%d@%d",
			camera_config->name, camera_config->extra_mode, camera_config->config_index);

	if(camera_config->sensor_mode == 6){
		g_gmsl_deserial_config.gpio_mfp[link_port] = 0x5;
	}
}
void vp_update_camera_config(const camera_config_t *camera_config_in, camera_config_t *camera_config_out, int link_port){
	camera_config_out->addr = (uint8_t)(camera_config_in->addr + 1 + link_port);
	camera_config_out->serial_addr = (uint8_t)(camera_config_in->serial_addr + 1  + link_port);
	camera_config_out->eeprom_addr = (uint8_t)(camera_config_in->eeprom_addr + 1  + link_port);
}
const deserial_config_t* vp_deserial_config_get(){
	return &g_gmsl_deserial_config;
}
void vp_deserial_config_show(){
	printf("All deserial link info:\n");
	for(int i = 0; i< 4; i++){
		printf("	[link_port:%d] %s\n", i, g_gmsl_deserial_config.link_desp[i]);
	}
}


void vp_sensor_detect_structed(csi_list_info_t *csi_list_info)
{
	struct vcon_properties vcon_props_array[VP_MAX_VCON_NUM];
	struct mipi_properties mipi_props_array[VP_MAX_VCON_NUM];
	csi_list_info->valid_count = 0;
	csi_list_info->max_count = VP_MAX_VCON_NUM;

	for (int i = 0; i < VP_MAX_VCON_NUM; ++i) {
		csi_info_t csi_info_tmp = {.index = i, .is_valid = 0};
		read_vcon_info_from_device_tree(i, &vcon_props_array[i]);
		read_mipi_info_from_device_tree(i, &mipi_props_array[i]);

		int mclk_is_not_configed = 0;
		printf("\n");
		printf("Searching camera sensor on device: %s ", vcon_props_array[i].device_path);
		printf("i2c bus: %d ", vcon_props_array[i].bus);
		printf("mipi rx phy: %d\n", vcon_props_array[i].rx_phy[1]);
		if(strlen(mipi_props_array[i].pinctrl_names) == 0){
			mclk_is_not_configed = 1;
			printf("mipi mclk is not configed.\n");
		}else{
			printf("mipi mclk is configed.\n");
		}
		csi_info_tmp.mclk_is_not_configed = mclk_is_not_configed;

		memset(csi_info_tmp.sensor_config_list, 0, sizeof(csi_info_tmp.sensor_config_list));
		if (vcon_props_array[i].status[0] == 'o') {
			if(!mclk_is_not_configed){
				/* enable mclk */
				write_mipi_host_freq(i, 24000000);
				enable_mipi_host_clock(i, 1);
			}

			for (int j = 0; j < vp_get_sensors_list_number(); j++) {
				for (int k = 0; k < 8; ++k) {
					if (vcon_props_array[i].gpio_oth[k] != 0) {
						if ((vp_sensor_config_list[j]->camera_config->gpio_enable & (1 << k)) != 0) {
							enable_sensor_pin(vcon_props_array[i].gpio_oth[k],
								(1 - vp_sensor_config_list[j]->camera_config->gpio_level));
						}
					}
				}
				int sensor_addr = get_valid_sensor_addr(vcon_props_array[i], vp_sensor_config_list[j]);
				if (sensor_addr > 0) {
					printf("INFO: Support sensor name:%s on mipi rx csi %d, "
							"i2c addr 0x%x, config_file:%s\n",
						vp_sensor_config_list[j]->sensor_name,
						vcon_props_array[i].rx_phy[1],
						sensor_addr,
						vp_sensor_config_list[j]->config_file);

					csi_info_tmp.index = i;
					csi_info_tmp.is_valid = 1;
					csi_info_tmp.sensor_addr = sensor_addr;

					if (strlen(csi_info_tmp.sensor_config_list) > 1) {
						strcat(csi_info_tmp.sensor_config_list, "/");
					}
					strcat(csi_info_tmp.sensor_config_list, vp_sensor_config_list[j]->sensor_name);
				}
			}
			csi_list_info->csi_info[i] = csi_info_tmp;
			if(csi_info_tmp.is_valid){
				csi_list_info->valid_count++;
			}
		}
	}
}

// Function to check sensor register value
static int32_t check_sensor_reg_value(vcon_propertie_t vcon_props,
		vp_sensor_config_t *sensor_config) {
	int i = 0;
	// Read sensor chip ID register
	int32_t chip_id = 0;
	uint32_t addr = sensor_config->camera_config->addr;

	// Add camera_config->addr to sensor_i2c_addr_list
	for (i = 1; i < 8; i++) {
		if (sensor_config->sensor_i2c_addr_list[i] == 0) {
			sensor_config->sensor_i2c_addr_list[i] = addr;
			break;
		}
	}

	// Try reading chip ID using sensor_i2c_addr_list
	for (i = 0; i < 8; i++) {
		addr = sensor_config->sensor_i2c_addr_list[i];
		if (addr == 0)
			continue;

		if (vp_i2c_read_reg16_data8(vcon_props.bus, addr, sensor_config->chip_id_reg, (uint8_t*)&chip_id) == 0) {
			if (sensor_config->chip_id == 0xA55A || // 如果有的 sensor 本身读不到ID，但是又想要使用它，就把 sensor 的 chip_id 设为 0xA55A
				((chip_id & 0xFF) == (sensor_config->chip_id >> 8 & 0xFF)) ||
				((chip_id & 0xFF) == (sensor_config->chip_id & 0xFF))) {
					// Update sensor address to the one successfully read from
					sensor_config->camera_config->addr = addr;
					// Update mipi rx phy
					// 修正配置中的 sensor 使用的 mipi rx phy
					// 此处的修改并不一定是最终修改，在vpp 的impl 的 param 设置中会根据具体情况再次修改
					sensor_config->vin_node_attr->cim_attr.mipi_rx = vcon_props.rx_phy[1];
				return 0;
			} else {
				printf("WARN: Sensor Name: %s, Expected Chip ID: 0x%02X, Actual Chip ID Read: 0x%02X\n",
					sensor_config->sensor_name, sensor_config->chip_id & 0x0000FFFF, chip_id);
				return -1;
			}
		}
	}

	// If none of the addresses worked
	// printf("Failed to read sensor chip ID register\n");
	return -1;
}

vp_sensor_config_t *vp_get_sensor_config_by_mipi_host(int32_t mipi_host_index,
	vp_csi_config_t* csi_config,int sensor_height,int sensor_width,int sensor_fps)
{
	int32_t ret = 0, j = 0;
	uint32_t frequency = 24000000;
	//int is_need_used_csi[VP_MAX_VCON_NUM] = {true, true, true, true};
	//should_used_csi(is_need_used_csi);
	int32_t sensor_n = 0;
	int32_t sensor_list[10] = {-1};

	struct vcon_properties vcon_props_array[VP_MAX_VCON_NUM];

	if (check_mipi_host_status(mipi_host_index) == 0) {
		printf("Mipi csi%d has been used, please use other Cam interfaces\n", mipi_host_index);
		return NULL;
	}
	int mclk_is_not_configed = vp_sensor_mipi_host_mclk_is_not_configed(mipi_host_index);
	read_vcon_info_from_device_tree(mipi_host_index, &vcon_props_array[mipi_host_index]);

	if(!mclk_is_not_configed) {
		/* enable mclk */
		write_mipi_host_freq(mipi_host_index, frequency);
		enable_mipi_host_clock(mipi_host_index, 1);
	}

	csi_config->mclk_is_not_configed = mclk_is_not_configed;

	printf("Searching camera sensor on device: %s ", vcon_props_array[mipi_host_index].device_path);
	printf("i2c bus: %d ", vcon_props_array[mipi_host_index].bus);
	printf("mipi rx phy: %d\n", vcon_props_array[mipi_host_index].rx_phy[1]);

	// 如果该vcon使能了，检测该vcon上是否有连接 sensor
	if (vcon_props_array[mipi_host_index].status[0] == 'o') { // okay
		// 检测该vcon上连接的 sensor
		for (j = 0; j < vp_get_sensors_list_number(); j++) {
			/*enable gpio_oth, enable camera sensor gpio, maybe pwd/reset gpio */
			for (int k = 0; k < 8; ++k) {
				if (vcon_props_array[mipi_host_index].gpio_oth[k] != 0) {
					if ((vp_sensor_config_list[j]->camera_config->gpio_enable & (1 << k)) != 0) {
						enable_sensor_pin(vcon_props_array[mipi_host_index].gpio_oth[k],
							(1 - vp_sensor_config_list[j]->camera_config->gpio_level));
					}
				}
			}

			ret = check_sensor_reg_value(vcon_props_array[mipi_host_index], vp_sensor_config_list[j]);
			if (ret == 0) {
				if ((sensor_height == -1 || vp_sensor_config_list[j]->camera_config->height == sensor_height) &&
    			  (sensor_width == -1 || vp_sensor_config_list[j]->camera_config->width == sensor_width) &&
    			  (sensor_fps == -1 || vp_sensor_config_list[j]->camera_config->fps == sensor_fps)) {
					printf("[%d] INFO: Found sensor name:%s on mipi rx csi %d, "
						"i2c addr 0x%x, config_file:%s\n",
					sensor_n,
					vp_sensor_config_list[j]->sensor_name,
					vcon_props_array[mipi_host_index].rx_phy[1],
					vp_sensor_config_list[j]->camera_config->addr,
					vp_sensor_config_list[j]->config_file);
					if(sensor_n < 10)
					{
						sensor_list[sensor_n] = j;
						sensor_n++;
					}
					else
						break;
				}
			}
		}

		if(sensor_n > 0)
		{
			j = 0;
			if(sensor_n > 1)
			{
				printf("plase chouse sensor config,the number should small than %d\n",sensor_n);
				scanf("%d", &j);
				if (j >= sensor_n)
				{
					printf("The number is greater than sensor_n %d ,use 0\n", sensor_n);
					j = 0;
				}
			}
			/* Fixed Mipi host */
			vp_sensor_config_list[sensor_list[j]]->vin_node_attr->cim_attr.mipi_rx = vcon_props_array[mipi_host_index].rx_phy[1];
			return vp_sensor_config_list[sensor_list[j]];
		}
	}
	return NULL;
}
