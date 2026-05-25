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

extern vp_sensor_config_t imx219_linear_1920x1080_raw10_30fps_2lane;
extern vp_sensor_config_t sc132gs_linear_1088x1280_raw10_30fps_1lane;
extern vp_sensor_config_t sc230ai_linear_1920x1080_raw10_30fps_1lane;

vp_sensor_config_t *vp_sensor_config_list[] = {
	&imx219_linear_1920x1080_raw10_30fps_2lane,
	&sc132gs_linear_1088x1280_raw10_30fps_1lane,
	&sc230ai_linear_1920x1080_raw10_30fps_1lane
};

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
	for (int i = 0; vp_sensor_config_list[i]->sensor_name != NULL; i++) {
		if (strcmp(vp_sensor_config_list[i]->sensor_name, sensor_name) == 0) {
			return vp_sensor_config_list[i];
		}
	}
	return NULL;
}


extern vp_sensor_config_t ovx3cstd_linear_1920x1080_raw12_30fps_1lane;
extern vp_sensor_config_t ar0820std_linear_1920x1080_yuv_30fps_1lane;
extern vp_sensor_config_t gsml_sc132gs_linear_1280x1088_raw10_30fps_1lane;

vp_sensor_config_t *vp_gmsl_config_list[] = {
	&ovx3cstd_linear_1920x1080_raw12_30fps_1lane,
	&ar0820std_linear_1920x1080_yuv_30fps_1lane,
	&gsml_sc132gs_linear_1280x1088_raw10_30fps_1lane
};

uint32_t vp_get_gmsl_list_number() {
	return sizeof(vp_gmsl_config_list) / sizeof(vp_gmsl_config_list[0]);
}


void vp_show_gmsl_list() {
	int num = 0;

	num = vp_get_gmsl_list_number();
	for (int i = 0; i < num; i++) {
		printf("index: %d  sensor_name: %-16s \tconfig_file:%s\n",
		i, vp_gmsl_config_list[i]->sensor_name,
		vp_gmsl_config_list[i]->config_file);
	}
}

vp_sensor_config_t *vp_get_gmsl_config_by_name(char *sensor_name)
{
	for (int i = 0; vp_gmsl_config_list[i]->sensor_name != NULL; i++) {
		if (strcmp(vp_gmsl_config_list[i]->sensor_name, sensor_name) == 0) {
			return vp_gmsl_config_list[i];
		}
	}
	return NULL;
}



extern vp_deserial_config_t deserial_max96712_4link;
extern vp_deserial_config_t deserial_max96712_4link_slave;

vp_deserial_config_t *vp_deserial_config_list[] = {
	&deserial_max96712_4link,
	&deserial_max96712_4link_slave
};

uint32_t vp_get_deserial_list_number() {
	return sizeof(vp_deserial_config_list) / sizeof(vp_deserial_config_list[0]);
}


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

static int32_t vp_i2c_read_reg16_data16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint16_t *value)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[32] = {0};
	uint8_t readbuf[32] = {0};
	struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};
	char filename[20];
	int file;

	// Open the I2C bus
	snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
	file = open(filename, O_RDWR);
	if (file < 0) {
		perror("Failed to open the I2C bus");
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

	data.msgs[1].len = 2;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = I2C_M_RD;
	data.msgs[1].buf = readbuf;

	ret = ioctl(file, I2C_RDWR, (uint64_t)&data);
	if (ret < 0) {
		// perror("Failed to read from the I2C bus");
		*value = 0;
		close(file);
		return -1;
	}

	*value = (uint16_t)((readbuf[0] << 8) | readbuf[1]);

	// Close the I2C bus
	close(file);

	return 0;
}


static int32_t vp_i2c_read_reg16_data8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint8_t *value)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[32] = {0};
	uint8_t readbuf[32] = {0};
	struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};
	char filename[20];
	int file;

	// Open the I2C bus
	snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
	file = open(filename, O_RDWR);
	if (file < 0) {
		perror("Failed to open the I2C bus");
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

	data.msgs[1].len = 2;
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

static int32_t read_chip_id(vcon_propertie_t vcon_props, vp_sensor_config_t *sensor_config, uint32_t addr, int32_t *chip_id) {

	if(sensor_config->chip_id >> 8 == 0) {
		// 读取 8 位 chip ID
		if (vp_i2c_read_reg16_data8(vcon_props.bus, addr, sensor_config->chip_id_reg, (uint8_t*)chip_id) == 0) {
			if (((chip_id[0] & 0xFF) == (sensor_config->chip_id & 0xFF))) {
				return 0;
			}
		}
	} else {
		// 读取 16 位 chip ID
		if (vp_i2c_read_reg16_data16(vcon_props.bus, addr, sensor_config->chip_id_reg, (uint16_t*)chip_id) == 0) {
			if ((sensor_config->chip_id == 0xA55A) // 如果有的 sensor 本身读不到ID，但是又想要使用它，就把 sensor 的 chip_id 设为 0xA55A
			|| ((chip_id[0] & 0xFFFF) == (sensor_config->chip_id & 0xFFFF))) {
				return 0;
			}
		}
	}
	return -1;
}

// Function to check sensor register value
int32_t check_sensor_reg_value(vcon_propertie_t vcon_props,
		vp_sensor_config_t *sensor_config) {
	int i = 0;
	// Read sensor chip ID register
	int32_t chip_id = 0;
	uint32_t addr = sensor_config->camera_config->addr;

	// Add camera_config->addr to sensor_i2c_addr_list
	if (sensor_config->sensor_i2c_addr_list[0] == 0) {
		sensor_config->sensor_i2c_addr_list[0] = addr;
	}
#if 0
	for (i = 1; i < 8; i++) {
		if (sensor_config->sensor_i2c_addr_list[i] == 0) {
			sensor_config->sensor_i2c_addr_list[i] = addr;
			break;
		}
	}
#endif
	// Try reading chip ID using sensor_i2c_addr_list
	for (i = 0; i < 8; i++) {
		addr = sensor_config->sensor_i2c_addr_list[i];
		if (addr == 0)
			continue;
		if (read_chip_id(vcon_props, sensor_config, addr, &chip_id) == 0) {
			// Update sensor address to the one successfully read from
			sensor_config->camera_config->addr = addr;
			// Update mipi rx phy
			// 修正配置中的 sensor 使用的 mipi rx phy
			// 此处的修改并不一定是最终修改，在vpp 的impl 的 param 设置中会根据具体情况再次修改
			sensor_config->vin_attr->vin_node_attr.cim_attr.mipi_rx = vcon_props.rx_phy[1];
			return 0;
		} else {
			printf("WARN: Sensor Name: %s, Expected Chip ID: 0x%02X, Actual Chip ID Read: 0x%02X\n",
					sensor_config->sensor_name, sensor_config->chip_id & 0x0000FFFF, chip_id);
			continue;
		}

	}

	// If none of the addresses worked
	// printf("Failed to read sensor chip ID register\n");
	return -1;
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

int get_board_id(char *data, size_t size)
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

static void should_used_csi(int *is_need_used_csi)
{
	char board_id[16];
	int ret = get_board_id(board_id, sizeof(board_id));

	if (ret == 0) {
		if (strncmp(board_id, "201", 3) == 0) {
			printf("[INFO] board_id is %s, so skip csi test for index 1\n", board_id);
			is_need_used_csi[1] = false;// board 201 not use csi1
		}

		if (strncmp(board_id, "301", 3) == 0) {
			printf("[INFO] board_id is %s, so skip csi test for index 1 and index 3\n", board_id);
			is_need_used_csi[1] = false;// board 301 not use csi1 csi3
			is_need_used_csi[3] = false;
		}
		if (strncmp(board_id, "302", 3) == 0) {
			printf("[INFO] board_id is %s, so skip csi test for index 1 and index 3\n", board_id);
			is_need_used_csi[1] = false;// board 302 not use csi1 csi3
			is_need_used_csi[3] = false;
		}
	} else {
		printf("read board_id file failed, so skip csi.\n");
	}
}

static int32_t vp_sensor_mipi_host_mclk_is_not_configed(int csi_index){
	int mclk_is_not_configed = 0;
	struct mipi_properties mipi_property;

	read_mipi_info_from_device_tree(csi_index, &mipi_property);
	if(strlen(mipi_property.pinctrl_names) == 0){
			mclk_is_not_configed = 1;
			//printf("mipi mclk is not configed.\n");
		}else{

			//printf("mipi mclk is configed.\n");
		}
	return mclk_is_not_configed;
}

void vp_deserial_config_update(deserial_config_t *deserial, const camera_config_t *camera_config, int link_port){
	if (!deserial || !camera_config) {
		return;
	}
	snprintf(deserial->link_desp[link_port],
			sizeof(deserial->link_desp[link_port]),
			"%.32s:%d@%d",
			camera_config->name, camera_config->extra_mode, camera_config->config_index);

	if(camera_config->sensor_mode == 6){
		deserial->gpio_mfp[link_port] = 0x05;
	}
}

void vp_sensor_detect_structed(csi_list_info_t *csi_list_info)
{
	struct vcon_properties vcon_props_array[VP_MAX_VCON_NUM];
	struct mipi_properties mipi_props_array[VP_MAX_VCON_NUM];
	csi_list_info->valid_count = 0;
	csi_list_info->max_count = VP_MAX_VCON_NUM;
	int is_need_used_csi[VP_MAX_VCON_NUM] = {true, true, true, true};
	should_used_csi(is_need_used_csi);
	// Iterate over vcon@0 - 3
	for (int i = 0; i < VP_MAX_VCON_NUM; ++i) {
		csi_info_t csi_info_tmp = {.index = i, .is_valid = 0};
		read_vcon_info_from_device_tree(i, &vcon_props_array[i]);
		read_mipi_info_from_device_tree(i, &mipi_props_array[i]);
		if (is_need_used_csi[i] == false) {
			csi_list_info->csi_info[i] = csi_info_tmp;
			continue;
		}

		int mclk_is_not_configed = 0;
		printf("\n");
		printf("Searching camera sensor on device: %s ", vcon_props_array[i].device_path);
		printf("i2c bus: %d ", vcon_props_array[i].bus);
		printf("mipi rx phy: %d\n", vcon_props_array[i].rx_phy[1]);
		if(strlen(mipi_props_array[i].pinctrl_names) == 0){
			mclk_is_not_configed = 1;
			//printf("mipi mclk is not configed.\n");
		}else{
			//printf("mipi mclk is configed.\n");
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

				int ret = check_sensor_reg_value(vcon_props_array[i], vp_sensor_config_list[j]);
				if (ret == 0) {

					printf("INFO: Support sensor name:%s on mipi rx csi %d, "
							"i2c addr 0x%x, config_file:%s\n",
						vp_sensor_config_list[j]->sensor_name,
						vcon_props_array[i].rx_phy[1],
						vp_sensor_config_list[j]->camera_config->addr,
						vp_sensor_config_list[j]->config_file);

					csi_info_tmp.index = i;
					csi_info_tmp.is_valid = 1;


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


int32_t vp_sensor_multi_fixed_mipi_host(vp_sensor_config_t *sensor_config, int used_mipi_host, vp_csi_config_t* csi_config)
{
	int32_t ret = -1, j = 0;
	static int32_t i = 0;
	uint32_t frequency = 24000000;
	int is_need_used_csi[VP_MAX_VCON_NUM] = {true, true, true, true};
	should_used_csi(is_need_used_csi);

	struct vcon_properties vcon_props_array[VP_MAX_VCON_NUM];

	// Iterate over vcon@0 - 3
	for (i = 0; i < VP_MAX_VCON_NUM; ++i) {
		if (is_need_used_csi[i] == false) {
			continue;
		}
		// 跳过使用使用的mipi csi控制器，支持同时接入相同的摄像头
		if (used_mipi_host & (1 << i))
			continue;

		if (check_mipi_host_status(i) == 0)
			continue;

		int mclk_is_not_configed = vp_sensor_mipi_host_mclk_is_not_configed(i);
		read_vcon_info_from_device_tree(i, &vcon_props_array[i]);

		printf("Searching camera sensor on device: %s ", vcon_props_array[i].device_path);
		printf("i2c bus: %d ", vcon_props_array[i].bus);
		printf("mipi rx phy: %d\n", vcon_props_array[i].rx_phy[1]);

		// 如果该vcon使能了，检测该vcon上是否有连接 sensor
		if (vcon_props_array[i].status[0] == 'o') { // okay
			// 检测该vcon上连接的 sensor
			/*enable gpio_oth, enable camera sensor gpio, maybe pwd/reset gpio */
			for (j = 0; j < 8; ++j) {
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
				write_mipi_host_freq(i, frequency);
				enable_mipi_host_clock(i, 1);
			}

			// 从指定的vcon关联的i2c bus上读取 vp_sensor_config_list 中指定的 chip_id_reg 对应的寄存器值
			ret = check_sensor_reg_value(vcon_props_array[i], sensor_config);
			if (ret == 0) {
				csi_config->index = i;
				csi_config->mclk_is_not_configed = mclk_is_not_configed;
				// 检测到 sensor，保存 sensor 信息
				printf("INFO: Found sensor_name:%s on mipi rx csi %d, i2c addr 0x%x, config_file:%s\n",
					sensor_config->sensor_name, vcon_props_array[i].rx_phy[1],
					sensor_config->camera_config->addr, sensor_config->config_file);
				break;
			}
		}
	}

	return ret;
}


int32_t vp_sensor_fixed_mipi_host(vp_sensor_config_t *sensor_config, vp_csi_config_t* csi_config)
{
	int32_t ret = 0, i = 0, j = 0;
	uint32_t frequency = 24000000;
	int is_need_used_csi[VP_MAX_VCON_NUM] = {true, true, true, true};
	should_used_csi(is_need_used_csi);

	struct vcon_properties vcon_props_array[VP_MAX_VCON_NUM];

	// Iterate over vcon@0 - 3
	for (i = 0; i < VP_MAX_VCON_NUM; ++i) {
		if (is_need_used_csi[i] == false) {
			continue;
		}
		if (check_mipi_host_status(i) == 0)
			continue;
		int mclk_is_not_configed = vp_sensor_mipi_host_mclk_is_not_configed(i);
		read_vcon_info_from_device_tree(i, &vcon_props_array[i]);

		printf("Searching camera sensor on device: %s ", vcon_props_array[i].device_path);
		printf("i2c bus: %d ", vcon_props_array[i].bus);
		printf("mipi rx phy: %d\n", vcon_props_array[i].rx_phy[1]);
		// printf("mipi mclk: %d\n", vcon_props_array[i].rx_phy[1]);

		// 如果该vcon使能了，检测该vcon上是否有连接 sensor
		if (vcon_props_array[i].status[0] == 'o') { // okay
			// 检测该vcon上连接的 sensor
			/*enable gpio_oth, enable camera sensor gpio, maybe pwd/reset gpio */
			for (j = 0; j < 8; ++j) {
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
				write_mipi_host_freq(i, frequency);
				enable_mipi_host_clock(i, 1);

			}
			// 从指定的vcon关联的i2c bus上读取 vp_sensor_config_list 中指定的 chip_id_reg 对应的寄存器值
			ret = check_sensor_reg_value(vcon_props_array[i], sensor_config);
			if (ret == 0) {
				csi_config->index = i;
				csi_config->mclk_is_not_configed = mclk_is_not_configed;
				// sensor_config->csi_index = vcon_props_array[i].rx_phy[1];
				// 检测到 sensor，保存 sensor 信息
				printf("INFO: Found sensor_name:%s on mipi rx csi %d, i2c addr 0x%x, config_file:%s\n",
					sensor_config->sensor_name, vcon_props_array[i].rx_phy[1],
					sensor_config->camera_config->addr, sensor_config->config_file);

				break;
			}
		}
	}

	return ret;
}


int32_t vp_sensor_detect_2(int host, mipi_host_info_t* host_info)
{
	int32_t ret = -1, j = 0;
	uint32_t frequency = 24000000;

	struct vcon_properties vcon_props;

	int mclk_is_not_configed = vp_sensor_mipi_host_mclk_is_not_configed(host);
	read_vcon_info_from_device_tree(host, &vcon_props);

	printf("Searching camera sensor on device: %s ", vcon_props.device_path);
	//printf("mclk_is_not_configed: %d\n", mclk_is_not_configed);
	printf("i2c bus: %d ", vcon_props.bus);
	printf("mipi rx phy: %d\n", vcon_props.rx_phy[1]);

	// 如果该vcon使能了，检测该vcon上是否有连接 sensor
	if (vcon_props.status[0] == 'o') { // okay
		if(!mclk_is_not_configed) {
			/* enable mclk */
			write_mipi_host_freq(host, frequency);
			enable_mipi_host_clock(host, 1);
		}
		for (j = 0; j < vp_get_sensors_list_number(); j++) {
			// 从指定的vcon关联的i2c bus上读取 vp_sensor_config_list 中指定的 chip_id_reg 对应的寄存器值
			/*enable gpio_oth, enable camera sensor gpio, maybe pwd/reset gpio */
			for (int k = 0; k < 8; ++k) {
				if (vcon_props.gpio_oth[k] != 0) {
					if ((vp_sensor_config_list[j]->camera_config->gpio_enable & (1 << k)) != 0) {
						// gpio_level should be from sensor config and sensor spec
						enable_sensor_pin(vcon_props.gpio_oth[k],
							(1 - vp_sensor_config_list[j]->camera_config->gpio_level));
					}
				}
			}

			ret = check_sensor_reg_value(vcon_props, vp_sensor_config_list[j]);
			if (ret == 0) {
				// 检测到 sensor，保存 sensor 信息
				printf("INFO: vp_sensor_detect_2 Support sensor index:%d, sensor_name:%s on mipi rx csi %d, "
						"i2c addr 0x%x, config_file:%s\n",
					host, vp_sensor_config_list[j]->sensor_name,
					vcon_props.rx_phy[1],
					vp_sensor_config_list[j]->camera_config->addr,
					vp_sensor_config_list[j]->config_file);
				
				host_info->host_num = host;
				host_info->sensor_index = j;
				host_info->addr = vp_sensor_config_list[j]->camera_config->addr;
				host_info->mipi_rx = vcon_props.rx_phy[1];
				return 0;
			}
		}
	}
	return -1;

}

int32_t vp_sensor_fixed_mipi_host_1(int host, vp_sensor_config_t *sensor_config, vp_csi_config_t* csi_config)
{
	int32_t ret = -1, j = 0;
	uint32_t frequency = 24000000;
	if (check_mipi_host_status(host) == 0) {
		return -1;
	}

	//struct vcon_properties vcon_props_array[VP_MAX_VCON_NUM];
	struct vcon_properties vcon_props;

	int mclk_is_not_configed = vp_sensor_mipi_host_mclk_is_not_configed(host);
	read_vcon_info_from_device_tree(host, &vcon_props);

	printf("Searching camera sensor on device: %s ", vcon_props.device_path);
	//printf("mclk_is_not_configed: %d\n", mclk_is_not_configed);
	printf("i2c bus: %d ", vcon_props.bus);
	printf("mipi rx phy: %d\n", vcon_props.rx_phy[1]);

	// 如果该vcon使能了，检测该vcon上是否有连接 sensor
	if (vcon_props.status[0] == 'o') { // okay
		if(!mclk_is_not_configed) {
			/* enable mclk */
			write_mipi_host_freq(host, frequency);
			enable_mipi_host_clock(host, 1);
		}
#if 1
		// 检测该vcon上连接的 sensor
		/*enable gpio_oth, enable camera sensor gpio, maybe pwd/reset gpio */
		for (j = 0; j < 8; ++j) {
			if (vcon_props.gpio_oth[j] != 0) {
				if (sensor_config->camera_config->gpio_enable != 0) {
					// gpio_level should be from sensor config and sensor spec
					enable_sensor_pin(vcon_props.gpio_oth[j],
						(1 - sensor_config->camera_config->gpio_level));
				}
			}
		}


#endif

		// 从指定的vcon关联的i2c bus上读取 vp_sensor_config_list 中指定的 chip_id_reg 对应的寄存器值
		ret = check_sensor_reg_value(vcon_props, sensor_config);
		if (ret == 0) {
			csi_config->index = host;
			csi_config->mclk_is_not_configed = mclk_is_not_configed;
			// 检测到 sensor，保存 sensor 信息
			printf("INFO: vp_sensor_fixed_mipi_host_1 Found sensor_name:%s on mipi rx csi %d, i2c addr 0x%x, config_file:%s\n",
				sensor_config->sensor_name, vcon_props.rx_phy[1],
				sensor_config->camera_config->addr, sensor_config->config_file);
		}
	}

	return ret;
}


int copy_config(vp_sensor_config_t* dest, vp_sensor_config_t* src) {
	if ((dest == NULL) || (src == NULL)) {
		return -1;
	}

	memcpy(dest, src, sizeof(vp_sensor_config_t));

	dest->camera_config = NULL;
	dest->camera_slave_config = NULL;
	dest->vin_attr = NULL;
	dest->isp_cfg = NULL;
	dest->ynr_attr = NULL;
	dest->pym_cfg = NULL;
	dest->deserial_attr = NULL;
	dest->deserial_slave_attr = NULL;

	if (src->camera_config) {
		dest->camera_config = malloc(sizeof(camera_config_t));
		if (dest->camera_config == NULL) {
			return -1;
		}
		memcpy(dest->camera_config, src->camera_config, sizeof(camera_config_t));
		if (src->camera_config->mipi_cfg) {
			dest->camera_config->mipi_cfg = malloc(sizeof(mipi_config_t));
			if (dest->camera_config->mipi_cfg == NULL) {
				return -1;
			}
			memcpy(dest->camera_config->mipi_cfg, src->camera_config->mipi_cfg, sizeof(mipi_config_t));
		}
	}
	if (src->camera_slave_config) {
		dest->camera_slave_config = malloc(sizeof(camera_config_t));
		if (dest->camera_slave_config == NULL) {
			return -1;
		}
		memcpy(dest->camera_slave_config, src->camera_slave_config, sizeof(camera_config_t));
		if (src->camera_slave_config->mipi_cfg) {
			dest->camera_slave_config->mipi_cfg = malloc(sizeof(mipi_config_t));
			if (dest->camera_slave_config->mipi_cfg == NULL) {
				return -1;
			}
			memcpy(dest->camera_slave_config->mipi_cfg, src->camera_slave_config->mipi_cfg, sizeof(mipi_config_t));
		}
	}
	if (src->vin_attr) {
		dest->vin_attr = malloc(sizeof(vin_attr_t));
		if (dest->vin_attr == NULL) {
			return -1;
		}
		memcpy(dest->vin_attr, src->vin_attr, sizeof(vin_attr_t));
	}
	if (src->isp_cfg) {
		dest->isp_cfg = malloc(sizeof(isp_cfg_t));
		if (dest->isp_cfg == NULL) {
			return -1;
		}
		memcpy(dest->isp_cfg, src->isp_cfg, sizeof(isp_cfg_t));
	}
	if (src->ynr_attr) {
		dest->ynr_attr = malloc(sizeof(struct ynr_init_attr));
		if (dest->ynr_attr == NULL) {
			return -1;
		}
		memcpy(dest->ynr_attr, src->ynr_attr, sizeof(struct ynr_init_attr));
	}
	if (src->pym_cfg) {
		dest->pym_cfg = malloc(sizeof(pym_cfg_t));
		if (dest->pym_cfg == NULL) {
			return -1;
		}
		memcpy(dest->pym_cfg, src->pym_cfg, sizeof(pym_cfg_t));
	}
	if (src->deserial_attr) {
		dest->deserial_attr = malloc(sizeof(deserial_config_t));
		if (dest->deserial_attr == NULL) {
			return -1;
		}
		memcpy(dest->deserial_attr, src->deserial_attr, sizeof(deserial_config_t));
	}
	if (src->deserial_slave_attr) {
		dest->deserial_slave_attr = malloc(sizeof(deserial_config_t));
		if (dest->deserial_slave_attr == NULL) {
			return -1;
		}
		memcpy(dest->deserial_slave_attr, src->deserial_slave_attr, sizeof(deserial_config_t));
	}
	return 0;
}

void free_config(vp_sensor_config_t* config) {
	if (config == NULL) {
		return;
	}
	
	// Free all dynamically allocated pointers
	if (config->camera_config->mipi_cfg) {
		free(config->camera_config->mipi_cfg);
		config->camera_config->mipi_cfg = NULL;
	}
	if (config->camera_config) {
		free(config->camera_config);
		config->camera_config = NULL;
	}
	if (config->camera_slave_config->mipi_cfg) {
		free(config->camera_slave_config->mipi_cfg);
		config->camera_slave_config->mipi_cfg = NULL;
	}
	if (config->camera_slave_config) {
		free(config->camera_slave_config);
		config->camera_slave_config = NULL;
	}
	if (config->vin_attr) {
		free(config->vin_attr);
		config->vin_attr = NULL;
	}
	if (config->isp_cfg) {
		free(config->isp_cfg);
		config->isp_cfg = NULL;
	}
	if (config->ynr_attr) {
		free(config->ynr_attr);
		config->ynr_attr = NULL;
	}
	if (config->pym_cfg) {
		free(config->pym_cfg);
		config->pym_cfg = NULL;
	}
	if (config->deserial_attr) {
		free(config->deserial_attr);
		config->deserial_attr = NULL;
	}
	if (config->deserial_slave_attr) {
		free(config->deserial_slave_attr);
		config->deserial_slave_attr = NULL;
	}
}


int copy_deserial_config(deserial_config_t* dest, deserial_config_t* src) {
	if ((dest == NULL) || (src == NULL)) {
		return -1;
	}

	memcpy(dest, src, sizeof(deserial_config_t));
	if (src->poc_cfg) {
		dest->poc_cfg = malloc(sizeof(poc_config_t));
		if (dest->poc_cfg == NULL) {
			return -1;
		}
		memcpy(dest->poc_cfg, src->poc_cfg, sizeof(poc_config_t));
	}
	return 0;
}


