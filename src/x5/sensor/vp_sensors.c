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

extern vp_sensor_config_t sc1330t_linear_1280x960_raw10_30fps_1lane;
extern vp_sensor_config_t irs2875_linear_208x1413_raw12_15fps_2lane;
extern vp_sensor_config_t sc230ai_linear_1920x1080_raw10_10fps_1lane;
extern vp_sensor_config_t sc230ai_linear_1920x1080_raw10_30fps_1lane;
extern vp_sensor_config_t sc132gs_linear_1088x1280_raw10_10fps_2lane;
extern vp_sensor_config_t sc132gs_linear_896x896_raw10_10fps_2lane;
extern vp_sensor_config_t sc035hgs_linear_640x480_raw10_30fps_1lane;
extern vp_sensor_config_t ov5640_linear_1920x1080_raw10_30fps_2lane;
extern vp_sensor_config_t f37_linear_1920x1080_raw10_30fps_1lane;

vp_sensor_config_t *vp_sensor_config_list[] = {
	&sc1330t_linear_1280x960_raw10_30fps_1lane,
	&irs2875_linear_208x1413_raw12_15fps_2lane,
	&sc230ai_linear_1920x1080_raw10_10fps_1lane,
	&sc230ai_linear_1920x1080_raw10_30fps_1lane,
	&sc132gs_linear_1088x1280_raw10_10fps_2lane,
	&sc132gs_linear_896x896_raw10_10fps_2lane,
	&sc035hgs_linear_640x480_raw10_30fps_1lane,
	&ov5640_linear_1920x1080_raw10_30fps_2lane,
	&f37_linear_1920x1080_raw10_30fps_1lane
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

	// Set GPIO direction to output
	if (gpio_set_direction(gpio_number, "out") != 0) {
		printf("Failed to set GPIO direction\n");
		return -1;
	}

	/* gpio level should be keep same with sensor driver power_on api */

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

static void read_device_tree(const int device, struct vcon_properties *properties) {
	memset(properties, 0, sizeof(struct vcon_properties));

	snprintf(properties->device_path, sizeof(properties->device_path),
		"/proc/device-tree/soc/cam/vcon@%d", device);

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

// Function to write frequency to MIPI host
static void write_mipi_host_freq(int freq, int host_id) {
	char path[256];
	FILE *file;

	// Construct path to the file
	snprintf(path, 256, "/sys/class/vps/mipi_host%d/param/snrclk_freq", host_id);

	// Open the file for writing
	file = fopen(path, "w");
	if (file) {
		// Write frequency to the file
		fprintf(file, "%d", freq);
		fclose(file);
	}
}

// Function to enable MIPI host clock
static void enable_mipi_host_clock(int enable, int host_id) {
	char path[256];
	FILE *file;

	// Construct path to the file
	snprintf(path, 256, "/sys/class/vps/mipi_host%d/param/snrclk_en", host_id);

	// Open the file for writing
	file = fopen(path, "w");
	if (file) {
		// Write enable value to the file
		fprintf(file, "%d", enable);
		fclose(file);
	}
}


int32_t vp_sensor_detect(char *sensor_list, int32_t *num_sensors)
{
	int32_t ret = 0, i = 0, j = 0, k = 0, index = 0;
	uint32_t frequency = 24000000;
	char sensor_name_with_prefix[256] = {0};

	struct vcon_properties vcon_props_array[VP_MAX_VCON_NUM];

	// Iterate over vcon@0 - 3
	for (i = 0; i < VP_MAX_VCON_NUM; ++i) {
		read_device_tree(i, &vcon_props_array[i]);

		printf("Searching camera sensor on device: %s ", vcon_props_array[i].device_path);
		printf("i2c bus: %d ", vcon_props_array[i].bus);
		printf("mipi rx phy: %d\n", vcon_props_array[i].rx_phy[1]);

		// 如果该vcon使能了，检测该vcon上是否有连接 sensor
		if (vcon_props_array[i].status[0] == 'o') { // okay
			/* enable mclk */
			write_mipi_host_freq(frequency, i);
			enable_mipi_host_clock(1, i);

			for (j = 0; j < vp_get_sensors_list_number(); j++) {
				// 从指定的vcon关联的i2c bus上读取 vp_sensor_config_list 中指定的 chip_id_reg 对应的寄存器值
				/*enable gpio_oth, enable camera sensor gpio, maybe pwd/reset gpio */
				for (k = 0; k < 8; ++k) {
					if (vcon_props_array[i].gpio_oth[k] != 0) {
						if ((vp_sensor_config_list[j]->camera_config->gpio_enable_bit & (1 << k)) != 0) {
							// gpio_level should be from sensor config and sensor spec
							enable_sensor_pin(vcon_props_array[i].gpio_oth[k],
								(1 - vp_sensor_config_list[j]->camera_config->gpio_level_bit));
						}
					}
				}

				ret = check_sensor_reg_value(vcon_props_array[i], vp_sensor_config_list[j]);
				if (ret == 0) {
					// 检测到 sensor，保存 sensor 信息
					printf("INFO: Support sensor index:%d, sensor_name:%s on mipi rx csi %d, "
							"i2c addr 0x%x, config_file:%s\n",
						index++, vp_sensor_config_list[j]->sensor_name,
						vcon_props_array[i].rx_phy[1],
						vp_sensor_config_list[j]->camera_config->addr,
						vp_sensor_config_list[j]->config_file);
					if (strlen(sensor_list) > 1) {
						strcat(sensor_list, "/");
					}
					// 添加 CSI_%d- 前缀
					memset(sensor_name_with_prefix, 0, sizeof(sensor_name_with_prefix));
					sprintf(sensor_name_with_prefix, "CSI_%d-%s", i, vp_sensor_config_list[j]->sensor_name);
					strcat(sensor_list, sensor_name_with_prefix);
				}
			}

			// Disable frequency
			enable_mipi_host_clock(0, i);
		}
	}

	*num_sensors = index;

	return -1;
}

int32_t vp_sensor_fixed_mipi_host(vp_sensor_config_t *sensor_config)
{
	int32_t ret = 0, i = 0, j = 0;
	uint32_t frequency = 24000000;

	struct vcon_properties vcon_props_array[VP_MAX_VCON_NUM];

	// Iterate over vcon@0 - 3
	for (i = 0; i < VP_MAX_VCON_NUM; ++i) {
		read_device_tree(i, &vcon_props_array[i]);

		printf("Searching camera sensor on device: %s ", vcon_props_array[i].device_path);
		printf("i2c bus: %d ", vcon_props_array[i].bus);
		printf("mipi rx phy: %d\n", vcon_props_array[i].rx_phy[1]);

		// 如果该vcon使能了，检测该vcon上是否有连接 sensor
		if (vcon_props_array[i].status[0] == 'o') { // okay
			// 检测该vcon上连接的 sensor
			/*enable gpio_oth, enable camera sensor gpio, maybe pwd/reset gpio */
			for (j = 0; j < 8; ++j) {
				if (vcon_props_array[i].gpio_oth[j] != 0) {
					if (sensor_config->camera_config->gpio_enable_bit != 0) {
						// gpio_level should be from sensor config and sensor spec
						enable_sensor_pin(vcon_props_array[i].gpio_oth[j],
							(1 - sensor_config->camera_config->gpio_level_bit));
					}
				}
			}

			/* enable mclk */
			write_mipi_host_freq(frequency, i);
			enable_mipi_host_clock(1, i);

			// 从指定的vcon关联的i2c bus上读取 vp_sensor_config_list 中指定的 chip_id_reg 对应的寄存器值
			ret = check_sensor_reg_value(vcon_props_array[i], sensor_config);
			if (ret == 0) {
				// 检测到 sensor，保存 sensor 信息
				printf("INFO: Found sensor_name:%s on mipi rx csi %d, i2c addr 0x%x, config_file:%s\n",
					sensor_config->sensor_name, vcon_props_array[i].rx_phy[1],
					sensor_config->camera_config->addr, sensor_config->config_file);
			}

			// Disable frequency
			enable_mipi_host_clock(0, i);
		}
	}

	return -1;
}

int32_t vp_sensor_fixed_mipi_host_1(int host, vp_sensor_config_t *sensor_config)
{
	int32_t ret = -1, i = 0, j = 0;
	uint32_t frequency = 24000000;

	struct vcon_properties vcon_props_array[VP_MAX_VCON_NUM];

	i = host;

	read_device_tree(i, &vcon_props_array[i]);

	printf("Searching camera sensor on device: %s ", vcon_props_array[i].device_path);
	printf("i2c bus: %d ", vcon_props_array[i].bus);
	printf("mipi rx phy: %d\n", vcon_props_array[i].rx_phy[1]);

	// 如果该vcon使能了，检测该vcon上是否有连接 sensor
	if (vcon_props_array[i].status[0] == 'o') { // okay
		// 检测该vcon上连接的 sensor
		/*enable gpio_oth, enable camera sensor gpio, maybe pwd/reset gpio */
		for (j = 0; j < 8; ++j) {
			if (vcon_props_array[i].gpio_oth[j] != 0) {
				if (sensor_config->camera_config->gpio_enable_bit != 0) {
					// gpio_level should be from sensor config and sensor spec
					enable_sensor_pin(vcon_props_array[i].gpio_oth[j],
						(1 - sensor_config->camera_config->gpio_level_bit));
				}
			}
		}

		/* enable mclk */
		write_mipi_host_freq(frequency, i);
		enable_mipi_host_clock(1, i);

		// 从指定的vcon关联的i2c bus上读取 vp_sensor_config_list 中指定的 chip_id_reg 对应的寄存器值
		ret = check_sensor_reg_value(vcon_props_array[i], sensor_config);
		if (ret == 0) {
			// 检测到 sensor，保存 sensor 信息
			printf("INFO: Found sensor_name:%s on mipi rx csi %d, i2c addr 0x%x, config_file:%s\n",
				sensor_config->sensor_name, vcon_props_array[i].rx_phy[1],
				sensor_config->camera_config->addr, sensor_config->config_file);
		}

		// Disable frequency
		enable_mipi_host_clock(0, i);
	}

	return ret;
}


int copy_config(vp_sensor_config_t* dest, vp_sensor_config_t* src) {
	printf("copy_config----------9999999999\r\n");
	if ((dest == NULL) || (src == NULL)) {
		return -1;
	}
	printf("copy_config----------1111111111\r\n");

	memcpy(dest, src, sizeof(vp_sensor_config_t));
	if (src->camera_config) {
		printf("copy_config----------22222222\r\n");
		dest->camera_config = malloc(sizeof(camera_config_t));
		memcpy(dest->camera_config, src->camera_config, sizeof(camera_config_t));
	}
	if (src->camera_config) {
		printf("copy_config----------33333333\r\n");
		dest->camera_config = malloc(sizeof(vin_node_attr_t));
		memcpy(dest->camera_config, src->camera_config, sizeof(vin_node_attr_t));
	}
	if (src->camera_config) {
		printf("copy_config----------44444444\r\n");
		dest->camera_config = malloc(sizeof(vin_ichn_attr_t));
		memcpy(dest->camera_config, src->camera_config, sizeof(vin_ichn_attr_t));
	}
	if (src->camera_config) {
		printf("copy_config----------555555\r\n");
		dest->camera_config = malloc(sizeof(vin_ochn_attr_t));
		memcpy(dest->camera_config, src->camera_config, sizeof(vin_ochn_attr_t));
	}
	if (src->isp_attr) {
		printf("copy_config----------666666666\r\n");
		dest->isp_attr = malloc(sizeof(isp_attr_t));
		memcpy(dest->isp_attr, src->isp_attr, sizeof(isp_attr_t));
	}
	if (src->isp_ichn_attr) {
		printf("copy_config----------77777\r\n");
		dest->isp_ichn_attr = malloc(sizeof(isp_ichn_attr_t));
		memcpy(dest->isp_ichn_attr, src->isp_ichn_attr, sizeof(isp_ichn_attr_t));
	}
	if (src->isp_ochn_attr) {
		printf("copy_config----------8888888888\r\n");
		dest->isp_ochn_attr = malloc(sizeof(isp_ochn_attr_t));
		memcpy(dest->isp_ochn_attr, src->isp_ochn_attr, sizeof(isp_ochn_attr_t));
	}
	return 0;
}
