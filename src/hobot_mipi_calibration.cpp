#include "hobot_mipi_calibration.hpp"
#include "hobot_mipi_comm.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "opencv2/opencv.hpp"

#include <string>
#include <fstream>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <regex>
#include <cmath>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <vector>
#include <filesystem>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <algorithm>
#include <yaml-cpp/yaml.h>

#include <sys/select.h>

#include "hb_media_codec.h"
#include "hb_media_error.h"

#include <rclcpp/rclcpp.hpp>
#include <json/json.h>
namespace fs = std::filesystem; 

namespace mipi_cam {

bool mipi_calibration::readEeprom16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, char* buf, int bufsize) {
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
		//std::cout << "Failed to open the I2C bus " << bus << std::endl;
		//perror("open the I2C bus");
		return false;
	}

	sendbuf[0] = (uint8_t)((reg_addr >> 8u) & 0xffu);
	sendbuf[1] = (uint8_t)(reg_addr & 0xffu);

	data.msgs = msgs; /*PRQA S 5118*/
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	data.msgs[1].len = bufsize;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = I2C_M_RD;
	data.msgs[1].buf = (uint8_t*)buf;

	ret = ioctl(file, I2C_RDWR, (uint64_t)&data);
	if (ret < 0) {
		// perror("Failed to read from the I2C bus");
		//*value = 0;
		close(file);
		return false;
	}

	//*value = (uint16_t)((readbuf[0] << 8) | readbuf[1]);

	// Close the I2C bus
	close(file);

	return true;
}

std::vector<int> mipi_calibration::i2c_bus_detect() {
	std::vector<int> buses;
    std::string path = "/sys/class/i2c-dev";

    // 1. 获取所有 I2C 总线 ID
    try {
        for (const auto& entry : fs::directory_iterator(path)) {
            std::string dirname = entry.path().filename().string(); // 例如 "i2c-1"
            if (dirname.find("i2c-") == 0) {
                int id = std::stoi(dirname.substr(4));
                buses.push_back(id);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "无法访问总线目录: " << e.what() << std::endl;
		std::sort(buses.begin(), buses.end());
        return buses;
    }

    std::sort(buses.begin(), buses.end());
	return buses;
}

int mipi_calibration::detectEeprom_lianhe(std::string &device, int &i2c_bus, uint16_t &i2c_addr) {

  // mipi sensor的信息数组
  EEPROM_ID_T eeprom_id_list[] = {
    {1, 0x50, I2C_ADDR_16, 0x21, 0x01, "P24C64G-C4H-MIR"},  // P24C64G-C4H-MIR
  };
  std::vector<int> i2c_buss= i2c_bus_detect();

  char buf[512];
  std::vector<char> buf_type;
  buf_type.resize(0x1f);
  char check_0;
  char checksum;
  std::string chip_type;

  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"==========detectEeprom lianhe start==========\n");

  for (auto num : i2c_buss) {
    for (auto eeprom_id : eeprom_id_list) {
      if (readEeprom16(num, eeprom_id.i2c_dev_addr, eeprom_id.det_reg, buf, 1)) {
		readEeprom16(num, eeprom_id.i2c_dev_addr, 0x0000, &check_0, 1);
		readEeprom16(num, eeprom_id.i2c_dev_addr, 0x0001, buf_type.data(), 0x1f);
		readEeprom16(num, eeprom_id.i2c_dev_addr, 0x0020, &checksum, 1);
		chip_type = buf_type.data();
		int sum = 0;
		std::for_each(buf_type.begin(), buf_type.end(), [&sum](char c) {
			sum += static_cast<int>(c);
		});

		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"eeprom infomation:" \
			"\n ----------------" \
			"\n bus: %d" \
			"\n check_0: %s" \
			"\n chip_type: %s" \
			"\n checksum: %s" \
			"\n sum: %s" \
			"\n ----------------",
			num,
			std::to_string(check_0).c_str(),
			chip_type.c_str(),
			std::to_string(checksum).c_str(),
			std::to_string(sum%255).c_str()
		);
		if (buf[0] == eeprom_id.check_value) {
			i2c_bus = num;
			i2c_addr = eeprom_id.i2c_dev_addr;
			device = eeprom_id.device_name;
			return 0;
		} else {
			RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"eeprom check failure bus: %d, addr: %d, device_name: %s", num, eeprom_id.i2c_dev_addr, eeprom_id.device_name);
		}
      }
    }
  }
  return -1;
}

int mipi_calibration::detectEeprom_drobot(int i2c_bus, std::string &device, uint16_t &i2c_addr) {

	// mipi sensor的信息数组
	EEPROM_DETECT_T eeprom_detect_list[] = {
		{1, 0x50, I2C_ADDR_16, 0x00, "SZYGSJKJ", "yuguang"},  // P24C64G-C4H-MIR
		{1, 0x50, I2C_ADDR_16, 0x00, "UNION", "union"},  // P24C64G-C4H-MIR
		{1, 0x50, I2C_ADDR_16, 0x00, "SZ_ABHAM", "sz_abham"},  // P24C64G-C4H-MIR
	};

	char buf[9] = {0};
	std::vector<char> buf_type;
	buf_type.resize(0x1f);
	char check_0;
	char checksum;
	std::string chip_type;
  
	for (auto eeprom_id : eeprom_detect_list) {
		if (readEeprom16(i2c_bus, eeprom_id.i2c_dev_addr, eeprom_id.det_reg, buf, 8)) {
		std::string buf_str = buf;
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),"i2c bus: %d, EEPROM FLAG: %s\n", i2c_bus, buf_str.c_str());
		if (eeprom_id.check_str == buf_str) {
			i2c_addr = eeprom_id.i2c_dev_addr;
			device = eeprom_id.device_name;
			return 0;
		}
		}
	}
    return -1;
}

int mipi_calibration::detectEeprom_baolong(std::string &device, int &i2c_bus, uint16_t &i2c_addr)
{
	constexpr uint16_t EEPROM_ADDR = 0x50;
	const std::string EXPECTED_S_NUMBER = "SE401839";
	constexpr uint8_t EXPECTED_LR_CAMERA = 0x12;
	constexpr uint8_t EXPECTED_DATA_FORMAT = 0x02;
	constexpr uint8_t EXPECTED_DIST_TYPE = 0x05;

	std::vector<int> buses = i2c_bus_detect();
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
					"detectEeprom_baolong: scanning %zu I2C buses...", buses.size());

	for (auto bus : buses)
	{
		// 1. S_number (0x12, 8 bytes) — 模组识别码
		char s_buf[8] = {0};
		if (!readEeprom16(bus, EEPROM_ADDR, 0x0012, s_buf, 8))
			continue;
		if (std::string(s_buf, 8) != EXPECTED_S_NUMBER)
			continue;

		// 2. Data_format (0x21, 1 byte)
		uint8_t data_format = 0;
		if (!readEeprom16(bus, EEPROM_ADDR, 0x0021, (char *)&data_format, 1))
			continue;
		if (data_format != EXPECTED_DATA_FORMAT)
			continue;

		// 3. L_R_camera (0x35, 1 byte) — 0x12=单EEPROM双目
		uint8_t lr_camera = 0;
		if (!readEeprom16(bus, EEPROM_ADDR, 0x0035, (char *)&lr_camera, 1))
			continue;
		if (lr_camera != EXPECTED_LR_CAMERA)
		{
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
							"bus %d: L_R_camera=0x%02X (need 0x12, single-EEPROM dual-cam)", bus, lr_camera);
			continue;
		}

		// 4. Distortion_Type (0x55, 1 byte) — 0x05=Rational Model
		uint8_t dist_type = 0;
		if (!readEeprom16(bus, EEPROM_ADDR, 0x0055, (char *)&dist_type, 1))
			continue;
		if (dist_type != EXPECTED_DIST_TYPE)
			continue;

		// 5. M_number (0x00, 18 bytes) — 仅用于日志
		char m_buf[18] = {0};
		readEeprom16(bus, EEPROM_ADDR, 0x0000, m_buf, 18);
		std::string m_number(m_buf, strnlen(m_buf, 18));

		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
						"Baolong EEPROM detected: bus=%d, addr=0x%02X, M=[%s], S=[%s]",
						bus, EEPROM_ADDR, m_number.c_str(), EXPECTED_S_NUMBER.c_str());

		i2c_bus = bus;
		i2c_addr = EEPROM_ADDR;
		device = "BaoLong_DualCam";
		return 0;
	}

	RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
					 "detectEeprom_baolong: no matching EEPROM found");
	return -1;
}

bool mipi_calibration::getCamCalibrationFromEeprom() {
  uint16_t i2c_addr;
  std::string device;
  auto v_i2c_bus = i2c_bus_detect(); 
  for(auto i2c_bus : v_i2c_bus) {
	if (detectEeprom_drobot(i2c_bus, device, i2c_addr) == -1) {
		continue;
	}
	if (device == "yuguang") {
		getCamCalibration_yugang(i2c_bus, i2c_addr);
	} else if (device == "union") {
		getCamCalibration_union(i2c_bus, i2c_addr);
	} else if (device == "sz_abham") {
		getCamCalibration_abham(i2c_bus, i2c_addr);
	} 
  }
  return true;
}

namespace {

#pragma pack(push, 4)
struct YuguangImuEepromData_ST {
  ImuMatrix3f_ST acc_scale_misalignment;
  float acc_noise_density;
  float acc_random_walk;
  ImuMatrix3f_ST gyro_scale_misalignment;
  float gyro_noise_density;
  float gyro_random_walk;
  float gravity;
  ImuMatrix3f_ST imu_to_left_rotation;
  float imu_to_left_translation[3];
  float left_cam_to_imu_timeshift;
  ImuMatrix3f_ST accel_to_gyro_coupling;
  ImuMatrix3f_ST gyro_to_accel_rotation;
};
#pragma pack(pop)

static_assert(sizeof(float) == 4, "Yuguang EEPROM requires float32");
static_assert(sizeof(ImuMatrix3f_ST) == 0x24,
              "Invalid IMU 3x3 matrix size");
static_assert(sizeof(YuguangImuEepromData_ST) == 0xD8,
              "Invalid Yuguang IMU EEPROM data size");

}

bool mipi_calibration::getCamCalibration_yugang(int i2c_bus, uint16_t i2c_addr) {
  std::string device;
  std::vector<char> head_buf;
  head_buf.resize(sizeof(EepromDrobotHead_ST));
  uint8_t check_value;
  if (readEeprom16(i2c_bus, i2c_addr, 0x0000, head_buf.data(),
		sizeof(EepromDrobotHead_ST)) == false) {
	return false;
  }
  const size_t check_index = sizeof(EepromDrobotHead_ST) - 1;
  check_value = static_cast<uint8_t>(head_buf[check_index]);
  uint32_t sum = 0;
  for (size_t index = 0; index < check_index; ++index) {
	sum += static_cast<uint8_t>(head_buf[index]);
  }
  const uint8_t calculated_check_value =
	  static_cast<uint8_t>((sum % 0xFFu) + 1u);
  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
	"[yuguang] header checksum stored=0x%02X calculated=0x%02X",
	static_cast<unsigned int>(check_value),
	static_cast<unsigned int>(calculated_check_value));
  if (calculated_check_value == check_value) {
	EepromDrobotHead_ST head_info{};
	std::memcpy(&head_info, head_buf.data(), sizeof(head_info));
	EepromDrobotHead_ST* head_buf_ptr = &head_info;
	struct CalibrationParams cal_param{};
	cal_param.eeprom_name_ = "yuguang";
	cal_param.i2c_bus = i2c_bus;


	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====EepromDrobotHead======" \
		"\n ----------------" \
		"\n bus: %d" \
		"\n flag: %.8s" \
		"\n camType: %d" \
		"\n cal_tpye: %d" \
		"\n ver_main: %d" \
		"\n ver_min: %d" \
		"\n angle: %d" \
		"\n d_num: %d" \
		"\n ----------------",
		i2c_bus,
		head_buf_ptr->flag,
		head_buf_ptr->camType,
		head_buf_ptr->cal_tpye,
		head_buf_ptr->ver_main,
		head_buf_ptr->ver_min,
		head_buf_ptr->angle,
		head_buf_ptr->d_num
	);

	if (head_buf_ptr->angle == 0x00) {
		cal_param.cal_rotation_ = 0.0;
	} else if (head_buf_ptr->angle == 0x01) {
		cal_param.cal_rotation_ = 90.0;
	} else if (head_buf_ptr->angle == 0x02) {
		cal_param.cal_rotation_ = 180.0;
	} else if (head_buf_ptr->angle == 0x03) {
		cal_param.cal_rotation_ = 270.0;
	}

	if ((head_buf_ptr->camType == 0x01) ||
		(head_buf_ptr->camType == 0x11)) {
		cal_param.cam_info_.resize(2);
		if (head_buf_ptr->cal_tpye == 0x01) {
			cal_param.cam_info_[0].distortion_model = cal_param.cam_info_[1].distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
		} else {
			cal_param.cam_info_[0].distortion_model = cal_param.cam_info_[1].distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
		} 
		CalDualMDInfo_ST m_d_info_l, m_d_info_r;
		CalDualRTInfo_ST r_t_info;
		if (readEeprom16(i2c_bus, i2c_addr, 0x0010, (char*)&m_d_info_l, sizeof(CalDualMDInfo_ST)) == false) {
			return false;
		}
		if (readEeprom16(i2c_bus, i2c_addr, 0x0048, (char*)&m_d_info_r, sizeof(CalDualMDInfo_ST)) == false) {
			return false;
		}
		if (readEeprom16(i2c_bus, i2c_addr, 0x008C, (char*)&r_t_info, sizeof(CalDualRTInfo_ST)) == false) {
			return false;
		}
		
		// ===================== 扩展：读取AWB硬件参数 =====================
		CONFIG_AWB_ST awb_config; // AWB参数缓冲区
		//读取EEPROM中的AWB配置参数（地址0x0134，长度为DualCamAwbCalib_ST大小）
		if (readEeprom16(i2c_bus, i2c_addr, 0x0134, (char*)&awb_config, sizeof(CONFIG_AWB_ST)) == false) {
			return false;
			// 可返回true（兼容无AWB参数的情况），或false（强制要求AWB参数）
		}
		//

		//左右相机的AWB参数
		DualCamAwbCalib_ST_L_R awb_info_l;
		DualCamAwbCalib_ST_L_R awb_info_r;
		//1.读取左相机参数
		if (readEeprom16(i2c_bus, i2c_addr, 0x0166, (char*)&awb_info_l, sizeof(DualCamAwbCalib_ST_L_R)) == false) {
			return false;
			// 可返回true（兼容无AWB参数的情况），或false（强制要求AWB参数）
		}
		//2.读取右相机参数
		if (readEeprom16(i2c_bus, i2c_addr, 0x018A, (char*)&awb_info_r, sizeof(DualCamAwbCalib_ST_L_R)) == false) {
			return false;
			// 可返回true（兼容无AWB参数的情况），或false（强制要求AWB参数）
		}

		//相机的AWB Golden参数
		DualCamAwbCalib_ST_Golden awb_info_golden;
		//3.读取相机golden参数
		if (readEeprom16(i2c_bus, i2c_addr, 0x01AE, (char*)&awb_info_golden, sizeof(DualCamAwbCalib_ST_Golden)) == false) {
			return false;
			// 可返回true（兼容无AWB参数的情况），或false（强制要求AWB参数）
		}

		// ===================== 扩展：读取LSC数据 =====================
		// LSC区域 0x0400..0x1618，校验字节 0x1619 = (Sum(0x0400..0x1618) % 0xFF) + 1
		// 布局: 9字节头部(宽/高/pattern/bls) + 8个17x17矩阵(左右目各R/GR/GB/B)
		// 读取失败或校验不匹配(含未烧录LSC的批次)时跳过，不影响其他标定数据
		// TODO(批次区分, 后补): valid/absent/corrupt 三态区分与内容合理性检查
		do {
			static const uint16_t kLscRegionAddr = 0x0400;
			static const uint32_t kLscRegionSize = 0x1219;   // 4633
			static const uint16_t kLscChecksumAddr = 0x1619;
			static const size_t kBlockSize =
				LSC_GRID_SIZE * LSC_GRID_SIZE * sizeof(uint16_t);  // 578
			static const size_t kChunkSize = 128;  // 分块读取, 避免单条I2C消息过长
			// 8个矩阵块相对区域基址0x0400的偏移
			static const size_t kOffLeftR  = 0x0009, kOffLeftGr  = 0x024B;
			static const size_t kOffLeftGb = 0x048D, kOffLeftB   = 0x06CF;
			static const size_t kOffRightR = 0x0911, kOffRightGr = 0x0B53;
			static const size_t kOffRightGb = 0x0D95, kOffRightB = 0x0FD7;

			std::vector<uint8_t> lsc_buf(kLscRegionSize);
			bool lsc_read_ok = true;

			for (size_t off = 0; off < kLscRegionSize; off += kChunkSize) {
				size_t len = std::min(kChunkSize, kLscRegionSize - off);
				if (readEeprom16(i2c_bus, i2c_addr,
						static_cast<uint16_t>(kLscRegionAddr + off),
						reinterpret_cast<char*>(lsc_buf.data() + off),
						static_cast<int>(len)) == false) {
					RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
						"[yuguang][lsc] EEPROM read failed at 0x%04X (%zu bytes), "
						"LSC skipped",
						static_cast<unsigned int>(kLscRegionAddr + off), len);
					lsc_read_ok = false;
					break;
				}
			}

			uint8_t lsc_stored_checksum = 0;
			if (lsc_read_ok &&
				readEeprom16(i2c_bus, i2c_addr, kLscChecksumAddr,
					reinterpret_cast<char*>(&lsc_stored_checksum), 1) == false) {
				RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
					"[yuguang][lsc] EEPROM checksum byte read failed at 0x%04X, "
					"LSC skipped",
					static_cast<unsigned int>(kLscChecksumAddr));
				lsc_read_ok = false;
			}

			if (lsc_read_ok) {
				uint32_t lsc_sum = 0;
				for (size_t i = 0; i < kLscRegionSize; ++i) {
					lsc_sum += lsc_buf[i];
				}
				const uint8_t lsc_calc_checksum =
					static_cast<uint8_t>((lsc_sum % 0xFFu) + 1u);
				RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
					"[yuguang][lsc] region read ok (%u bytes), "
					"checksum stored=0x%02X calculated=0x%02X",
					static_cast<unsigned int>(kLscRegionSize),
					static_cast<unsigned int>(lsc_stored_checksum),
					static_cast<unsigned int>(lsc_calc_checksum));
				if (lsc_calc_checksum != lsc_stored_checksum) {
					RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
						"[yuguang][lsc] checksum mismatch, LSC skipped "
						"(batch without LSC data or corrupted)");
					break;
				}

				// 区域头部(左右目共用)
				const uint16_t lsc_width =
					static_cast<uint16_t>(lsc_buf[0] | (lsc_buf[1] << 8));
				const uint16_t lsc_height =
					static_cast<uint16_t>(lsc_buf[2] | (lsc_buf[3] << 8));
				const uint8_t lsc_pattern = lsc_buf[4];
				if (lsc_width == 0 || lsc_height == 0) {
					RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
						"[yuguang][lsc] implausible calibration resolution %ux%u, "
						"EEPROM may be uncalibrated",
						static_cast<unsigned int>(lsc_width),
						static_cast<unsigned int>(lsc_height));
				}

				auto lsc_fill_entry = [&](const std::shared_ptr<Opt_Lsc_Config>& cfg,
						size_t off_r, size_t off_gr, size_t off_gb, size_t off_b) {
					cfg->width = lsc_width;
					cfg->height = lsc_height;
					cfg->pattern = lsc_pattern;
					cfg->bls_r = lsc_buf[5];
					cfg->bls_gr = lsc_buf[6];
					cfg->bls_gb = lsc_buf[7];
					cfg->bls_b = lsc_buf[8];
					std::memcpy(cfg->r, lsc_buf.data() + off_r, kBlockSize);
					std::memcpy(cfg->gr, lsc_buf.data() + off_gr, kBlockSize);
					std::memcpy(cfg->gb, lsc_buf.data() + off_gb, kBlockSize);
					std::memcpy(cfg->b, lsc_buf.data() + off_b, kBlockSize);
				};

				cal_param.lsc_otp_data_.resize(2);
				cal_param.lsc_otp_data_[0] = std::make_shared<Opt_Lsc_Config>();
				cal_param.lsc_otp_data_[1] = std::make_shared<Opt_Lsc_Config>();
				lsc_fill_entry(cal_param.lsc_otp_data_[0],
					kOffLeftR, kOffLeftGr, kOffLeftGb, kOffLeftB);
				lsc_fill_entry(cal_param.lsc_otp_data_[1],
					kOffRightR, kOffRightGr, kOffRightGb, kOffRightB);

				RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
					"[yuguang][lsc] LSC OTP loaded: %ux%u pattern=%u "
					"bls=[%u,%u,%u,%u]",
					static_cast<unsigned int>(lsc_width),
					static_cast<unsigned int>(lsc_height),
					static_cast<unsigned int>(lsc_pattern),
					static_cast<unsigned int>(lsc_buf[5]),
					static_cast<unsigned int>(lsc_buf[6]),
					static_cast<unsigned int>(lsc_buf[7]),
					static_cast<unsigned int>(lsc_buf[8]));
				auto lsc_log_summary = [](const char* eye, const char* ch,
						const uint16_t* data) {
					uint16_t mn = data[0], mx = data[0];
					for (size_t i = 1;
							i < static_cast<size_t>(LSC_GRID_SIZE * LSC_GRID_SIZE);
							++i) {
						if (data[i] < mn) mn = data[i];
						if (data[i] > mx) mx = data[i];
					}
					RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
						"[yuguang][lsc] %s %s: first=%u min=%u max=%u",
						eye, ch,
						static_cast<unsigned int>(data[0]),
						static_cast<unsigned int>(mn),
						static_cast<unsigned int>(mx));
				};
				lsc_log_summary("LEFT ", "R ", cal_param.lsc_otp_data_[0]->r);
				lsc_log_summary("LEFT ", "GR", cal_param.lsc_otp_data_[0]->gr);
				lsc_log_summary("LEFT ", "GB", cal_param.lsc_otp_data_[0]->gb);
				lsc_log_summary("LEFT ", "B ", cal_param.lsc_otp_data_[0]->b);
				lsc_log_summary("RIGHT", "R ", cal_param.lsc_otp_data_[1]->r);
				lsc_log_summary("RIGHT", "GR", cal_param.lsc_otp_data_[1]->gr);
				lsc_log_summary("RIGHT", "GB", cal_param.lsc_otp_data_[1]->gb);
				lsc_log_summary("RIGHT", "B ", cal_param.lsc_otp_data_[1]->b);
			}
		} while (0);
		//
		
		cal_param.awb_otp_data_.resize(2);
		cal_param.awb_otp_data_[0] = std::make_shared<Opt_Awb_Config>(); // 分配内存
		auto& left_awb_otp_data_ = cal_param.awb_otp_data_[0];
		left_awb_otp_data_->awb_config_ = awb_config;
		left_awb_otp_data_->awb_info_l_r_ = awb_info_l;
		left_awb_otp_data_->awb_info_golden_ = awb_info_golden;

		cal_param.awb_otp_data_[1] = std::make_shared<Opt_Awb_Config>();
		auto& right_awb_otp_data_ = cal_param.awb_otp_data_[1];
		right_awb_otp_data_->awb_config_ = awb_config;
		right_awb_otp_data_->awb_info_l_r_ = awb_info_r;
		right_awb_otp_data_->awb_info_golden_ = awb_info_golden;

		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====awb_info_l======" \
			"\n ----------------" \
			"\n rg_ratio_3100K: %ld" \
			"\n bg_ratio_3100K: %ld" \
			"\n rg_ratio_4000K: %ld" \
			"\n bg_ratio_4000K: %ld" \
			"\n rg_ratio_5800K: %ld" \
			"\n bg_ratio_5800K: %ld" \
			"\n ----------------",
			awb_info_l.rg_ratio_3100K,
			awb_info_l.bg_ratio_3100K,
			awb_info_l.rg_ratio_4000K,
			awb_info_l.bg_ratio_4000K,
			awb_info_l.rg_ratio_5800K,
			awb_info_l.bg_ratio_5800K
		);

		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====awb_info_r======" \
			"\n ----------------" \
			"\n rg_ratio_3100K: %ld" \
			"\n bg_ratio_3100K: %ld" \
			"\n rg_ratio_4000K: %ld" \
			"\n bg_ratio_4000K: %ld" \
			"\n rg_ratio_5800K: %ld" \
			"\n bg_ratio_5800K: %ld" \
			"\n ----------------",
			awb_info_r.rg_ratio_3100K,
			awb_info_r.bg_ratio_3100K,
			awb_info_r.rg_ratio_4000K,
			awb_info_r.bg_ratio_4000K,
			awb_info_r.rg_ratio_5800K,
			awb_info_r.bg_ratio_5800K
		);

		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====awb_info_golden======" \
			"\n ----------------" \
			"\n rg_ratio_3100K: %ld" \
			"\n bg_ratio_3100K: %ld" \
			"\n rg_ratio_4000K: %ld" \
			"\n bg_ratio_4000K: %ld" \
			"\n rg_ratio_5800K: %ld" \
			"\n bg_ratio_5800K: %ld" \
			"\n ----------------",
			awb_info_golden.rg_ratio_3100K,
			awb_info_golden.bg_ratio_3100K,
			awb_info_golden.rg_ratio_4000K,
			awb_info_golden.bg_ratio_4000K,
			awb_info_golden.rg_ratio_5800K,
			awb_info_golden.bg_ratio_5800K
		);

		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====m_d_info_l======" \
			"\n ----------------" \
			"\n width: %d" \
			"\n height: %d" \
			"\n fx: %f" \
			"\n fx: %f" \
			"\n fy: %f" \
			"\n cy: %f" \
			"\n ----------------",
			m_d_info_l.width,
			m_d_info_l.height,
			m_d_info_l.fx,
			m_d_info_l.fx,
			m_d_info_l.fy,
			m_d_info_l.cy
		);

		// printf("k1:%f\n",m_d_info_l.k1);
		// printf("k2:%f\n",m_d_info_l.k2);
		// printf("p1:%f\n",m_d_info_l.p1);
		// printf("p2:%f\n",m_d_info_l.p2);
		// printf("k3:%f\n",m_d_info_l.k3);
		// printf("k4:%f\n",m_d_info_l.k4);
		// printf("k5:%f\n",m_d_info_l.k5);
		// printf("k6:%f\n",m_d_info_l.k6);

		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====m_d_info_r======" \
			"\n ----------------" \
			"\n width: %d" \
			"\n height: %d" \
			"\n fx: %f" \
			"\n fx: %f" \
			"\n fy: %f" \
			"\n cy: %f" \
			"\n ----------------",
			m_d_info_r.width,
			m_d_info_r.height,
			m_d_info_r.fx,
			m_d_info_r.fx,
			m_d_info_r.fy,
			m_d_info_r.cy
		);

		// printf("k1:%f\n",m_d_info_r.k1);
		// printf("k2:%f\n",m_d_info_r.k2);
		// printf("p1:%f\n",m_d_info_r.p1);
		// printf("p2:%f\n",m_d_info_r.p2);
		// printf("k3:%f\n",m_d_info_r.k3);
		// printf("k4:%f\n",m_d_info_r.k4);
		// printf("k5:%f\n",m_d_info_r.k5);
		// printf("k6:%f\n",m_d_info_r.k6);





		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====r_t_info======" \
			"\n ----------------" \
			"\n r11: %f" \
			"\n r12: %f" \
			"\n r13: %f" \
			"\n r21: %f" \
			"\n r22: %f" \
			"\n r23: %f" \
			"\n r31: %f" \
			"\n r32: %f" \
			"\n r33: %f" \
			"\n tx: %f" \
			"\n ty: %f" \
			"\n tz: %f" \
			"\n ----------------",
			r_t_info.r11,
			r_t_info.r12,
			r_t_info.r13,
			r_t_info.r21,
			r_t_info.r22,
			r_t_info.r23,
			r_t_info.r31,
			r_t_info.r32,
			r_t_info.r33,
			r_t_info.tx,
			r_t_info.ty,
			r_t_info.tz
		);

		cal_param.cam_info_[0].width = m_d_info_l.width;
		cal_param.cam_info_[0].height = m_d_info_l.height;
		cal_param.cam_info_[1].width = m_d_info_r.width;
		cal_param.cam_info_[1].height = m_d_info_r.height;

		cv::Mat l_k= cv::Mat::zeros(3,3,CV_64F);
		l_k.at<double>(0,0) = m_d_info_l.fx;
		l_k.at<double>(0,2) = m_d_info_l.cx;
		l_k.at<double>(1,1) = m_d_info_l.fy;
		l_k.at<double>(1,2) = m_d_info_l.cy;
		l_k.at<double>(2,2) = 1;
		std::copy(l_k.ptr<double>(0), l_k.ptr<double>(0) + l_k.total(), cal_param.cam_info_[0].k.begin());
		
		int d_num = 8;
		if (head_buf_ptr->d_num > 0 && head_buf_ptr->d_num < 8) {
			d_num = head_buf_ptr->d_num;
		}
		cal_param.cam_info_[0].d.resize(d_num);
		for (int i = 0; i < d_num; i++) {
			cal_param.cam_info_[0].d[i] = m_d_info_l.d[i];
		}
		// cam_info_[0].d[0] = m_d_info_l.k1;
		// cam_info_[0].d[1] = m_d_info_l.k2;
		// cam_info_[0].d[2] = m_d_info_l.p1;
		// cam_info_[0].d[3] = m_d_info_l.p2;
		// cam_info_[0].d[4] = m_d_info_l.k3;
		// cam_info_[0].d[5] = m_d_info_l.k4;
		// cam_info_[0].d[6] = m_d_info_l.k5;
		// cam_info_[0].d[7] = m_d_info_l.k6;

		cv::Mat l_r_eye = cv::Mat::eye(3, 3, CV_64F);
		std::copy(l_r_eye.ptr<double>(0), l_r_eye.ptr<double>(0) + l_r_eye.total(), cal_param.cam_info_[0].r.begin());

		cv::Mat l_p_eye = cv::Mat::eye(3, 4, CV_64F);
		cv::Mat l_p = l_k * l_p_eye;
		std::copy(l_p.ptr<double>(0), l_p.ptr<double>(0) + l_p.total(), cal_param.cam_info_[0].p.begin());



		cv::Mat r_k= cv::Mat::zeros(3,3,CV_64F);
		r_k.at<double>(0,0) = m_d_info_r.fx;
		r_k.at<double>(0,2) = m_d_info_r.cx;
		r_k.at<double>(1,1) = m_d_info_r.fy;
		r_k.at<double>(1,2) = m_d_info_r.cy;
		r_k.at<double>(2,2) = 1;
		std::copy(r_k.ptr<double>(0), r_k.ptr<double>(0) + r_k.total(), cal_param.cam_info_[1].k.begin());

		// cam_info_[1].d.resize(8);
		// cam_info_[1].d[0] = m_d_info_r.k1;
		// cam_info_[1].d[1] = m_d_info_r.k2;
		// cam_info_[1].d[2] = m_d_info_r.p1;
		// cam_info_[1].d[3] = m_d_info_r.p2;
		// cam_info_[1].d[4] = m_d_info_r.k3;
		// cam_info_[1].d[5] = m_d_info_r.k4;
		// cam_info_[1].d[6] = m_d_info_r.k5;
		// cam_info_[1].d[7] = m_d_info_r.k6;

		cal_param.cam_info_[1].d.resize(d_num);
		for (int i = 0; i < d_num; i++) {
			cal_param.cam_info_[1].d[i] = m_d_info_r.d[i];
		}

		cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
		R.at<double>(0,0) = r_t_info.r11;
		R.at<double>(0,1) = r_t_info.r12;
		R.at<double>(0,2) = r_t_info.r13;
		R.at<double>(1,0) = r_t_info.r21;
		R.at<double>(1,1) = r_t_info.r22;
		R.at<double>(1,2) = r_t_info.r23;
		R.at<double>(2,0) = r_t_info.r31;
		R.at<double>(2,1) = r_t_info.r32;
		R.at<double>(2,2) = r_t_info.r33;

		cv::Mat T = cv::Mat::zeros(3, 1, CV_64F);
		T.at<double>(0,0) = r_t_info.tx;
		T.at<double>(1,0) = r_t_info.ty;
		T.at<double>(2,0) = r_t_info.tz;

		cv::Mat RT;
		cv::hconcat(R, T, RT);
		cv::Mat P = r_k * RT;
		std::copy(R.ptr<double>(0), R.ptr<double>(0) + R.total(), cal_param.cam_info_[1].r.begin());
		std::copy(P.ptr<double>(0), P.ptr<double>(0) + P.total(), cal_param.cam_info_[1].p.begin());

		if (head_buf_ptr->camType == 0x11) {
			YuguangImuEepromData_ST imu_eeprom_data{};
			uint8_t stored_imu_checksum = 0;
			if (readEeprom16(i2c_bus, i2c_addr, 0x0200,
					(char*)&imu_eeprom_data,
					sizeof(YuguangImuEepromData_ST)) == false) {
				RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
					"[yuguang][imu] EEPROM data read failed");
			} else if (readEeprom16(i2c_bus, i2c_addr, 0x02D8,
					(char*)&stored_imu_checksum,
					sizeof(stored_imu_checksum)) == false) {
				RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
					"[yuguang][imu] EEPROM checksum read failed");
			} else {
				const uint8_t* imu_raw_data =
					reinterpret_cast<const uint8_t*>(&imu_eeprom_data);
				RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
					"====Yuguang IMU raw data======");
				for (size_t offset = 0;
						offset < sizeof(YuguangImuEepromData_ST);
						offset += 12) {
					RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
						"0x%04X: %02X %02X %02X %02X %02X %02X "
						"%02X %02X %02X %02X %02X %02X",
						static_cast<unsigned int>(0x0200 + offset),
						static_cast<unsigned int>(imu_raw_data[offset]),
						static_cast<unsigned int>(imu_raw_data[offset + 1]),
						static_cast<unsigned int>(imu_raw_data[offset + 2]),
						static_cast<unsigned int>(imu_raw_data[offset + 3]),
						static_cast<unsigned int>(imu_raw_data[offset + 4]),
						static_cast<unsigned int>(imu_raw_data[offset + 5]),
						static_cast<unsigned int>(imu_raw_data[offset + 6]),
						static_cast<unsigned int>(imu_raw_data[offset + 7]),
						static_cast<unsigned int>(imu_raw_data[offset + 8]),
						static_cast<unsigned int>(imu_raw_data[offset + 9]),
						static_cast<unsigned int>(imu_raw_data[offset + 10]),
						static_cast<unsigned int>(imu_raw_data[offset + 11]));
				}

				uint32_t imu_checksum_sum = 0;
				for (size_t index = 0;
						index < sizeof(YuguangImuEepromData_ST); ++index) {
					imu_checksum_sum += imu_raw_data[index];
				}
				const uint8_t calculated_imu_checksum = static_cast<uint8_t>(
					(imu_checksum_sum % 0xFFu) + 1u);
				RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
					"[yuguang][imu] checksum address=0x%04X stored=0x%02X calculated=0x%02X",
					static_cast<unsigned int>(0x02D8),
					static_cast<unsigned int>(stored_imu_checksum),
					static_cast<unsigned int>(calculated_imu_checksum));

				if (stored_imu_checksum == calculated_imu_checksum) {
					Imu_params imu_param{};
					imu_param.acc_n_w_.n = imu_eeprom_data.acc_noise_density;
					imu_param.acc_n_w_.w = imu_eeprom_data.acc_random_walk;
					imu_param.gyro_n_w_.n = imu_eeprom_data.gyro_noise_density;
					imu_param.gyro_n_w_.w = imu_eeprom_data.gyro_random_walk;

					const ImuMatrix3f_ST& rotation =
						imu_eeprom_data.imu_to_left_rotation;
					imu_param.r_t_info_.r11 = rotation.m00;
					imu_param.r_t_info_.r12 = rotation.m01;
					imu_param.r_t_info_.r13 = rotation.m02;
					imu_param.r_t_info_.r21 = rotation.m10;
					imu_param.r_t_info_.r22 = rotation.m11;
					imu_param.r_t_info_.r23 = rotation.m12;
					imu_param.r_t_info_.r31 = rotation.m20;
					imu_param.r_t_info_.r32 = rotation.m21;
					imu_param.r_t_info_.r33 = rotation.m22;
					imu_param.r_t_info_.tx =
						imu_eeprom_data.imu_to_left_translation[0];
					imu_param.r_t_info_.ty =
						imu_eeprom_data.imu_to_left_translation[1];
					imu_param.r_t_info_.tz =
						imu_eeprom_data.imu_to_left_translation[2];
					imu_param.r_t_info_.timeshift =
						imu_eeprom_data.left_cam_to_imu_timeshift;

					imu_param.combined_calibration_.acc_scale_misalignment_ =
						imu_eeprom_data.acc_scale_misalignment;
					imu_param.combined_calibration_.gyro_scale_misalignment_ =
						imu_eeprom_data.gyro_scale_misalignment;
					imu_param.combined_calibration_.gravity_ =
						imu_eeprom_data.gravity;
					imu_param.combined_calibration_.accel_to_gyro_coupling_ =
						imu_eeprom_data.accel_to_gyro_coupling;
					imu_param.combined_calibration_.gyro_to_accel_rotation_ =
						imu_eeprom_data.gyro_to_accel_rotation;
					imu_param.combined_calibration_.valid_ = true;
					cal_param.imu_info_.push_back(imu_param);

					RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====Yuguang IMU r_t_info======" \
						"\n ----------------" \
						"\n r11: %f" \
						"\n r12: %f" \
						"\n r13: %f" \
						"\n r21: %f" \
						"\n r22: %f" \
						"\n r23: %f" \
						"\n r31: %f" \
						"\n r32: %f" \
						"\n r33: %f" \
						"\n tx: %f" \
						"\n ty: %f" \
						"\n tz: %f" \
						"\n timeshift: %f" \
						"\n ----------------",
						rotation.m00,
						rotation.m01,
						rotation.m02,
						rotation.m10,
						rotation.m11,
						rotation.m12,
						rotation.m20,
						rotation.m21,
						rotation.m22,
						imu_eeprom_data.imu_to_left_translation[0],
						imu_eeprom_data.imu_to_left_translation[1],
						imu_eeprom_data.imu_to_left_translation[2],
						imu_eeprom_data.left_cam_to_imu_timeshift
					);

					const ImuMatrix3f_ST& acc_sm =
						imu_eeprom_data.acc_scale_misalignment;
					const ImuMatrix3f_ST& gyro_sm =
						imu_eeprom_data.gyro_scale_misalignment;
					const ImuMatrix3f_ST& a2g_coup =
						imu_eeprom_data.accel_to_gyro_coupling;
					const ImuMatrix3f_ST& g2a_rot =
						imu_eeprom_data.gyro_to_accel_rotation;
					RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====Yuguang IMU acc_calib======" \
						"\n ----------------" \
						"\n acc_scale_misalignment:" \
						"\n   [0]: %f  [1]: %f  [2]: %f" \
						"\n   [3]: %f  [4]: %f  [5]: %f" \
						"\n   [6]: %f  [7]: %f  [8]: %f" \
						"\n acc_noise_density: %f" \
						"\n acc_random_walk:    %f" \
						"\n ----------------",
						acc_sm.m00, acc_sm.m01, acc_sm.m02,
						acc_sm.m10, acc_sm.m11, acc_sm.m12,
						acc_sm.m20, acc_sm.m21, acc_sm.m22,
						imu_eeprom_data.acc_noise_density,
						imu_eeprom_data.acc_random_walk
					);
					RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====Yuguang IMU gyro_calib======" \
						"\n ----------------" \
						"\n gyro_scale_misalignment:" \
						"\n   [0]: %f  [1]: %f  [2]: %f" \
						"\n   [3]: %f  [4]: %f  [5]: %f" \
						"\n   [6]: %f  [7]: %f  [8]: %f" \
						"\n gyro_noise_density: %f" \
						"\n gyro_random_walk:   %f" \
						"\n gravity:            %f" \
						"\n ----------------",
						gyro_sm.m00, gyro_sm.m01, gyro_sm.m02,
						gyro_sm.m10, gyro_sm.m11, gyro_sm.m12,
						gyro_sm.m20, gyro_sm.m21, gyro_sm.m22,
						imu_eeprom_data.gyro_noise_density,
						imu_eeprom_data.gyro_random_walk,
						imu_eeprom_data.gravity
					);
					RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====Yuguang IMU coupling======" \
						"\n ----------------" \
						"\n accel_to_gyro_coupling:" \
						"\n   [0]: %f  [1]: %f  [2]: %f" \
						"\n   [3]: %f  [4]: %f  [5]: %f" \
						"\n   [6]: %f  [7]: %f  [8]: %f" \
						"\n gyro_to_accel_rotation:" \
						"\n   [0]: %f  [1]: %f  [2]: %f" \
						"\n   [3]: %f  [4]: %f  [5]: %f" \
						"\n   [6]: %f  [7]: %f  [8]: %f" \
						"\n ----------------",
						a2g_coup.m00, a2g_coup.m01, a2g_coup.m02,
						a2g_coup.m10, a2g_coup.m11, a2g_coup.m12,
						a2g_coup.m20, a2g_coup.m21, a2g_coup.m22,
						g2a_rot.m00, g2a_rot.m01, g2a_rot.m02,
						g2a_rot.m10, g2a_rot.m11, g2a_rot.m12,
						g2a_rot.m20, g2a_rot.m21, g2a_rot.m22
					);
				} else {
					RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
						"[yuguang][imu] checksum mismatch; "
						"camera calibration remains available");
				}
			}
		}

		v_cal_params_.push_back(cal_param);
		return true;
	}
  }
  return false;
}

bool mipi_calibration::getCamCalibration_union(int i2c_bus, uint16_t i2c_addr) {
	std::string device;
	std::vector<char> head_buf;
	head_buf.resize(sizeof(EepromDrobotHead_ST));
	char check_value;
	if (readEeprom16(i2c_bus, i2c_addr, 0x0000, head_buf.data(), sizeof(EepromDrobotHead_ST)) == false) {
	  return false;
	}
	int chech_index = sizeof(EepromDrobotHead_ST) - 1;
	check_value = head_buf[chech_index];
	head_buf[chech_index] = 0;
	int sum = 0;
	
	std::for_each(head_buf.begin(), head_buf.end(), [&sum](char c) {
	  sum += static_cast<int>(c);
	});
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"sum % 255 + 1 : %x, check_value : %x", sum % 255 + 1, check_value);
	if (((sum % 255) + 1) == check_value) {
	  EepromDrobotHead_ST* head_buf_ptr = (EepromDrobotHead_ST *)head_buf.data();
  	  struct CalibrationParams cal_param;
	  cal_param.eeprom_name_ = "union";
	  cal_param.i2c_bus = i2c_bus;
	  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====EepromDrobotHead======" \
		  "\n ----------------" \
		  "\n bus: %d" \
		  "\n flag: %s" \
		  "\n camType: %d" \
		  "\n cal_tpye: %d" \
		  "\n ver_main: %d" \
		  "\n ver_min: %d" \
		  "\n angle: %d" \
		  "\n d_num: %d" \
		  "\n ----------------",
		  i2c_bus,
		  head_buf_ptr->flag,
		  head_buf_ptr->camType,
		  head_buf_ptr->cal_tpye,
		  head_buf_ptr->ver_main,
		  head_buf_ptr->ver_min,
		  head_buf_ptr->angle,
		  head_buf_ptr->d_num
	  );
  
	  if (head_buf_ptr->angle == 0x00) {
		  cal_param.cal_rotation_ = 0.0;
	  } else if (head_buf_ptr->angle == 0x01) {
		  cal_param.cal_rotation_ = 90.0;
	  } else if (head_buf_ptr->angle == 0x02) {
		  cal_param.cal_rotation_ = 180.0;
	  } else if (head_buf_ptr->angle == 0x03) {
		  cal_param.cal_rotation_ = 270.0;
	  }
  
	  if ((head_buf_ptr->camType == 0x01) || (head_buf_ptr->camType == 0x11)) {
		  cal_param.cam_info_.resize(2);
		  if (head_buf_ptr->cal_tpye == 0x01) {
			cal_param.cam_info_[0].distortion_model = cal_param.cam_info_[1].distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
		  } else {
			cal_param.cam_info_[0].distortion_model = cal_param.cam_info_[1].distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
		  } 
		  CalDualMDInfo_d_ST m_d_info_l, m_d_info_r;
		  CalDualRTInfo_d_ST r_t_info;
		  CalDualWHInfo_d_ST w_h_info,w_h_info_tmp;
		  if (readEeprom16(i2c_bus, i2c_addr, 0x0010, (char*)&w_h_info, sizeof(CalDualWHInfo_d_ST)) == false) {
			return false;
		  }
		  if (readEeprom16(i2c_bus, i2c_addr, 0x0018, (char*)&m_d_info_l, sizeof(CalDualMDInfo_d_ST)) == false) {
			  return false;
		  }
		  if (readEeprom16(i2c_bus, i2c_addr, 0x0081, (char*)&m_d_info_r, sizeof(CalDualMDInfo_d_ST)) == false) {
			  return false;
		  }
		  if (readEeprom16(i2c_bus, i2c_addr, 0x00EA, (char*)&r_t_info, sizeof(CalDualRTInfo_d_ST)) == false) {
			  return false;
		  }

		  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====w_h_info======" \
			"\n ----------------" \
			"\n width: %d" \
			"\n height: %d" \
			"\n ----------------",
			w_h_info.width,
			w_h_info.height
		  );

		  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====m_d_info_l======" \
			  "\n ----------------" \
			  "\n fx: %lf" \
			  "\n fy: %lf" \
			  "\n cx: %lf" \
			  "\n cy: %lf" \
			  "\n ----------------",
			  m_d_info_l.fx,
			  m_d_info_l.fy,
			  m_d_info_l.cx,
			  m_d_info_l.cy
		  );
  
		  // printf("k1:%f\n",m_d_info_l.k1);
		  // printf("k2:%f\n",m_d_info_l.k2);
		  // printf("p1:%f\n",m_d_info_l.p1);
		  // printf("p2:%f\n",m_d_info_l.p2);
		  // printf("k3:%f\n",m_d_info_l.k3);
		  // printf("k4:%f\n",m_d_info_l.k4);
		  // printf("k5:%f\n",m_d_info_l.k5);
		  // printf("k6:%f\n",m_d_info_l.k6);
  
		  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====m_d_info_r======" \
			  "\n ----------------" \
			  "\n fx: %lf" \
			  "\n fy: %lf" \
			  "\n cx: %lf" \
			  "\n cy: %lf" \
			  "\n ----------------",
			  m_d_info_r.fx,
			  m_d_info_r.fy,
			  m_d_info_r.cx,
			  m_d_info_r.cy
		  );
  
		  // printf("k1:%f\n",m_d_info_r.k1);
		  // printf("k2:%f\n",m_d_info_r.k2);
		  // printf("p1:%f\n",m_d_info_r.p1);
		  // printf("p2:%f\n",m_d_info_r.p2);
		  // printf("k3:%f\n",m_d_info_r.k3);
		  // printf("k4:%f\n",m_d_info_r.k4);
		  // printf("k5:%f\n",m_d_info_r.k5);
		  // printf("k6:%f\n",m_d_info_r.k6);
  
  
  
  
		  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====r_t_info======" \
			  "\n ----------------" \
			  "\n r11: %lf" \
			  "\n r12: %lf" \
			  "\n r13: %lf" \
			  "\n r21: %lf" \
			  "\n r22: %lf" \
			  "\n r23: %lf" \
			  "\n r31: %lf" \
			  "\n r32: %lf" \
			  "\n r33: %lf" \
			  "\n tx: %lf" \
			  "\n ty: %lf" \
			  "\n tz: %lf" \
			  "\n ----------------",
			  r_t_info.r11,
			  r_t_info.r12,
			  r_t_info.r13,
			  r_t_info.r21,
			  r_t_info.r22,
			  r_t_info.r23,
			  r_t_info.r31,
			  r_t_info.r32,
			  r_t_info.r33,
			  r_t_info.tx,
			  r_t_info.ty,
			  r_t_info.tz
		  );
  
		  cal_param.cam_info_[0].width = w_h_info.width;
		  cal_param.cam_info_[0].height = w_h_info.height;
		  cal_param.cam_info_[1].width = w_h_info.width;
		  cal_param.cam_info_[1].height = w_h_info.height;
  
		  cv::Mat l_k= cv::Mat::zeros(3,3,CV_64F);
		  l_k.at<double>(0,0) = m_d_info_l.fx;
		  l_k.at<double>(0,2) = m_d_info_l.cx;
		  l_k.at<double>(1,1) = m_d_info_l.fy;
		  l_k.at<double>(1,2) = m_d_info_l.cy;
		  l_k.at<double>(2,2) = 1;
		  std::copy(l_k.ptr<double>(0), l_k.ptr<double>(0) + l_k.total(), cal_param.cam_info_[0].k.begin());

		  if (head_buf_ptr->cal_tpye == 0x00 && m_d_info_l.d[2] == 0.0 && m_d_info_r.d[2] == 0.0
			&& m_d_info_l.d[3] == 0.0 && m_d_info_r.d[3] == 0.0 && m_d_info_l.d[6] == 0.0 && m_d_info_r.d[6] == 0.0
			&& m_d_info_l.d[7] == 0.0 && m_d_info_r.d[7] == 0.0) {
				// if cal_tpye is 0x00 and d[2] d[3] d[6] d[7] are all 0, then change to fisheye calibration
				RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "change to fisheye calibration");
				cal_param.cam_info_[0].distortion_model = cal_param.cam_info_[1].distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
				head_buf_ptr->cal_tpye = 0x01;
				head_buf_ptr->d_num = 4;
				cal_param.cam_info_[0].d.resize(4);
				cal_param.cam_info_[0].d[0] = m_d_info_l.d[0];
				cal_param.cam_info_[0].d[1] = m_d_info_l.d[1];
				cal_param.cam_info_[0].d[2] = m_d_info_l.d[4];
				cal_param.cam_info_[0].d[3] = m_d_info_l.d[5];
				cal_param.cam_info_[1].d.resize(4);
				cal_param.cam_info_[1].d[0] = m_d_info_r.d[0];
				cal_param.cam_info_[1].d[1] = m_d_info_r.d[1];
				cal_param.cam_info_[1].d[2] = m_d_info_r.d[4];
				cal_param.cam_info_[1].d[3] = m_d_info_r.d[5];
		  } else {
			int d_num = 8;
			if (head_buf_ptr->d_num > 0 && head_buf_ptr->d_num < 8) {
				d_num = head_buf_ptr->d_num;
			}
			cal_param.cam_info_[0].d.resize(d_num);
			for (int i = 0; i < d_num; i++) {
				cal_param.cam_info_[0].d[i] = m_d_info_l.d[i];
			}
			cal_param.cam_info_[1].d.resize(d_num);
			for (int i = 0; i < d_num; i++) {
				cal_param.cam_info_[1].d[i] = m_d_info_r.d[i];
			}
		  }
		  // cam_info_[0].d[0] = m_d_info_l.k1;
		  // cam_info_[0].d[1] = m_d_info_l.k2;
		  // cam_info_[0].d[2] = m_d_info_l.p1;
		  // cam_info_[0].d[3] = m_d_info_l.p2;
		  // cam_info_[0].d[4] = m_d_info_l.k3;
		  // cam_info_[0].d[5] = m_d_info_l.k4;
		  // cam_info_[0].d[6] = m_d_info_l.k5;
		  // cam_info_[0].d[7] = m_d_info_l.k6;
  
		  cv::Mat l_r_eye = cv::Mat::eye(3, 3, CV_64F);
		  std::copy(l_r_eye.ptr<double>(0), l_r_eye.ptr<double>(0) + l_r_eye.total(), cal_param.cam_info_[0].r.begin());
  
		  cv::Mat l_p_eye = cv::Mat::eye(3, 4, CV_64F);
		  cv::Mat l_p = l_k * l_p_eye;
		  std::copy(l_p.ptr<double>(0), l_p.ptr<double>(0) + l_p.total(), cal_param.cam_info_[0].p.begin());
  
  
  
		  cv::Mat r_k= cv::Mat::zeros(3,3,CV_64F);
		  r_k.at<double>(0,0) = m_d_info_r.fx;
		  r_k.at<double>(0,2) = m_d_info_r.cx;
		  r_k.at<double>(1,1) = m_d_info_r.fy;
		  r_k.at<double>(1,2) = m_d_info_r.cy;
		  r_k.at<double>(2,2) = 1;
		  std::copy(r_k.ptr<double>(0), r_k.ptr<double>(0) + r_k.total(), cal_param.cam_info_[1].k.begin());
  
		  // cam_info_[1].d.resize(8);
		  // cam_info_[1].d[0] = m_d_info_r.k1;
		  // cam_info_[1].d[1] = m_d_info_r.k2;
		  // cam_info_[1].d[2] = m_d_info_r.p1;
		  // cam_info_[1].d[3] = m_d_info_r.p2;
		  // cam_info_[1].d[4] = m_d_info_r.k3;
		  // cam_info_[1].d[5] = m_d_info_r.k4;
		  // cam_info_[1].d[6] = m_d_info_r.k5;
		  // cam_info_[1].d[7] = m_d_info_r.k6;
  
		  cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
		  R.at<double>(0,0) = r_t_info.r11;
		  R.at<double>(0,1) = r_t_info.r12;
		  R.at<double>(0,2) = r_t_info.r13;
		  R.at<double>(1,0) = r_t_info.r21;
		  R.at<double>(1,1) = r_t_info.r22;
		  R.at<double>(1,2) = r_t_info.r23;
		  R.at<double>(2,0) = r_t_info.r31;
		  R.at<double>(2,1) = r_t_info.r32;
		  R.at<double>(2,2) = r_t_info.r33;
  
		  cv::Mat T = cv::Mat::zeros(3, 1, CV_64F);
		  T.at<double>(0,0) = r_t_info.tx;
		  T.at<double>(0,1) = r_t_info.ty;
		  T.at<double>(0,2) = r_t_info.tz;
  
		  cv::Mat RT;
		  cv::hconcat(R, T, RT);
		  cv::Mat P = r_k * RT;
		  std::copy(R.ptr<double>(0), R.ptr<double>(0) + R.total(), cal_param.cam_info_[1].r.begin());
		  std::copy(P.ptr<double>(0), P.ptr<double>(0) + P.total(), cal_param.cam_info_[1].p.begin());

		  if (head_buf_ptr->camType == 0x11) {
			ImuMislign_ST acc_mislign, gyro_mislign;
			ImuScale_ST acc_scale, gyro_scale;
			ImuBias_ST acc_bias, gyro_bias;
			ImuNW_ST acc_n_w, gyro_n_w;
			ImuRTTimeInfo_d_ST r_t_info;
			if (readEeprom16(i2c_bus, i2c_addr, 0x0153, (char*)&acc_mislign, sizeof(ImuMislign_ST)) == false) {
			  return false;
			}
			if (readEeprom16(i2c_bus, i2c_addr, 0x0177, (char*)&acc_scale, sizeof(ImuScale_ST)) == false) {
				return false;
			}
			if (readEeprom16(i2c_bus, i2c_addr, 0x0183, (char*)&acc_bias, sizeof(ImuBias_ST)) == false) {
				return false;
			}
			if (readEeprom16(i2c_bus, i2c_addr, 0x018f, (char*)&acc_n_w, sizeof(ImuNW_ST)) == false) {
				return false;
			}
			if (readEeprom16(i2c_bus, i2c_addr, 0x0197, (char*)&gyro_mislign, sizeof(ImuMislign_ST)) == false) {
				return false;
			}
			if (readEeprom16(i2c_bus, i2c_addr, 0x01bb, (char*)&gyro_scale, sizeof(ImuScale_ST)) == false) {
				return false;
			}
			if (readEeprom16(i2c_bus, i2c_addr, 0x01c7, (char*)&gyro_bias, sizeof(ImuBias_ST)) == false) {
				return false;
			}
			if (readEeprom16(i2c_bus, i2c_addr, 0x01d3, (char*)&gyro_n_w, sizeof(ImuNW_ST)) == false) {
				return false;
			}
			if (readEeprom16(i2c_bus, i2c_addr, 0x01e0, (char*)&r_t_info, sizeof(ImuRTTimeInfo_d_ST)) == false) {
				return false;
			}

			cal_param.imu_info_.emplace_back();
			auto& imu_param = cal_param.imu_info_.back();
			imu_param.acc_mislign_ = acc_mislign;
			imu_param.acc_scale_ = acc_scale;
			imu_param.acc_bias_ = acc_bias;
			imu_param.acc_n_w_ = acc_n_w;
			imu_param.gyro_mislign_ = gyro_mislign;
			imu_param.gyro_scale_ = gyro_scale;
			imu_param.gyro_bias_ = gyro_bias;
			imu_param.gyro_n_w_ = gyro_n_w;
			imu_param.r_t_info_ = r_t_info;

			RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====acc_info======" \
				"\n ----------------" \
				"\n mislign_00: %lf" \
				"\n mislign_01: %lf" \
				"\n mislign_02: %lf" \
				"\n mislign_10: %lf" \
				"\n mislign_11: %lf" \
				"\n mislign_12: %lf" \
				"\n mislign_20: %lf" \
				"\n mislign_21: %lf" \
				"\n mislign_22: %lf" \
				"\n scale_0: %lf" \
				"\n scale_1: %lf" \
				"\n scale_2: %lf" \
				"\n bias_0: %lf" \
				"\n bias_1: %lf" \
				"\n bias_2: %lf" \
				"\n n: %lf" \
				"\n w: %lf" \			
				"\n ----------------",
				acc_mislign.m00,
				acc_mislign.m01,
				acc_mislign.m01,
				acc_mislign.m10,
				acc_mislign.m11,
				acc_mislign.m12,
				acc_mislign.m20,
				acc_mislign.m21,
				acc_mislign.m22,
				acc_scale.s0,
				acc_scale.s1,
				acc_scale.s2,
				acc_bias.b0,
				acc_bias.b1,
				acc_bias.b2,
				acc_n_w.n,
				acc_n_w.w
			);
			
			RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====gyro_info======" \
				"\n ----------------" \
				"\n mislign_00: %lf" \
				"\n mislign_01: %lf" \
				"\n mislign_02: %lf" \
				"\n mislign_10: %lf" \
				"\n mislign_11: %lf" \
				"\n mislign_12: %lf" \
				"\n mislign_20: %lf" \
				"\n mislign_21: %lf" \
				"\n mislign_22: %lf" \
				"\n scale_0: %lf" \
				"\n scale_1: %lf" \
				"\n scale_2: %lf" \
				"\n bias_0: %lf" \
				"\n bias_1: %lf" \
				"\n bias_2: %lf" \
				"\n n: %lf" \
				"\n w: %lf" \			
				"\n ----------------",
				gyro_mislign.m00,
				gyro_mislign.m01,
				gyro_mislign.m01,
				gyro_mislign.m10,
				gyro_mislign.m11,
				gyro_mislign.m12,
				gyro_mislign.m20,
				gyro_mislign.m21,
				gyro_mislign.m22,
				gyro_scale.s0,
				gyro_scale.s1,
				gyro_scale.s2,
				gyro_bias.b0,
				gyro_bias.b1,
				gyro_bias.b2,
				gyro_n_w.n,
				gyro_n_w.w
			);

			RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====IMU r_t_info======" \
				"\n ----------------" \
				"\n r11: %lf" \
				"\n r12: %lf" \
				"\n r13: %lf" \
				"\n r21: %lf" \
				"\n r22: %lf" \
				"\n r23: %lf" \
				"\n r31: %lf" \
				"\n r32: %lf" \
				"\n r33: %lf" \
				"\n tx: %lf" \
				"\n ty: %lf" \
				"\n tz: %lf" \
				"\n timeshift: %lf" \
				"\n reporject: %lf" \
				"\n ----------------",
				r_t_info.r11,
				r_t_info.r12,
				r_t_info.r13,
				r_t_info.r21,
				r_t_info.r22,
				r_t_info.r23,
				r_t_info.r31,
				r_t_info.r32,
				r_t_info.r33,
				r_t_info.tx,
				r_t_info.ty,
				r_t_info.tz,
				r_t_info.timeshift,
				r_t_info.reporject
			);
			v_cal_params_.push_back(cal_param);
		  } else {
			v_cal_params_.push_back(cal_param);
		  }
		  return true;
	  }
  
	}
	return false;
}
    

bool mipi_calibration::getCamCalibration_abham(int i2c_bus, uint16_t i2c_addr) {
	std::string device;
	std::vector<char> head_buf;
	head_buf.resize(sizeof(EepromDrobotHead_ST));
	char chech_value;
	if (readEeprom16(i2c_bus, i2c_addr, 0x0000, head_buf.data(), sizeof(EepromDrobotHead_ST)) == false) {
	  return false;
	}
	int chech_index = sizeof(EepromDrobotHead_ST) - 1;
	chech_value = head_buf[chech_index];
	head_buf[chech_index] = 0;
	int sum = 0;
	
	std::for_each(head_buf.begin(), head_buf.end(), [&sum](char c) {
	  sum += static_cast<int>(c);
	});
	if (((sum % 255) + 1) == chech_value) {
	  EepromDrobotHead_ST* head_buf_ptr = (EepromDrobotHead_ST *)head_buf.data();
  	  struct CalibrationParams cal_param;
	  cal_param.eeprom_name_ = "sz_abham";
	  cal_param.i2c_bus = i2c_bus;  
	  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====EepromDrobotHead======" \
		  "\n ----------------" \
		  "\n bus: %d" \
		  "\n flag: %s" \
		  "\n camType: %d" \
		  "\n cal_tpye: %d" \
		  "\n ver_main: %d" \
		  "\n ver_min: %d" \
		  "\n angle: %d" \
		  "\n d_num: %d" \
		  "\n ----------------",
		  i2c_bus,
		  head_buf_ptr->flag,
		  head_buf_ptr->camType,
		  head_buf_ptr->cal_tpye,
		  head_buf_ptr->ver_main,
		  head_buf_ptr->ver_min,
		  head_buf_ptr->angle,
		  head_buf_ptr->d_num
	  );
  
	  if (head_buf_ptr->angle == 0x00) {
		  cal_param.cal_rotation_ = 0.0;
	  } else if (head_buf_ptr->angle == 0x01) {
		  cal_param.cal_rotation_ = 90.0;
	  } else if (head_buf_ptr->angle == 0x02) {
		  cal_param.cal_rotation_ = 180.0;
	  } else if (head_buf_ptr->angle == 0x03) {
		  cal_param.cal_rotation_ = 270.0;
	  }
  
	  if (head_buf_ptr->camType == 0x01) {
		  cal_param.cam_info_.resize(2);
		  if (head_buf_ptr->cal_tpye == 0x01) {
			cal_param.cam_info_[0].distortion_model = cal_param.cam_info_[1].distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
		  } else {
			cal_param.cam_info_[0].distortion_model = cal_param.cam_info_[1].distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
		  } 
		  CalDualMDInfo_ST m_d_info_l, m_d_info_r;
		  CalDualRTInfo_ST r_t_info;
		  if (readEeprom16(i2c_bus, i2c_addr, 0x0010, (char*)&m_d_info_l, sizeof(CalDualMDInfo_ST)) == false) {
			  return false;
		  }
		  if (readEeprom16(i2c_bus, i2c_addr, 0x0048, (char*)&m_d_info_r, sizeof(CalDualMDInfo_ST)) == false) {
			  return false;
		  }
		  if (readEeprom16(i2c_bus, i2c_addr, 0x008C, (char*)&r_t_info, sizeof(CalDualRTInfo_ST)) == false) {
			  return false;
		  }
		  
		  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====m_d_info_l======" \
			  "\n ----------------" \
			  "\n width: %d" \
			  "\n height: %d" \
			  "\n fx: %f" \
			  "\n fx: %f" \
			  "\n fy: %f" \
			  "\n cy: %f" \
			  "\n ----------------",
			  m_d_info_l.width,
			  m_d_info_l.height,
			  m_d_info_l.fx,
			  m_d_info_l.fx,
			  m_d_info_l.fy,
			  m_d_info_l.cy
		  );
  
		  // printf("k1:%f\n",m_d_info_l.k1);
		  // printf("k2:%f\n",m_d_info_l.k2);
		  // printf("p1:%f\n",m_d_info_l.p1);
		  // printf("p2:%f\n",m_d_info_l.p2);
		  // printf("k3:%f\n",m_d_info_l.k3);
		  // printf("k4:%f\n",m_d_info_l.k4);
		  // printf("k5:%f\n",m_d_info_l.k5);
		  // printf("k6:%f\n",m_d_info_l.k6);
  
		  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====m_d_info_r======" \
			  "\n ----------------" \
			  "\n width: %d" \
			  "\n height: %d" \
			  "\n fx: %f" \
			  "\n fx: %f" \
			  "\n fy: %f" \
			  "\n cy: %f" \
			  "\n ----------------",
			  m_d_info_r.width,
			  m_d_info_r.height,
			  m_d_info_r.fx,
			  m_d_info_r.fx,
			  m_d_info_r.fy,
			  m_d_info_r.cy
		  );
  
		  // printf("k1:%f\n",m_d_info_r.k1);
		  // printf("k2:%f\n",m_d_info_r.k2);
		  // printf("p1:%f\n",m_d_info_r.p1);
		  // printf("p2:%f\n",m_d_info_r.p2);
		  // printf("k3:%f\n",m_d_info_r.k3);
		  // printf("k4:%f\n",m_d_info_r.k4);
		  // printf("k5:%f\n",m_d_info_r.k5);
		  // printf("k6:%f\n",m_d_info_r.k6);
  
  
  
  
		  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"====r_t_info======" \
			  "\n ----------------" \
			  "\n r11: %f" \
			  "\n r12: %f" \
			  "\n r13: %f" \
			  "\n r21: %f" \
			  "\n r22: %f" \
			  "\n r23: %f" \
			  "\n r31: %f" \
			  "\n r32: %f" \
			  "\n r33: %f" \
			  "\n tx: %f" \
			  "\n ty: %f" \
			  "\n tz: %f" \
			  "\n ----------------",
			  r_t_info.r11,
			  r_t_info.r12,
			  r_t_info.r13,
			  r_t_info.r21,
			  r_t_info.r22,
			  r_t_info.r23,
			  r_t_info.r31,
			  r_t_info.r32,
			  r_t_info.r33,
			  r_t_info.tx,
			  r_t_info.ty,
			  r_t_info.tz
		  );
  
		  cal_param.cam_info_[0].width = m_d_info_l.width;
		  cal_param.cam_info_[0].height = m_d_info_l.height;
		  cal_param.cam_info_[1].width = m_d_info_r.width;
		  cal_param.cam_info_[1].height = m_d_info_r.height;
  
		  cv::Mat l_k= cv::Mat::zeros(3,3,CV_64F);
		  l_k.at<double>(0,0) = m_d_info_l.fx;
		  l_k.at<double>(0,2) = m_d_info_l.cx;
		  l_k.at<double>(1,1) = m_d_info_l.fy;
		  l_k.at<double>(1,2) = m_d_info_l.cy;
		  l_k.at<double>(2,2) = 1;
		  std::copy(l_k.ptr<double>(0), l_k.ptr<double>(0) + l_k.total(), cal_param.cam_info_[0].k.begin());
		  
		  int d_num = 8;
		  if (head_buf_ptr->d_num > 0 && head_buf_ptr->d_num < 8) {
			  d_num = head_buf_ptr->d_num;
		  }
		  cal_param.cam_info_[0].d.resize(d_num);
		  for (int i = 0; i < d_num; i++) {
			  cal_param.cam_info_[0].d[i] = m_d_info_l.d[i];
		  }
		  // cam_info_[0].d[0] = m_d_info_l.k1;
		  // cam_info_[0].d[1] = m_d_info_l.k2;
		  // cam_info_[0].d[2] = m_d_info_l.p1;
		  // cam_info_[0].d[3] = m_d_info_l.p2;
		  // cam_info_[0].d[4] = m_d_info_l.k3;
		  // cam_info_[0].d[5] = m_d_info_l.k4;
		  // cam_info_[0].d[6] = m_d_info_l.k5;
		  // cam_info_[0].d[7] = m_d_info_l.k6;
  
		  cv::Mat l_r_eye = cv::Mat::eye(3, 3, CV_64F);
		  std::copy(l_r_eye.ptr<double>(0), l_r_eye.ptr<double>(0) + l_r_eye.total(), cal_param.cam_info_[0].r.begin());
  
		  cv::Mat l_p_eye = cv::Mat::eye(3, 4, CV_64F);
		  cv::Mat l_p = l_k * l_p_eye;
		  std::copy(l_p.ptr<double>(0), l_p.ptr<double>(0) + l_p.total(), cal_param.cam_info_[0].p.begin());
  
  
  
		  cv::Mat r_k= cv::Mat::zeros(3,3,CV_64F);
		  r_k.at<double>(0,0) = m_d_info_r.fx;
		  r_k.at<double>(0,2) = m_d_info_r.cx;
		  r_k.at<double>(1,1) = m_d_info_r.fy;
		  r_k.at<double>(1,2) = m_d_info_r.cy;
		  r_k.at<double>(2,2) = 1;
		  std::copy(r_k.ptr<double>(0), r_k.ptr<double>(0) + r_k.total(), cal_param.cam_info_[1].k.begin());
  
		  // cam_info_[1].d.resize(8);
		  // cam_info_[1].d[0] = m_d_info_r.k1;
		  // cam_info_[1].d[1] = m_d_info_r.k2;
		  // cam_info_[1].d[2] = m_d_info_r.p1;
		  // cam_info_[1].d[3] = m_d_info_r.p2;
		  // cam_info_[1].d[4] = m_d_info_r.k3;
		  // cam_info_[1].d[5] = m_d_info_r.k4;
		  // cam_info_[1].d[6] = m_d_info_r.k5;
		  // cam_info_[1].d[7] = m_d_info_r.k6;
  
		  cal_param.cam_info_[1].d.resize(d_num);
		  for (int i = 0; i < d_num; i++) {
			  cal_param.cam_info_[1].d[i] = m_d_info_r.d[i];
		  }
  
		  cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
		  R.at<double>(0,0) = r_t_info.r11;
		  R.at<double>(0,1) = r_t_info.r12;
		  R.at<double>(0,2) = r_t_info.r13;
		  R.at<double>(1,0) = r_t_info.r21;
		  R.at<double>(1,1) = r_t_info.r22;
		  R.at<double>(1,2) = r_t_info.r23;
		  R.at<double>(2,0) = r_t_info.r31;
		  R.at<double>(2,1) = r_t_info.r32;
		  R.at<double>(2,2) = r_t_info.r33;
  
		  cv::Mat T = cv::Mat::zeros(3, 1, CV_64F);
		  T.at<double>(0,0) = r_t_info.tx;
		  T.at<double>(0,1) = r_t_info.ty;
		  T.at<double>(0,2) = r_t_info.tz;
  
		  cv::Mat RT;
		  cv::hconcat(R, T, RT);
		  cv::Mat P = r_k * RT;
		  std::copy(R.ptr<double>(0), R.ptr<double>(0) + R.total(), cal_param.cam_info_[1].r.begin());
		  std::copy(P.ptr<double>(0), P.ptr<double>(0) + P.total(), cal_param.cam_info_[1].p.begin());
		  v_cal_params_.push_back(cal_param);
		  return true;
	  }
  
	}
	return false;
  }


bool mipi_calibration::getDualCamCalibrationFromEeprom_230ai(std::vector<sensor_msgs::msg::CameraInfo> &cam_info) {
  int i2c_bus;
  uint16_t i2c_addr;
  std::string device;
  std::vector<char> i2c_buf;
  i2c_buf.resize(sizeof(CalDualCamInfo_ST));
  char chech_value;
  if (detectEeprom_lianhe(device, i2c_bus, i2c_addr) == -1) {
	return false;
  }

  if (readEeprom16(i2c_bus, i2c_addr, 0x0022, i2c_buf.data(), sizeof(CalDualCamInfo_ST)) == false) {
	return false;
  }
  if (readEeprom16(i2c_bus, i2c_addr, 0x0022+sizeof(CalDualCamInfo_ST), &chech_value, 1) == false) {
	return false;
  }
  int sum = 0;
  std::for_each(i2c_buf.begin(), i2c_buf.end(), [&sum](char c) {
	sum += static_cast<int>(c);
  });
  if (((sum % 255) + 1) == chech_value) {
	cam_info.resize(2);
	cam_info[0].distortion_model = cam_info[1].distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
	CalDualCamInfo_ST* i2c_buf_ptr = (CalDualCamInfo_ST *)i2c_buf.data();
	int width = (i2c_buf_ptr->h_v[0] << 8) | i2c_buf_ptr->h_v[1];
	int height = (i2c_buf_ptr->h_v[2] << 8) | i2c_buf_ptr->h_v[3];

	cam_info[0].width = width;
    cam_info[0].height = height;
	cam_info[1].width = width;
    cam_info[1].height = height;

	cv::Mat l_k= cv::Mat::zeros(3,3,CV_64F);
	l_k.at<double>(0,0) = i2c_buf_ptr->fxl;
	l_k.at<double>(0,2) = i2c_buf_ptr->cxl;
	l_k.at<double>(1,1) = i2c_buf_ptr->fyl;
	l_k.at<double>(1,2) = i2c_buf_ptr->cyl;
	l_k.at<double>(2,2) = 1;
	std::copy(l_k.ptr<double>(0), l_k.ptr<double>(0) + l_k.total(), cam_info[0].k.begin());

	cam_info[0].d.resize(8);
	cam_info[0].d[0] = i2c_buf_ptr->k1l;
	cam_info[0].d[1] = i2c_buf_ptr->k2l;
	cam_info[0].d[2] = i2c_buf_ptr->p1l;
	cam_info[0].d[3] = i2c_buf_ptr->p2l;
	cam_info[0].d[4] = i2c_buf_ptr->k3l;
	cam_info[0].d[5] = i2c_buf_ptr->k4l;
	cam_info[0].d[6] = i2c_buf_ptr->k5l;
	cam_info[0].d[7] = i2c_buf_ptr->k6l;

	cv::Mat l_r_eye = cv::Mat::eye(3, 3, CV_64F);
    std::copy(l_r_eye.ptr<double>(0), l_r_eye.ptr<double>(0) + l_r_eye.total(), cam_info[0].r.begin());

	cv::Mat l_p_eye = cv::Mat::eye(3, 4, CV_64F);
    cv::Mat l_p = l_k * l_p_eye;
    std::copy(l_p.ptr<double>(0), l_p.ptr<double>(0) + l_p.total(), cam_info[0].p.begin());

	cv::Mat r_k= cv::Mat::zeros(3,3,CV_64F);
	r_k.at<double>(0,0) = i2c_buf_ptr->fxr;
	r_k.at<double>(0,2) = i2c_buf_ptr->cxr;
	r_k.at<double>(1,1) = i2c_buf_ptr->fyr;
	r_k.at<double>(1,2) = i2c_buf_ptr->cyr;
	r_k.at<double>(2,2) = 1;
	std::copy(r_k.ptr<double>(0), r_k.ptr<double>(0) + r_k.total(), cam_info[1].k.begin());

	cam_info[1].d.resize(8);
	cam_info[1].d[0] = i2c_buf_ptr->k1r;
	cam_info[1].d[1] = i2c_buf_ptr->k2r;
	cam_info[1].d[2] = i2c_buf_ptr->p1r;
	cam_info[1].d[3] = i2c_buf_ptr->p2r;
	cam_info[1].d[4] = i2c_buf_ptr->k3r;
	cam_info[1].d[5] = i2c_buf_ptr->k4r;
	cam_info[1].d[6] = i2c_buf_ptr->k5r;
	cam_info[1].d[7] = i2c_buf_ptr->k6r;

	cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
	R.at<double>(0,0) = i2c_buf_ptr->r11;
	R.at<double>(0,1) = i2c_buf_ptr->r12;
	R.at<double>(0,2) = i2c_buf_ptr->r13;
	R.at<double>(1,0) = i2c_buf_ptr->r21;
	R.at<double>(1,1) = i2c_buf_ptr->r22;
	R.at<double>(1,2) = i2c_buf_ptr->r23;
	R.at<double>(2,0) = i2c_buf_ptr->r31;
	R.at<double>(2,1) = i2c_buf_ptr->r32;
	R.at<double>(2,2) = i2c_buf_ptr->r33;

	cv::Mat T = cv::Mat::zeros(3, 1, CV_64F);
	T.at<double>(0,0) = i2c_buf_ptr->tx;
	T.at<double>(0,1) = i2c_buf_ptr->ty;
	T.at<double>(0,2) = i2c_buf_ptr->tz;

	cv::Mat RT;
	cv::hconcat(R, T, RT);
	cv::Mat P = r_k * RT;
    std::copy(R.ptr<double>(0), R.ptr<double>(0) + R.total(), cam_info[1].r.begin());
    std::copy(P.ptr<double>(0), P.ptr<double>(0) + P.total(), cam_info[1].p.begin());
	return true;
  }
  return false;
}

bool mipi_calibration::getDualCamCalibrationFromEeprom_baolong(std::vector<sensor_msgs::msg::CameraInfo> &cam_info)
{
	int i2c_bus = -1;
	uint16_t i2c_addr = 0;
	std::string device;
	if (detectEeprom_baolong(device, i2c_bus, i2c_addr) != 0)
	{
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
						 "Baolong EEPROM not detected");
		return false;
	}
	//读取标定数据 (0x55 ~ 0xE1, 141 字节)
	constexpr uint16_t CAL_DATA_ADDR = 0x0055;
	constexpr size_t CAL_DATA_SIZE = sizeof(CalDualCamInfo_BaoLong_ST);
	std::vector<uint8_t> cal_buf(CAL_DATA_SIZE);
	if (!readEeprom16(i2c_bus, i2c_addr, CAL_DATA_ADDR,
							reinterpret_cast<char *>(cal_buf.data()), CAL_DATA_SIZE))
	{
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
						 "Failed to read calibration data at 0x%04X (%zu bytes)", CAL_DATA_ADDR, CAL_DATA_SIZE);
		return false;
	}
	uint8_t crc_buf[2] = {0};
	if (!readEeprom16(i2c_bus, i2c_addr, 0x00E2,
							reinterpret_cast<char *>(crc_buf), 2))
	{
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
						 "Failed to read CRC at 0x00E2");
		return false;
	}
	uint16_t crc_expected = (static_cast<uint16_t>(crc_buf[0]) << 8) |
									static_cast<uint16_t>(crc_buf[1]);
	uint32_t sum = 0;
	for (size_t i = 0; i < CAL_DATA_SIZE; ++i)
		sum += cal_buf[i];
	uint16_t crc_actual = static_cast<uint16_t>(sum & 0xFFFF);
	if (crc_actual != crc_expected)
	{
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
						 "CRC mismatch: expected=0x%04X, actual=0x%04X", crc_expected, crc_actual);
		return false;
	}
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"), "EEPROM CRC OK: 0x%04X", crc_actual);
	const auto *cal = reinterpret_cast<const CalDualCamInfo_BaoLong_ST *>(cal_buf.data());
	if (cal->distortion_type != 0x05)
	{
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
						"Distortion_Type=0x%02X (expected 0x05), proceeding", cal->distortion_type);
	}
	
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
					"Baolong calibration:"
					"\n  Left:  fx=%.4f fy=%.4f cx=%.4f cy=%.4f"
					"\n         k1~k6=[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f] p1=%.6f p2=%.6f"
					"\n         mean_err=%.4f max_err=%.4f"
					"\n  Right: fx=%.4f fy=%.4f cx=%.4f cy=%.4f"
					"\n         k1~k6=[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f] p1=%.6f p2=%.6f"
					"\n         mean_err=%.4f max_err=%.4f"
					"\n  Ext(L->R): rot=[%.6f, %.6f, %.6f] trans=[%.6f, %.6f, %.6f]"
					"\n  measur_error=%.4f m",
					cal->fxl, cal->fyl, cal->cxl, cal->cyl,
					cal->k1l, cal->k2l, cal->k3l, cal->k4l, cal->k5l, cal->k6l, cal->p1l, cal->p2l,
					cal->mean_errl, cal->max_errl,
					cal->fxr, cal->fyr, cal->cxr, cal->cyr,
					cal->k1r, cal->k2r, cal->k3r, cal->k4r, cal->k5r, cal->k6r, cal->p1r, cal->p2r,
					cal->mean_errr, cal->max_errr,
					cal->rot_x, cal->rot_y, cal->rot_z,
					cal->tx, cal->ty, cal->tz,
					cal->measur_error);
	constexpr uint32_t IMAGE_WIDTH = 1600;
	constexpr uint32_t IMAGE_HEIGHT = 1300;
	cam_info.resize(2);
	for (int idx = 0; idx < 2; ++idx)
	{
		cam_info[idx].width = IMAGE_WIDTH;
		cam_info[idx].height = IMAGE_HEIGHT;
		cam_info[idx].distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
	}
	cam_info[0].k = {
		 static_cast<double>(cal->fxl), 0.0, static_cast<double>(cal->cxl),
		 0.0, static_cast<double>(cal->fyl), static_cast<double>(cal->cyl),
		 0.0, 0.0, 1.0};
	cam_info[0].d = {
		 static_cast<double>(cal->k1l), static_cast<double>(cal->k2l),
		 static_cast<double>(cal->p1l), static_cast<double>(cal->p2l),
		 static_cast<double>(cal->k3l), static_cast<double>(cal->k4l),
		 static_cast<double>(cal->k5l), static_cast<double>(cal->k6l)};
	cam_info[0].r = {1.0, 0.0, 0.0,
						  0.0, 1.0, 0.0,
						  0.0, 0.0, 1.0};
	cam_info[0].p = {
		 static_cast<double>(cal->fxl), 0.0, static_cast<double>(cal->cxl), 0.0,
		 0.0, static_cast<double>(cal->fyl), static_cast<double>(cal->cyl), 0.0,
		 0.0, 0.0, 1.0, 0.0};
	cam_info[1].k = {
		 static_cast<double>(cal->fxr), 0.0, static_cast<double>(cal->cxr),
		 0.0, static_cast<double>(cal->fyr), static_cast<double>(cal->cyr),
		 0.0, 0.0, 1.0};
	cam_info[1].d = {
		 static_cast<double>(cal->k1r), static_cast<double>(cal->k2r),
		 static_cast<double>(cal->p1r), static_cast<double>(cal->p2r),
		 static_cast<double>(cal->k3r), static_cast<double>(cal->k4r),
		 static_cast<double>(cal->k5r), static_cast<double>(cal->k6r)};
	// ---- 外参处理 ----
#if 0
	// EEPROM 存储的是 左→右 (L2R)，转换为 右→左 (R2L)
	cv::Mat rvec_l2r = (cv::Mat_<double>(3, 1) << static_cast<double>(cal->rot_x),
							  static_cast<double>(cal->rot_y),
							  static_cast<double>(cal->rot_z));
	cv::Mat R_l2r;
	cv::Rodrigues(rvec_l2r, R_l2r);
	cv::Mat T_l2r = (cv::Mat_<double>(3, 1) << static_cast<double>(cal->tx),
						  static_cast<double>(cal->ty),
						  static_cast<double>(cal->tz));
	// 求逆: R_r2l = R_l2r^T,  T_r2l = -R_r2l * T_l2r
	cv::Mat R_r2l = R_l2r.t();
	cv::Mat T_r2l = -R_r2l * T_l2r;
	// R (右→左 旋转矩阵)
	for (int i = 0; i < 9; ++i)
		cam_info[1].r[i] = R_r2l.at<double>(i / 3, i % 3);
	// P (3×4 投影矩阵 = K_right × [R_r2l | T_r2l])
	cv::Mat K_r = (cv::Mat_<double>(3, 3) << static_cast<double>(cal->fxr), 0.0, static_cast<double>(cal->cxr),
						0.0, static_cast<double>(cal->fyr), static_cast<double>(cal->cyr),
						0.0, 0.0, 1.0);
	cv::Mat RT;
	cv::hconcat(R_r2l, T_r2l, RT); // [R_r2l | T_r2l] → 3×4
	cv::Mat P_r = K_r * RT;			 // K × [R|T] → 3×4

#else 
	// EEPROM 存储的是 左→右 (L2R)，转换为 右→左 (R2L)
	cv::Mat rvec_l2r = (cv::Mat_<double>(3, 1) << static_cast<double>(cal->rot_x),
							  static_cast<double>(cal->rot_y),
							  static_cast<double>(cal->rot_z));

	cv::Mat T_l2r = (cv::Mat_<double>(3, 1) << static_cast<double>(cal->tx),
						  static_cast<double>(cal->ty),
						  static_cast<double>(cal->tz));

	cv::Mat K_r = (cv::Mat_<double>(3, 3) << static_cast<double>(cal->fxr), 0.0, static_cast<double>(cal->cxr),
						0.0, static_cast<double>(cal->fyr), static_cast<double>(cal->cyr),
						0.0, 0.0, 1.0);
						  
	cv::Mat R_l2r;
	cv::Rodrigues(rvec_l2r, R_l2r);

	cv::Mat RT;
	cv::hconcat(R_l2r, T_l2r, RT); // [R_r2l | T_r2l] → 3×4
	cv::Mat P_r = K_r * RT;			 // K × [R|T] → 3×4

	for (int i = 0; i < 9; ++i)
		cam_info[1].r[i] = R_l2r.at<double>(i / 3, i % 3);

	cv::Mat R_r2l = R_l2r.t();
	cv::Mat T_r2l = R_r2l * T_l2r;
#endif
	for (int i = 0; i < 12; ++i)
		cam_info[1].p[i] = P_r.at<double>(i / 4, i % 4);
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
					"Baolong dual-cam calibration loaded (bus=%d, addr=0x%02X, %ux%u)"
					"\n  Left:  fx=%.2f fy=%.2f cx=%.2f cy=%.2f"
					"\n  Right: fx=%.2f fy=%.2f cx=%.2f cy=%.2f"
					"\n  Baseline: %.4f m",
					i2c_bus, i2c_addr, IMAGE_WIDTH, IMAGE_HEIGHT,
					cal->fxl, cal->fyl, cal->cxl, cal->cyl,
					cal->fxr, cal->fyr, cal->cxr, cal->cyr,
					std::sqrt(cal->tx * cal->tx + cal->ty * cal->ty + cal->tz * cal->tz));
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
					"Baolong dual-cam calibration loaded (bus=%d, addr=0x%02X, %ux%u)"
					"\n  ---- Left Camera ----"
					"\n    K: fx=%.4f fy=%.4f cx=%.4f cy=%.4f"
					"\n    R: [identity]"
					"\n    P: [%.2f, %.2f, %.2f, %.2f;"
					"\n        %.2f, %.2f, %.2f, %.2f;"
					"\n        %.2f, %.2f, %.2f, %.2f]"
					"\n  ---- Right Camera ----"
					"\n    K: fx=%.4f fy=%.4f cx=%.4f cy=%.4f"
					"\n    R(R2L): [%.6f, %.6f, %.6f;"
					"\n             %.6f, %.6f, %.6f;"
					"\n             %.6f, %.6f, %.6f]"
					"\n    T(R2L): [%.6f, %.6f, %.6f] m"
					"\n    P: [%.2f, %.2f, %.2f, %.2f;"
					"\n        %.2f, %.2f, %.2f, %.2f;"
					"\n        %.2f, %.2f, %.2f, %.2f]"
					"\n  ---- Extrinsics ----"
					"\n    Baseline: %.4f m"
					"\n    R(L2R) rodrigues: [%.6f, %.6f, %.6f] rad"
					"\n    T(L2R): [%.6f, %.6f, %.6f] m",
					i2c_bus, i2c_addr, IMAGE_WIDTH, IMAGE_HEIGHT,
					// Left K
					cal->fxl, cal->fyl, cal->cxl, cal->cyl,
					// Left P
					cam_info[0].p[0], cam_info[0].p[1], cam_info[0].p[2], cam_info[0].p[3],
					cam_info[0].p[4], cam_info[0].p[5], cam_info[0].p[6], cam_info[0].p[7],
					cam_info[0].p[8], cam_info[0].p[9], cam_info[0].p[10], cam_info[0].p[11],
					// Right K
					cal->fxr, cal->fyr, cal->cxr, cal->cyr,
					// Right R (R2L)
					cam_info[1].r[0], cam_info[1].r[1], cam_info[1].r[2],
					cam_info[1].r[3], cam_info[1].r[4], cam_info[1].r[5],
					cam_info[1].r[6], cam_info[1].r[7], cam_info[1].r[8],
					// Right T (R2L)
					T_r2l.at<double>(0, 0), T_r2l.at<double>(1, 0), T_r2l.at<double>(2, 0),
					// Right P
					cam_info[1].p[0], cam_info[1].p[1], cam_info[1].p[2], cam_info[1].p[3],
					cam_info[1].p[4], cam_info[1].p[5], cam_info[1].p[6], cam_info[1].p[7],
					cam_info[1].p[8], cam_info[1].p[9], cam_info[1].p[10], cam_info[1].p[11],
					// Baseline
					std::sqrt(cal->tx * cal->tx + cal->ty * cal->ty + cal->tz * cal->tz),
					// L2R rodrigues (EEPROM 原始值)
					cal->rot_x, cal->rot_y, cal->rot_z,
					// L2R translation (EEPROM 原始值)
					cal->tx, cal->ty, cal->tz);
	return true;
}



bool mipi_calibration::getDualCamCalibrationIml(sensor_msgs::msg::CameraInfo &cam_info_l, sensor_msgs::msg::CameraInfo &cam_info_r,
																const std::string &file_path)
{
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "cal_file:%s", file_path.c_str());
	try
	{
		if ((file_path.length() == 0) || (file_path == "default"))
		{
			return false;
		}
		cv::FileStorage fs(file_path.c_str(), cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
							"Camera calibration file: %s is not exist!"
							"\nIf you need calibration msg, please make sure the calibration file path is correct and the calibration file exists!",
							file_path.c_str());
			return false;
		}
		cv::Mat l_k, l_d, r_k, r_d, R, T;

		int width = fs["image_width"];
		int height = fs["image_height"];
		std::string dist_model;
		fs["left_camera_matrix"] >> l_k;
		fs["left_distortion_coefficients"] >> l_d;
		fs["right_camera_matrix"] >> r_k;
		fs["right_distortion_coefficients"] >> r_d;
		fs["R"] >> R;
		fs["T"] >> T;
		fs["distortion_model"] >> dist_model;
		fs.release();
		// 检查数据类型并进行转换（如果需要）
		if (l_k.type() != CV_64F)
		{
			l_k.convertTo(l_k, CV_64F); // 转换为double类型
		}
		if (l_d.type() != CV_64F)
		{
			l_d.convertTo(l_d, CV_64F); // 转换为double类型
		}
		if (r_k.type() != CV_64F)
		{
			r_k.convertTo(r_k, CV_64F); // 转换为double类型
		}
		if (r_d.type() != CV_64F)
		{
			r_d.convertTo(r_d, CV_64F); // 转换为double类型
		}
		if (R.type() != CV_64F)
		{
			R.convertTo(R, CV_64F); // 转换为double类型
		}
		if (T.type() != CV_64F)
		{
			T.convertTo(T, CV_64F); // 转换为double类型
		}

		cam_info_r.width = cam_info_l.width = width;
		cam_info_r.height = cam_info_l.height = height;

		cam_info_l.d.resize(l_d.total());
		std::copy(l_d.ptr<double>(0), l_d.ptr<double>(0) + l_d.total(), cam_info_l.d.begin());
		std::copy(l_k.ptr<double>(0), l_k.ptr<double>(0) + l_k.total(), cam_info_l.k.begin());

		cam_info_r.d.resize(r_d.total());
		std::copy(r_d.ptr<double>(0), r_d.ptr<double>(0) + r_d.total(), cam_info_r.d.begin());
		std::copy(r_k.ptr<double>(0), r_k.ptr<double>(0) + r_k.total(), cam_info_r.k.begin());

		cv::Mat l_r_eye = cv::Mat::eye(3, 3, CV_64F);
		std::copy(l_r_eye.ptr<double>(0), l_r_eye.ptr<double>(0) + l_r_eye.total(), cam_info_l.r.begin());

		cv::Mat l_p_eye = cv::Mat::eye(3, 4, CV_64F);
		cv::Mat l_p = l_k * l_p_eye;
		std::copy(l_p.ptr<double>(0), l_p.ptr<double>(0) + l_p.total(), cam_info_l.p.begin());

		cv::Mat RT = cv::Mat::zeros(3, 4, CV_64F);
		R.copyTo(RT(cv::Rect(0, 0, 3, 3)));
		T.reshape(1).copyTo(RT.col(3));
		cv::Mat P = r_k * RT;
		std::copy(R.ptr<double>(0), R.ptr<double>(0) + R.total(), cam_info_r.r.begin());
		std::copy(P.ptr<double>(0), P.ptr<double>(0) + P.total(), cam_info_r.p.begin());
		if (dist_model == "fisheye")
		{
			cam_info_l.distortion_model = cam_info_r.distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
		}
		else
		{
			cam_info_l.distortion_model = cam_info_r.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
			RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
							"Camera calibration file did not specify distortion model, "
							"assuming plumb bob");
		}

		// fs.release();
		return true;
	}
	catch (cv::Exception &e)
	{
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
						 "Unable to parse camera calibration file normally:%s",
						 e.what());
		return false;
	}
}

bool mipi_calibration::getCamCalibrationIml_single(sensor_msgs::msg::CameraInfo &cam_info,
																	const std::string &file_path)
{
	try
	{
		std::string cal_file;
		if ((file_path.length() == 0) || (file_path == "default"))
		{
			// MIPI_CAP_INFO_ST cap_info;
			// mipiCap_ptr_->getCapInfo(cap_info);
			// std::string sensor_name = cap_info.sensor_type;
			// std::transform(sensor_name.begin(), sensor_name.end(), sensor_name.begin(), [](unsigned char c)
			// 					{ return std::toupper(c); });
			// cal_file = cap_info.config_path + "/" + sensor_name + "_calibration.yaml";
			return false;
		}
		else
		{
			cal_file = file_path;
		}
		std::string camera_name;
		std::ifstream fin(cal_file.c_str());
		if (!fin)
		{
			RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
							"Camera calibration file: %s is not exist!"
							"\nIf you need calibration msg, please make sure the calibration file path is correct and the calibration file exists!",
							cal_file.c_str());
			return false;
		}
		YAML::Node calibration_doc = YAML::Load(fin);
		if (calibration_doc["camera_name"])
		{
			camera_name = calibration_doc["camera_name"].as<std::string>();
		}
		else
		{
			camera_name = "unknown";
		}
		cam_info.width = calibration_doc["image_width"].as<int>();
		cam_info.height = calibration_doc["image_height"].as<int>();

		const YAML::Node &camera_matrix = calibration_doc["camera_matrix"];
		const YAML::Node &camera_matrix_data = camera_matrix["data"];
		for (int i = 0; i < 9; i++)
		{
			cam_info.k[i] = camera_matrix_data[i].as<double>();
		}
		const YAML::Node &rectification_matrix =
			 calibration_doc["rectification_matrix"];
		const YAML::Node &rectification_matrix_data = rectification_matrix["data"];
		for (int i = 0; i < 9; i++)
		{
			cam_info.r[i] = rectification_matrix_data[i].as<double>();
		}
		const YAML::Node &projection_matrix = calibration_doc["projection_matrix"];
		const YAML::Node &projection_matrix_data = projection_matrix["data"];
		for (int i = 0; i < 12; i++)
		{
			cam_info.p[i] = projection_matrix_data[i].as<double>();
		}

		if (calibration_doc["distortion_model"])
		{
			cam_info.distortion_model =
				 calibration_doc["distortion_model"].as<std::string>();
		}
		else
		{
			cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
			RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
							"Camera calibration file did not specify distortion model, "
							"assuming plumb bob");
		}
		const YAML::Node &distortion_coefficients =
			 calibration_doc["distortion_coefficients"];
		int d_rows, d_cols;
		d_rows = distortion_coefficients["rows"].as<int>();
		d_cols = distortion_coefficients["cols"].as<int>();
		const YAML::Node &distortion_coefficients_data =
			 distortion_coefficients["data"];
		cam_info.d.resize(d_rows * d_cols);
		for (int i = 0; i < d_rows * d_cols; ++i)
		{
			cam_info.d[i] = distortion_coefficients_data[i].as<double>();
		}
		RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
						"[getCamCalibration]->parse calibration file successfully");
		return true;
	}
	catch (YAML::Exception &e)
	{
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
						 "Unable to parse camera calibration file normally:%s",
						 e.what());
		return false;
	}
}

}










