#include "hobot_mipi_calibration.hpp"
#include "hobot_mipi_comm.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "opencv2/opencv.hpp"

#include <string>
#include <fstream>
#include <iostream>
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

bool mipi_calibration::getCamCalibration_yugang(int i2c_bus, uint16_t i2c_addr) {
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
	cal_param.eeprom_name_ = "yuguang";
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
		if (head_buf_ptr->d_num <= 0 && head_buf_ptr->d_num >=4) {
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

		// ----------------------------------awb -----------------------------------
		//左目
		// cal_param.awb_otp_data_.resize(2);
		// cal_param.awb_otp_data_[0] = std::make_shared<sensor_otp_t_>(); // 分配内存
		// auto& left_awb_otp_data_ = cal_param.awb_otp_data_[0];
		// left_awb_otp_data_->otp_awb_enable = 1;
		// left_awb_otp_data_->awb_ct_num = 3;
		// left_awb_otp_data_->awb_data[0].color_temperature = COLOR_TEMPERATURE_3100K;
		// left_awb_otp_data_->awb_data[0].rg_ratio = awb_info_l.rg_ratio_3100K;
		// left_awb_otp_data_->awb_data[0].bg_ratio = awb_info_l.bg_ratio_3100K;
		// left_awb_otp_data_->awb_golden_data[0].rg_ratio = awb_info_golden.rg_ratio_3100K;
		// left_awb_otp_data_->awb_golden_data[0].bg_ratio = awb_info_golden.bg_ratio_3100K;
		// left_awb_otp_data_->awb_data[1].color_temperature = COLOR_TEMPERATURE_4000K;
		// left_awb_otp_data_->awb_data[1].rg_ratio = awb_info_l.rg_ratio_4000K;
		// left_awb_otp_data_->awb_data[1].bg_ratio = awb_info_l.bg_ratio_4000K;
		// left_awb_otp_data_->awb_golden_data[1].rg_ratio = awb_info_golden.rg_ratio_4000K;
		// left_awb_otp_data_->awb_golden_data[1].bg_ratio = awb_info_golden.bg_ratio_4000K;
		// left_awb_otp_data_->awb_data[2].color_temperature = COLOR_TEMPERATURE_5800K;
		// left_awb_otp_data_->awb_data[2].rg_ratio = awb_info_l.rg_ratio_5800K;
		// left_awb_otp_data_->awb_data[2].bg_ratio = awb_info_l.bg_ratio_5800K;
		// left_awb_otp_data_->awb_golden_data[2].rg_ratio = awb_info_golden.rg_ratio_5800K;
		// left_awb_otp_data_->awb_golden_data[2].bg_ratio = awb_info_golden.bg_ratio_5800K;

		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "=> ================== left awb otp data ==================");
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_golden_data[0].rg_ratio: %ld", left_awb_otp_data_->awb_golden_data[0].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_golden_data[0].bg_ratio: %ld", left_awb_otp_data_->awb_golden_data[0].bg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_golden_data[1].rg_ratio: %ld", left_awb_otp_data_->awb_golden_data[1].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_golden_data[1].bg_ratio: %ld", left_awb_otp_data_->awb_golden_data[1].bg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_golden_data[2].rg_ratio: %ld", left_awb_otp_data_->awb_golden_data[2].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_golden_data[2].bg_ratio: %ld", left_awb_otp_data_->awb_golden_data[2].bg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_data[0].rg_ratio: %ld", left_awb_otp_data_->awb_data[0].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_data[0].bg_ratio: %ld", left_awb_otp_data_->awb_data[0].bg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_data[1].rg_ratio: %ld", left_awb_otp_data_->awb_data[1].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_data[1].bg_ratio: %ld", left_awb_otp_data_->awb_data[1].bg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_data[2].rg_ratio: %ld", left_awb_otp_data_->awb_data[2].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "left_awb_otp_data_.awb_data[2].bg_ratio: %ld", left_awb_otp_data_->awb_data[2].bg_ratio);

		// left_awb_otp_data_->awb_data[0].r = awb_config.bls_r;
		// left_awb_otp_data_->awb_data[0].gr = awb_config.bls_gr;
		// left_awb_otp_data_->awb_data[0].gb = awb_config.bls_gb;
		// left_awb_otp_data_->awb_data[0].b = awb_config.bls_b;

		// //右目
		// cal_param.awb_otp_data_[1] = std::make_shared<sensor_otp_t_>();
		// auto& right_awb_otp_data_ = cal_param.awb_otp_data_[1];
		// right_awb_otp_data_->otp_awb_enable = 1;
		// right_awb_otp_data_->awb_ct_num = 3;
		// right_awb_otp_data_->awb_data[0].color_temperature = COLOR_TEMPERATURE_3100K;
		// right_awb_otp_data_->awb_data[0].rg_ratio = awb_info_r.rg_ratio_3100K;
		// right_awb_otp_data_->awb_data[0].bg_ratio = awb_info_r.bg_ratio_3100K;
		// right_awb_otp_data_->awb_golden_data[0].rg_ratio = awb_info_golden.rg_ratio_3100K;
		// right_awb_otp_data_->awb_golden_data[0].bg_ratio = awb_info_golden.bg_ratio_3100K;
		// right_awb_otp_data_->awb_data[1].color_temperature = COLOR_TEMPERATURE_4000K;
		// right_awb_otp_data_->awb_data[1].rg_ratio = awb_info_r.rg_ratio_4000K;
		// right_awb_otp_data_->awb_data[1].bg_ratio = awb_info_r.bg_ratio_4000K;
		// right_awb_otp_data_->awb_golden_data[1].rg_ratio = awb_info_golden.rg_ratio_4000K;
		// right_awb_otp_data_->awb_golden_data[1].bg_ratio = awb_info_golden.bg_ratio_4000K;
		// right_awb_otp_data_->awb_data[2].color_temperature = COLOR_TEMPERATURE_5800K;
		// right_awb_otp_data_->awb_data[2].rg_ratio = awb_info_r.rg_ratio_5800K;
		// right_awb_otp_data_->awb_data[2].bg_ratio = awb_info_r.bg_ratio_5800K;
		// right_awb_otp_data_->awb_golden_data[2].rg_ratio = awb_info_golden.rg_ratio_5800K;
		// right_awb_otp_data_->awb_golden_data[2].bg_ratio = awb_info_golden.bg_ratio_5800K;

		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "=> ================== right awb otp data ==================");
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_golden_data[0].rg_ratio: %ld", right_awb_otp_data_->awb_golden_data[0].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_golden_data[0].bg_ratio: %ld", right_awb_otp_data_->awb_golden_data[0].bg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_golden_data[1].rg_ratio: %ld", right_awb_otp_data_->awb_golden_data[1].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_golden_data[1].bg_ratio: %ld", right_awb_otp_data_->awb_golden_data[1].bg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_golden_data[2].rg_ratio: %ld", right_awb_otp_data_->awb_golden_data[2].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_golden_data[2].bg_ratio: %ld", right_awb_otp_data_->awb_golden_data[2].bg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_data[0].rg_ratio: %ld", right_awb_otp_data_->awb_data[0].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_data[0].bg_ratio: %ld", right_awb_otp_data_->awb_data[0].bg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_data[1].rg_ratio: %ld", right_awb_otp_data_->awb_data[1].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_data[1].bg_ratio: %ld", right_awb_otp_data_->awb_data[1].bg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_data[2].rg_ratio: %ld", right_awb_otp_data_->awb_data[2].rg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "right_awb_otp_data_.awb_data[2].bg_ratio: %ld", right_awb_otp_data_->awb_data[2].bg_ratio);
		// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "=> ========================================================");

		// right_awb_otp_data_->awb_data[0].r = awb_config.bls_r;
		// right_awb_otp_data_->awb_data[0].gr = awb_config.bls_gr;
		// right_awb_otp_data_->awb_data[0].gb = awb_config.bls_gb;
		// right_awb_otp_data_->awb_data[0].b = awb_config.bls_b;
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
			if (head_buf_ptr->d_num <= 0 && head_buf_ptr->d_num >=4) {
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
		  if (head_buf_ptr->d_num <= 0 && head_buf_ptr->d_num >=4) {
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

}










