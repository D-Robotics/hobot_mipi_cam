// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cap_iml.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "opencv2/opencv.hpp"

#include "hobot_mipi_calibration.hpp"

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
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#include <sys/select.h>

#include "hb_media_codec.h"
#include "hb_media_error.h"

#include <rclcpp/rclcpp.hpp>
#include <json/json.h>

#define ERR_CON_EQ(ret, a) do {\
		if ((ret) != (a)) {\
			RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "%s(%d) failed, ret %d\n", __func__, __LINE__, (int32_t)(ret));\
			return (ret);\
		}\
	} while(0)\


#define ERR_CON_NE(ret, a) do {\
		if ((ret) == (a)) {\
			RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "%s(%d) failed, ret %ld\n", __func__, __LINE__, (ret));\
			return (ret);\
		}\
	} while(0)\


namespace mipi_cam {

int HobotMipiCapIml::initEnv() {
  std::vector<int> mipi_hosts;
  std::vector<int> mipi_bus;
  if (analysis_board_config ()) {
    if (board_config_m_.size() > 0) {
      for (auto board : board_config_m_) {
        mipi_hosts.push_back(board.first);
        mipi_bus.push_back(board.second.i2c_bus);
      }
    } else {
      mipi_hosts = {0,1,2,3};
    }
  } else {
    mipi_hosts = {0,1,2,3};
  }

  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"), "this board support mipi:");
  for (auto host : mipi_hosts) {
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"), "host %d", host);
  }


  listMipiHost(mipi_hosts, mipi_started_, mipi_stoped_);

  if (mipi_stoped_.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "There are no available host.\n");
    return -1;
  }
  if (board_config_m_.size() == 0) {
    if (mipi_started_.size() > 0) {
      return -1;
    }
  }

  std::ofstream qos_file("/sys/devices/platform/soc/20510100.dw230_gdc_qos/read_priority_qos_ctrl/priority");
  if (qos_file.is_open()) {
	qos_file << "0";  // 写入目标值
	qos_file.close();
  }

  return 0;
}

int HobotMipiCapIml::init(MIPI_CAP_INFO_ST &info) {
  int ret = 0;
  cap_info_ = info;
  std::vector<int> sensor_v;
  std::vector<int> host_v;
  std::vector<mipi_host_info_t> v_host_info;
  std::vector<mipi_host_info_t> v_host_info_detect;
  int sensor_index = 0;
  bool sensor_flag = false;
  int sensor_index2 = 0;
  bool sensor_flag2 = false;
  mipi_host_info_t host_info;
  hb_mem_module_open();
  vp_show_sensors_list();
  if (cap_info_.device_mode_.compare("dual") == 0) {
	for (auto i : mipi_stoped_) {
		ret = vp_sensor_detect_2(i, &host_info);
		if (ret == 0) {
			v_host_info_detect.push_back(host_info);
		}
	}
	if (v_host_info_detect.size() < 2) {
		RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
       		"The detected sensors are 2 less than expected.\n");
		return -1;
	}

	if (cap_info_.channel_ == cap_info_.channel2_) {
		for(auto& host : v_host_info_detect) {
			v_host_info.push_back(host);
		}
	} else {
		for(int k = 0; k < v_host_info_detect.size(); k++) {
			if (v_host_info_detect[k].host_num == cap_info_.channel_) {
				sensor_index = k;
				sensor_flag = true;
			} else if (v_host_info_detect[k].host_num == cap_info_.channel2_) {
				sensor_index2 = k;
				sensor_flag2 = true;
			}
		}
		if ((sensor_flag == true) && (sensor_flag2 == true)) {
			v_host_info.push_back(v_host_info_detect[sensor_index]);
			v_host_info.push_back(v_host_info_detect[sensor_index2]);
		} else {
			for(auto& host : v_host_info_detect) {
				v_host_info.push_back(host);
			}
		}
	}

	pipe_contex.resize(2);
	pipe_contex[0].cap_info_ = &cap_info_;
	pipe_contex[1].cap_info_ = &cap_info_;
	memcpy(&pipe_contex[1].sensor_config, vp_sensor_config_list[v_host_info[1].sensor_index], sizeof(vp_sensor_config_t));
	ret = vp_sensor_fixed_mipi_host_1(v_host_info[1].host_num, &pipe_contex[1].sensor_config, &pipe_contex[1].csi_config);
	ERR_CON_EQ(ret, 0);
	gdc_bin_buf_.clear();

	vp_sensor_config_t *sensor_cof = &pipe_contex[1].sensor_config;

	if ((cap_info_.stream_mode_ == 1) && (cap_info_.rotation_ != 0)) {
		vp_sensor_config_t *sensor_conf = &pipe_contex[1].sensor_config;
		int out_width = sensor_cof->isp_ichn_attr->width;
		int out_height = sensor_cof->isp_ichn_attr->height;
		if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
			out_width = sensor_cof->isp_ichn_attr->height;
			out_height = sensor_cof->isp_ichn_attr->width;
		}		
		auto gdc_bin = gen_gdc_bin_rotation(sensor_conf->isp_ichn_attr->width, sensor_conf->isp_ichn_attr->height, out_width, out_height, cap_info_.rotation_);
		if (gdc_bin) {
			gdc_bin_buf_.push_back(gdc_bin);
			pipe_contex[0].gdc_bin_r = gdc_bin;
			pipe_contex[1].gdc_bin_r = gdc_bin;
		}
	}
	mipi_calibration& calibration_instance = mipi_calibration::GetInstance();
    if (cam_info_.size() != 2) {
		auto cal_params = mipi_calibration::GetInstance().getCalibrationParams();
		if (cal_params.size() >= 1) {
			cam_info_ = cal_params[0].cam_info_;
			cap_info_.cal_rotation_ = cal_params[0].cal_rotation_;
			eeprom_name_ = cal_params[0].eeprom_name_;
			imu_info_ = cal_params[0].imu_info_;
			awb_otp_data_ = cal_params[0].awb_otp_data_;
		}
	}
	if((cam_info_.size() != 2) && (strcasecmp(sensor_cof->sensor_name, "sc230ai-30fps") == 0)) {
		calibration_instance.getDualCamCalibrationFromEeprom_230ai(cam_info_);
	}

	if (cap_info_.gdc_enable_) {
		std::vector<std::shared_ptr<GdcBinBuf_ST>> gdc_bin;
		if (cap_info_.stream_mode_ == 1) {
			vp_sensor_config_t *sensor_conf = &pipe_contex[1].sensor_config;
			int input_width = sensor_cof->isp_ichn_attr->width;
			int input_height = sensor_cof->isp_ichn_attr->height;
			if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
				input_width = sensor_cof->isp_ichn_attr->height;
				input_height = sensor_cof->isp_ichn_attr->width;
			}		
			gdc_bin = gen_gdc_bin_stereo(input_width, input_height, cap_info_.width,
				cap_info_.height, cam_info_, cal_cam_info_, cap_info_.rotation_, cap_info_.cal_rotation_,
				cap_info_.cal_alpha_, !gdc_bin_buf_.empty());
		} else {
			int out_width = sensor_cof->isp_ichn_attr->width;
			int out_height = sensor_cof->isp_ichn_attr->height;
			if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
				out_width = sensor_cof->isp_ichn_attr->height;
				out_height = sensor_cof->isp_ichn_attr->width;
			}
			if (((cap_info_.width > cap_info_.height) && (out_width < out_height)) ||
			((cap_info_.width < cap_info_.height) && (out_width > out_height))) {
				out_width = cap_info_.width;
				out_height = cap_info_.height;
				pipe_contex[0].gdc_resize_enable = true;
				pipe_contex[1].gdc_resize_enable = true;
			}
			gdc_bin = gen_gdc_bin_stereo(sensor_cof->isp_ichn_attr->width, sensor_cof->isp_ichn_attr->height, out_width,
				out_height, cam_info_, cal_cam_info_, cap_info_.rotation_, cap_info_.cal_rotation_,
				cap_info_.cal_alpha_);
		}
		if (gdc_bin.size() == 2) {
			gdc_bin_buf_.push_back(gdc_bin[0]);
			gdc_bin_buf_.push_back(gdc_bin[1]);
			pipe_contex[0].gdc_bin = gdc_bin[0];
			pipe_contex[1].gdc_bin = gdc_bin[1];
		}

	} else if (cam_info_.size() == 2) {
		cal_cam_info_.push_back(cam_info_[0]);
		cal_cam_info_.push_back(cam_info_[1]);
	}
	if ((cap_info_.stream_mode_ != 1) && (cap_info_.rotation_ != 0) && (gdc_bin_buf_.size() == 0)) {
		vp_sensor_config_t *sensor_conf = &pipe_contex[1].sensor_config;
		auto gdc_bin = gen_gdc_bin_rotation(sensor_conf->isp_ichn_attr->width, sensor_conf->isp_ichn_attr->height, cap_info_.width, cap_info_.height, cap_info_.rotation_);
		if (gdc_bin) {
			gdc_bin_buf_.push_back(gdc_bin);
			pipe_contex[0].gdc_bin_r = gdc_bin;
			pipe_contex[1].gdc_bin_r = gdc_bin;
		}
	}
	if ((eeprom_name_ == "yuguang") && (strcasecmp(pipe_contex[1].sensor_config.sensor_name, "sc132gs-1280p") == 0)) {
		std::string sensor_tuning = "/usr/hobot/bin/sc132gs_tuning_yg.json";
		rcpputils::fs::path file_path = sensor_tuning;
		if (rcpputils::fs::exists(file_path)) {
			std::strcpy(pipe_contex[1].sensor_config.camera_config->calib_lname ,"sc132gs_tuning_yg.json");
		}	
	}

	if (awb_otp_data_.size() == 2) {
		pipe_contex[0].awb_otp_data = awb_otp_data_[0];
		pipe_contex[1].awb_otp_data = awb_otp_data_[1];
	}
	ret = create_and_run_vflow(&pipe_contex[1]);
	ERR_CON_EQ(ret, 0);
	//copy_config(&pipe_contex[1].sensor_config, vp_sensor_config_list[v_host_info[1].sensor_index]);
	memcpy(&pipe_contex[0].sensor_config, vp_sensor_config_list[v_host_info[0].sensor_index], sizeof(vp_sensor_config_t));
	ret = vp_sensor_fixed_mipi_host_1(v_host_info[0].host_num, &pipe_contex[0].sensor_config, &pipe_contex[0].csi_config);
	ERR_CON_EQ(ret, 0);
	if ((eeprom_name_ == "yuguang") && (strcasecmp(pipe_contex[0].sensor_config.sensor_name, "sc132gs-1280p") == 0)) {
		std::string sensor_tuning = "/usr/hobot/bin/sc132gs_tuning_yg.json";
		rcpputils::fs::path file_path = sensor_tuning;
		if (rcpputils::fs::exists(file_path)) {
			std::strcpy(pipe_contex[0].sensor_config.camera_config->calib_lname ,"sc132gs_tuning_yg.json");
		}
	}
	ret = create_and_run_vflow(&pipe_contex[0]);
	ERR_CON_EQ(ret, 0);
	if ((cap_info_.dual_combine_ == 1) || (cap_info_.dual_combine_ == 2)) {
		combine_flag_ = true;
	}
  } else {
	bool deteced_flag = false;
	int sensor_count = vp_get_sensors_list_number();
	for (int i = 0; i < sensor_count; i++) {
		auto sensor_info = vp_sensor_config_list[i];
		if (cap_info_.sensor_type.compare(sensor_info->sensor_name) == 0) {
			pipe_contex.resize(1);
			pipe_contex[0].cap_info_ = &cap_info_;
			memcpy(&pipe_contex[0].sensor_config, sensor_info, sizeof(vp_sensor_config_t));
			ret = vp_sensor_fixed_mipi_host_1(cap_info_.channel_, &pipe_contex[0].sensor_config, &pipe_contex[0].csi_config);
			ERR_CON_EQ(ret, 0);
			deteced_flag = true;
			break;
		}
	}
	if (deteced_flag == false) {
		for (auto i : mipi_stoped_) {
			ret = vp_sensor_detect_2(i, &host_info);
			if (ret == 0) {
				v_host_info_detect.push_back(host_info);
			}
		}
		if (v_host_info_detect.size() < 1) {
			RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
				"The detected sensors are 1 less than expected.\n");
			return -1;
		}

		for(auto& host : v_host_info_detect) {
			if (host.host_num == cap_info_.channel_) {
				v_host_info.push_back(host);
				sensor_flag = true;
				break;
			}
		}
		if (sensor_flag == false) {
			v_host_info.push_back(v_host_info_detect[0]);
		}

		pipe_contex.resize(1);
		pipe_contex[0].cap_info_ = &cap_info_;
		memcpy(&pipe_contex[0].sensor_config, vp_sensor_config_list[v_host_info[0].sensor_index], sizeof(vp_sensor_config_t));
		ret = vp_sensor_fixed_mipi_host_1(v_host_info[0].host_num, &pipe_contex[0].sensor_config, &pipe_contex[0].csi_config);
		ERR_CON_EQ(ret, 0);
	}
	gdc_bin_buf_.clear();
	if ((cap_info_.stream_mode_ == 1) && (cap_info_.rotation_ != 0)) {
		vp_sensor_config_t *sensor_conf = &pipe_contex[0].sensor_config;
		auto gdc_bin = gen_gdc_bin_rotation(sensor_conf->isp_ichn_attr->width, sensor_conf->isp_ichn_attr->height, cap_info_.width, cap_info_.height, cap_info_.rotation_);
		if (gdc_bin) {
			gdc_bin_buf_.push_back(gdc_bin);
			pipe_contex[0].gdc_bin_r = gdc_bin;
		}
	}
	if (cap_info_.gdc_enable_) {
		vp_sensor_config_t *sensor_cof = &pipe_contex[0].sensor_config;
		auto gdb_bin_fig = get_gdc_bin(pipe_contex[0].cap_info_->gdc_bin_file_);
		if (gdb_bin_fig) {
			gdc_bin_buf_.push_back(gdb_bin_fig);
			pipe_contex[0].gdc_bin = gdb_bin_fig;
		}
		else if (cam_info_.size() > 0) {
			sensor_msgs::msg::CameraInfo cal_cam_info;
			std::shared_ptr<GdcBinBuf_ST> gdc_bin = nullptr;
			if (cap_info_.stream_mode_ == 1) {
				gdc_bin = gen_gdc_bin(cap_info_.width, cap_info_.height, cap_info_.width, cap_info_.height,
					&cam_info_[0], &cal_cam_info, cap_info_.rotation_, cap_info_.cal_rotation_, cap_info_.cal_alpha_, !gdc_bin_buf_.empty());
			} else {
				int out_width = sensor_cof->isp_ichn_attr->width;
				int out_height = sensor_cof->isp_ichn_attr->height;
				if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
					out_width = sensor_cof->isp_ichn_attr->height;
					out_height = sensor_cof->isp_ichn_attr->width;
				}
				if (((cap_info_.width > cap_info_.height) && (out_width < out_height)) ||
				((cap_info_.width < cap_info_.height) && (out_width > out_height))) {
					out_width = cap_info_.width;
					out_height = cap_info_.height;
					pipe_contex[0].gdc_resize_enable = true;
				}
				gdc_bin = gen_gdc_bin(sensor_cof->isp_ichn_attr->width, sensor_cof->isp_ichn_attr->height, out_width, out_height,
					&cam_info_[0], &cal_cam_info, cap_info_.rotation_, cap_info_.cal_rotation_, cap_info_.cal_alpha_);
			}
			//auto gdc_bin = gen_gdc_bin_json("./gdc_bin_custom_config.json");
			if (gdc_bin) {
				gdc_bin_buf_.push_back(gdc_bin);
				pipe_contex[0].gdc_bin = gdc_bin;
				cal_cam_info_.push_back(cal_cam_info);
			}
		}
	}
	if ((cap_info_.stream_mode_ != 1) && (cap_info_.rotation_ != 0) && (gdc_bin_buf_.size() == 0)) {
		vp_sensor_config_t *sensor_conf = &pipe_contex[0].sensor_config;
		auto gdc_bin = gen_gdc_bin_rotation(sensor_conf->isp_ichn_attr->width, sensor_conf->isp_ichn_attr->height, cap_info_.width, cap_info_.height, cap_info_.rotation_);
		if (gdc_bin) {
			gdc_bin_buf_.push_back(gdc_bin);
			pipe_contex[0].gdc_bin_r = gdc_bin;
		}
	}
	if ((strcasecmp(pipe_contex[0].sensor_config.sensor_name, "sc132gs-1280p") == 0)) {
		std::string sensor_tuning = "/usr/hobot/bin/sc132gs_tuning_yg.json";
		rcpputils::fs::path file_path = sensor_tuning;
		if (rcpputils::fs::exists(file_path)) {
			std::strcpy(pipe_contex[0].sensor_config.camera_config->calib_lname ,"sc132gs_tuning_yg.json");
		}	
	}

	ret = create_and_run_vflow(&pipe_contex[0]);
	ERR_CON_EQ(ret, 0);
  }

  cap_info_.sensor_type = pipe_contex[0].sensor_config.sensor_name;

  m_inited_ = true;

  return ret;
}

int HobotMipiCapIml::deInit() {
  int i = 0;
  if (m_inited_) {
	m_inited_ = false;
	
	for(auto contex : pipe_contex){
		hbn_vflow_destroy(contex.vflow_fd);
	}

	hb_mem_module_close();
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
       "x5_cam_deinit end.\n");
  }

  return 0;
}


int HobotMipiCapIml::start() {
  int i = 0, ret = 0;
  // 使能 vps
  for(auto contex : pipe_contex){
    ret = hbn_vflow_start(contex.vflow_fd);
    ERR_CON_EQ(ret, 0);
  }
  for(auto contex : pipe_contex){
	contex.vse_node_handle = hbn_vflow_get_vnode_handle(contex.vflow_fd, HB_VSE, 0);
	if (contex.vse_node_handle <= 0) {
		printf("get vflow %d vse handle error\n", i);
	}
  }
  started_ = true;
  if (!pipe_contex.empty()) {
	  for(auto contex : pipe_contex) {
		auto que_manger = std::make_shared<BuffQueueManage>();
		que_manger->creat_buff(5);
		v_buff_que_manger_.push_back(que_manger);
	  }
	  combine_buff_que_manger_ = std::make_shared<BuffQueueManage>();
	  combine_buff_que_manger_->creat_buff(5);
	  multi_frame_task_ = std::make_shared<std::thread>(
		std::bind(&HobotMipiCapIml::multiFrameTask, this));
	  if (combine_flag_) {
		for(auto contex : pipe_contex) {
			v_frame_que_.push_back(std::make_shared<FrameQueue>());
		}
		sync_task_ = std::make_shared<std::thread>(
				std::bind(&HobotMipiCapIml::sync_task, this));
	  }
	  if (cap_info_.sub_stream_enable_ == true) {
		for(auto contex : pipe_contex) {
			auto que_manger = std::make_shared<BuffQueueManage>();
			que_manger->creat_buff(5);
			v_sub_buff_que_manger_.push_back(que_manger);
		  }
		  sub_combine_buff_que_manger_ = std::make_shared<BuffQueueManage>();
		  sub_combine_buff_que_manger_->creat_buff(5);
		  sub_multi_frame_task_ = std::make_shared<std::thread>(
			std::bind(&HobotMipiCapIml::subMultiFrameTask, this));
		  if (combine_flag_) {
			for(auto contex : pipe_contex) {
				v_sub_frame_que_.push_back(std::make_shared<FrameQueue>());
			}
			sub_sync_task_ = std::make_shared<std::thread>(
					std::bind(&HobotMipiCapIml::sub_sync_task, this));
		  }
	  }
  }
  return 0;
}

int HobotMipiCapIml::stop() {
  int i = 0, ret = 0;
  if (!started_) {
     RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
      "x5 camera isn't started");
    return -1;
  }
  started_ = false;
  for(auto contex : pipe_contex){
    ret = hbn_vflow_stop(contex.vflow_fd);
    ERR_CON_EQ(ret, 0);
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"), "x5_mipi_cam_stop end.\n");
  return 0;
}

std::shared_ptr<VideoBuffer> HobotMipiCapIml::getFrame(std::string channel) {
	std::shared_ptr<VideoBuffer> buff_ptr = nullptr;
	if (!started_) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
		"x5 camera isn't started");
		return buff_ptr;
	}
	int loop = (1000 / cap_info_.fps + 100) / 10;
	do {
		if (!rclcpp::ok()) break;

		if (channel == "single") {
			buff_ptr = v_buff_que_manger_[0]->get_data_buff();
			if (buff_ptr) {
				return buff_ptr;
			} 
		} else if (channel == "sub_single") {
			buff_ptr = v_sub_buff_que_manger_[0]->get_data_buff();
			if (buff_ptr) {
				return buff_ptr;
			}
		} else if (channel == "left") {
			buff_ptr = v_buff_que_manger_[0]->get_data_buff();
			if (buff_ptr) {
				return buff_ptr;
			}
		} else if (channel == "right") {
			buff_ptr = v_buff_que_manger_[1]->get_data_buff();
			if (buff_ptr) {
				return buff_ptr;
			}
		} else if (channel == "combine") {
			buff_ptr = combine_buff_que_manger_->get_data_buff();
			if (buff_ptr) {
				return buff_ptr;
			}
		} else if (channel == "sub_left") {
			buff_ptr = v_sub_buff_que_manger_[0]->get_data_buff();
			if (buff_ptr) {
				return buff_ptr;
			}
		} else if (channel == "sub_right") {
			buff_ptr = v_sub_buff_que_manger_[1]->get_data_buff();
			if (buff_ptr) {
				return buff_ptr;
			}
		} else if (channel == "sub_combine") {
			buff_ptr = sub_combine_buff_que_manger_->get_data_buff();
			if (buff_ptr) {
				return buff_ptr;
			}
		}
		usleep(10 * 1000);
	} while ((loop-- > 0) && started_);
	return buff_ptr;
}

int HobotMipiCapIml::getVnodeFrame(hbn_vnode_handle_t handle, int channel, std::shared_ptr<VideoBuffer> buff_ptr) {

	if (buff_ptr == nullptr) {

		return -1;
	}
	hbn_vnode_image_t out_img;
	int ret = hbn_vnode_getframe(handle, channel, 1000, &out_img);

	if (ret != 0) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_vnode_getframe VSE channel  = %d,ret = %d failed\n", channel,ret);
		return -1;
	}
	hb_mem_invalidate_buf_with_vaddr((uint64_t)out_img.buffer.virt_addr[0],out_img.buffer.size[0]);

	hb_mem_invalidate_buf_with_vaddr((uint64_t)out_img.buffer.virt_addr[1],out_img.buffer.size[1]);

	//*timestamp = out_img.info.trig_tv.tv_sec * 1e9 + out_img.info.trig_tv.tv_usec * 1e3;
	//*timestamp = out_img.info.tv.tv_sec * 1e9 + out_img.info.tv.tv_usec * 1e3;
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);

	int32_t exposure_time = (out_img.info.tv.tv_sec - out_img.info.trig_tv.tv_sec) * 1e9 + 
						(out_img.info.tv.tv_usec - out_img.info.trig_tv.tv_usec) * 1e3;  

	if (out_img.info.trig_tv.tv_sec != 0 && 
		out_img.info.trig_tv.tv_usec != 0) {
		out_img.info.sys_timestamps -= exposure_time;
	}

	//  timestamps means kernel timestamp when the frame is obtained
	//  sys_timestamps means kernel system timestamp when the frame is obtained
	//  tv means hardware timestamp when the frame is obtained
	//  trig_tv means hardware timestamp when the frame is triggered by the external trigger
	struct timespec upts;
	clock_gettime(CLOCK_MONOTONIC, &upts);
	double timestamps = out_img.info.timestamps * 1e-9;
	double sys_timestamps = out_img.info.sys_timestamps * 1e-9;
	double hw_timestamp = out_img.info.tv.tv_sec + (double)out_img.info.tv.tv_usec * 1e-6;
	double tri_timestamp = out_img.info.trig_tv.tv_sec + (double)out_img.info.trig_tv.tv_usec * 1e-6;
	double current_ts =  ts.tv_sec + (double)ts.tv_nsec * 1e-9;
	double current_uptime_ts =  upts.tv_sec + (double)upts.tv_nsec * 1e-9;

	buff_ptr->frame_id = out_img.info.frame_id;
	if ("realtime" == cap_info_.frame_ts_type_) {
		buff_ptr->timestamp = out_img.info.sys_timestamps;
	} else {
		buff_ptr->timestamp = out_img.info.timestamps;
	}                       
						
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
			"capture a frame, handle: %llu, id: %d, timestamps: %f, current uptime: %f, sys_timestamps: %f, HW timestamp: %f, trig timestamp: %f,"
			"current timestamp: %f, laps ms: %fms, exposure_time: %fms.", 
									handle, buff_ptr->frame_id, timestamps, current_uptime_ts, sys_timestamps, hw_timestamp, tri_timestamp,
							current_ts, (current_ts - sys_timestamps) * 1e3, (double)exposure_time * 1e-6);

	//std::cout << "getVnodeFrame--system time sec:" << tv.tv_sec << ", image time sec:" << out_img.info.tv.tv_sec
	//          << ", trig time sec:" << out_img.info.trig_tv.tv_sec 
	//		  << ", image timestamps(/1e9) sec:" << out_img.info.timestamps / 1e9 <<  std::endl;

	//std::cout << "getVnodeFrame--system time sec:" << tv.tv_sec << ", timestamp time sec:" << (int)(*timestamp / 1e9) <<  std::endl;

	buff_ptr->stride = out_img.buffer.stride;
	buff_ptr->width = out_img.buffer.width;
	buff_ptr->height = out_img.buffer.height;
	buff_ptr->buff_size = out_img.buffer.size[0] + out_img.buffer.size[1];
	buff_ptr->buff.resize(buff_ptr->buff_size);
	buff_ptr->encode = "nv12";

	memcpy(buff_ptr->buff.data(), out_img.buffer.virt_addr[0], out_img.buffer.size[0]);
	memcpy(buff_ptr->buff.data() + out_img.buffer.size[0], out_img.buffer.virt_addr[1], out_img.buffer.size[1]);
	hbn_vnode_releaseframe(handle, channel, &out_img);
	return 0;
}

void HobotMipiCapIml::multiFrameTask() {
	if (!started_) {
	   RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "s600 camera isn't started");
	  return;
	}
	if (pipe_contex.empty()) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "s600 pipeline  is zero");
	  return;
	}

	int pipe_num = pipe_contex.size();
	int ret = 0;
	fd_set readfds;
	struct timeval timeout;
	int result;
	int max_handle;
	std::vector<int> ochn_fd;
	ochn_fd.resize(pipe_num);
	// 1. 准备空间
	std::vector<int> indices(pipe_num);

	// 2. 填充递增序列：从 0 开始，后续元素依次加 1
	std::iota(indices.begin(), indices.end(), 0); 

	std::for_each(indices.begin(), indices.end(), [&](int i) {
		hbn_vnode_get_fd(pipe_contex[i].stream_handle, 0, &ochn_fd[i]);
	});

	while (started_) {
	  max_handle = 0;
	  FD_ZERO(&readfds);
	  std::for_each(indices.begin(), indices.end(), [&](int i) {
		FD_SET(ochn_fd[i], &readfds);
		max_handle = max_handle > ochn_fd[i]?max_handle : ochn_fd[i];
	  });

	  timeout.tv_sec = 2;
	  timeout.tv_usec = 0;
	  result = select(max_handle + 1, &readfds, nullptr, nullptr, &timeout);
	  if (result == -1) {
		  std::cerr << "Select error" << std::endl;
		  break;
	  } else if (result == 0) {
		  // 超时
		  std::cout << "Timeout occurred" << std::endl;
		  continue;
	  } else {
		  for (int i = 0; i < ochn_fd.size(); i++) {
			  //if (!rclcpp::ok()) break;
			  if (FD_ISSET(ochn_fd[i], &readfds)) {
				std::shared_ptr<VideoBuffer> buff_ptr = v_buff_que_manger_[i]->get_empty_buff();
				if (buff_ptr) {
					ret = getVnodeFrame(pipe_contex[i].stream_handle, 0, buff_ptr);
					if (ret == 0) {
						if (combine_flag_) {
							auto buff_tmp = std::make_shared<VideoBuffer>(*buff_ptr);
							v_frame_que_[i]->push(buff_tmp);
						}
						buff_ptr->return_data_que();
					} else {
						RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_vnode_getframe VSE channel = %d failed ,ret = %d\n", i,ret);
						buff_ptr->return_empty_que();
					}
				}
			  }
		  }
	  }
	}
	return;
}

void HobotMipiCapIml::subMultiFrameTask() {
	if (!started_) {
	   RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "s600 camera isn't started");
	  return;
	}
	if (pipe_contex.empty()) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "s600 pipeline  is zero");
	  return;
	}

	int pipe_num = pipe_contex.size();
	int ret = 0;
	fd_set readfds;
	struct timeval timeout;
	int result;
	int max_handle;
	std::vector<int> ochn_fd;
	ochn_fd.resize(pipe_num);
	// 1. 准备空间
	std::vector<int> indices(pipe_num);

	// 2. 填充递增序列：从 0 开始，后续元素依次加 1
	std::iota(indices.begin(), indices.end(), 0); 

	std::for_each(indices.begin(), indices.end(), [&](int i) {
		hbn_vnode_get_fd(pipe_contex[i].sub_stream_handle, pipe_contex[i].sub_stream_channel, &ochn_fd[i]);
	});

	while (started_) {
	  max_handle = 0;
	  FD_ZERO(&readfds);
	  std::for_each(indices.begin(), indices.end(), [&](int i) {
		FD_SET(ochn_fd[i], &readfds);
		max_handle = max_handle > ochn_fd[i]?max_handle : ochn_fd[i];
	  });

	  timeout.tv_sec = 2;
	  timeout.tv_usec = 0;
	  result = select(max_handle + 1, &readfds, nullptr, nullptr, &timeout);
	  if (result == -1) {
		  std::cerr << "Select error" << std::endl;
		  break;
	  } else if (result == 0) {
		  // 超时
		  std::cout << "Timeout occurred" << std::endl;
		  continue;
	  } else {
		  for (int i = 0; i < ochn_fd.size(); i++) {
			  //if (!rclcpp::ok()) break;
			  if (FD_ISSET(ochn_fd[i], &readfds)) {
				std::shared_ptr<VideoBuffer> buff_ptr = v_sub_buff_que_manger_[i]->get_empty_buff();
				if (buff_ptr) {
					ret = getVnodeFrame(pipe_contex[i].sub_stream_handle, pipe_contex[i].sub_stream_channel, buff_ptr);
					if (ret == 0) {
						if (combine_flag_) {
							auto buff_tmp = std::make_shared<VideoBuffer>(*buff_ptr);
							v_sub_frame_que_[i]->push(buff_tmp);
						}
						buff_ptr->return_data_que();
					} else {
						RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_vnode_getframe VSE channel = %d failed ,ret = %d\n", pipe_contex[i].sub_stream_channel,ret);
						buff_ptr->return_empty_que();
					}
				}
			  }
		  }
	  }
	}
	return;
}

long long TOLERANCE = 52*1000*1000; // ms
void HobotMipiCapIml::sync_task() {
	TOLERANCE = 500000000 / cap_info_.fps;
	using clock = std::chrono::steady_clock;
	auto last_time = clock::now();

	while (rclcpp::ok()) {
	  std::vector<std::shared_ptr<VideoBuffer>> frames;

	  std::for_each(v_frame_que_.begin(), v_frame_que_.end(), [&](auto &frame_que) {
		auto frame = frame_que->peek();
		if (frame) {
			frames.push_back(frame);
		}
	  });

	  if (frames.size() != v_frame_que_.size()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		continue;
	  }

	  auto target_ts = frames[0]->timestamp;
	  std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
		target_ts = std::max(target_ts, frame->timestamp);
	  });

	  // drop old frames
	  std::for_each(v_frame_que_.begin(), v_frame_que_.end(), [&](auto &frame_que) {
		frame_que->popUntil(target_ts - TOLERANCE);
	  });
  
	  frames.clear();
  
	  // re-peek
	  std::for_each(v_frame_que_.begin(), v_frame_que_.end(), [&](auto &frame_que) {
		auto frame = frame_que->peek();
		if (frame) {
			frames.push_back(frame);
		}
	  });

	  if (frames.size() != v_frame_que_.size()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		continue;
	  }
  
	  if (isSynced(frames, TOLERANCE)) {
		// sync, pop and save
		frames.clear();
		std::for_each(v_frame_que_.begin(), v_frame_que_.end(), [&](auto &frame_que) {
			auto frame = frame_que->pop();
			if (frame) {
				frames.push_back(frame);
			}
		});
		if (frames.size() != v_frame_que_.size()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

		auto buff_ptr = combine_buff_que_manger_->get_empty_buff();

		buff_ptr->timestamp = frames[0]->timestamp;
		buff_ptr->frame_id = frames[0]->frame_id;
		buff_ptr->width = frames[0]->width;
		buff_ptr->height = frames[0]->height * frames.size();
		buff_ptr->stride = frames[0]->stride;
		buff_ptr->encode = frames[0]->encode;
		int buff_offset = 0;
		int y_size = frames[0]->width * frames[0]->height;
		int uv_size = y_size / 2;
		if (buff_ptr->encode == "nv12") {
			buff_ptr->buff.resize((y_size + uv_size) * frames.size());
			std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
				memcpy(buff_ptr->buff.data() + buff_offset, frame->buff.data(), y_size);
				buff_offset += y_size;
			  });
			std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
				memcpy(buff_ptr->buff.data() + buff_offset, frame->buff.data() + y_size, uv_size);
				buff_offset += uv_size;
			  });
		} else {
			int buff_size = 0;
			std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
				buff_size += frame->buff.size();
			  });
			buff_ptr->buff.resize(buff_size);
			std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
				memcpy(buff_ptr->buff.data() + buff_offset, frame->buff.data(), frame->buff.size());
				buff_offset += frame->buff.size();
			  });
		}
		buff_ptr->return_data_que();
	  }
	  std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}
  
void HobotMipiCapIml::sub_sync_task() {
	TOLERANCE = 500000000 / cap_info_.fps;
	using clock = std::chrono::steady_clock;
	auto last_time = clock::now();

	while (rclcpp::ok()) {
	  std::vector<std::shared_ptr<VideoBuffer>> frames;

	  std::for_each(v_sub_frame_que_.begin(), v_sub_frame_que_.end(), [&](auto &frame_que) {
		auto frame = frame_que->peek();
		if (frame) {
			frames.push_back(frame);
		}
	  });

	  if (frames.size() != v_sub_frame_que_.size()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		continue;
	  }

	  auto target_ts = frames[0]->timestamp;
	  std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
		target_ts = std::max(target_ts, frame->timestamp);
	  });

	  // drop old frames
	  std::for_each(v_sub_frame_que_.begin(), v_sub_frame_que_.end(), [&](auto &frame_que) {
		frame_que->popUntil(target_ts - TOLERANCE);
	  });
  
	  frames.clear();
  
	  // re-peek
	  std::for_each(v_sub_frame_que_.begin(), v_sub_frame_que_.end(), [&](auto &frame_que) {
		auto frame = frame_que->peek();
		if (frame) {
			frames.push_back(frame);
		}
	  });

	  if (frames.size() != v_sub_frame_que_.size()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		continue;
	  }
  
	  if (isSynced(frames, TOLERANCE)) {
		// sync, pop and save
		frames.clear();
		std::for_each(v_sub_frame_que_.begin(), v_sub_frame_que_.end(), [&](auto &frame_que) {
			auto frame = frame_que->pop();
			if (frame) {
				frames.push_back(frame);
			}
		});
		if (frames.size() != v_sub_frame_que_.size()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

		auto buff_ptr = sub_combine_buff_que_manger_->get_empty_buff();

		buff_ptr->timestamp = frames[0]->timestamp;
		buff_ptr->frame_id = frames[0]->frame_id;
		buff_ptr->width = frames[0]->width;
		buff_ptr->height = frames[0]->height * frames.size();
		buff_ptr->stride = frames[0]->stride;
		buff_ptr->encode = frames[0]->encode;
		int buff_offset = 0;
		int y_size = frames[0]->width * frames[0]->height;
		int uv_size = y_size / 2;
		if (buff_ptr->encode == "nv12") {
			buff_ptr->buff.resize((y_size + uv_size) * frames.size());
			std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
				memcpy(buff_ptr->buff.data() + buff_offset, frame->buff.data(), y_size);
				buff_offset += y_size;
			  });
			std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
				memcpy(buff_ptr->buff.data() + buff_offset, frame->buff.data() + y_size, uv_size);
				buff_offset += uv_size;
			  });
		} else {
			int buff_size = 0;
			std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
				buff_size += frame->buff.size();
			  });
			buff_ptr->buff.resize(buff_size);
			std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
				memcpy(buff_ptr->buff.data() + buff_offset, frame->buff.data(), frame->buff.size());
				buff_offset += frame->buff.size();
			  });
		}
		buff_ptr->return_data_que();
	  }
	  std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

bool HobotMipiCapIml::isSynced(const std::vector<std::shared_ptr<VideoBuffer>> &frames, long long tolerance) {
	  long long min_ts = (long long)frames[0]->timestamp;
	  long long max_ts = (long long)frames[0]->timestamp;
	  std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
		min_ts = std::min(min_ts, (long long)frame->timestamp);
		max_ts = std::max(max_ts, (long long)frame->timestamp);
	  });
	  return (max_ts - min_ts) <= tolerance;
  }

int HobotMipiCapIml::getCapInfo(MIPI_CAP_INFO_ST &info) {
  info = cap_info_;
  return 0;
}

int HobotMipiCapIml::creat_camera_node(camera_config_t* camera_config,int64_t* cam_fd) {
	int32_t ret = 0;
	ret = hbn_camera_create(camera_config, cam_fd);
	ERR_CON_EQ(ret, 0);
	return 0;
}

static void print_lpwm_attr(vin_node_attr_t *vin_node_attr) {
  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"lpwm_enable: %d\n", vin_node_attr->lpwm_attr.enable);
  for (int i = 0; i < 4; ++i) {
    RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"lpwm_index: %d, trigger_source: %d, trigger_mode: %d, period: %d, offset: %d, duty_time: %d, threshold: %d, adjust_step: %d\n",
    i, vin_node_attr->lpwm_attr.lpwm_chn_attr[i].trigger_source, vin_node_attr->lpwm_attr.lpwm_chn_attr[i].trigger_mode,
    vin_node_attr->lpwm_attr.lpwm_chn_attr[i].period, vin_node_attr->lpwm_attr.lpwm_chn_attr[i].offset,
    vin_node_attr->lpwm_attr.lpwm_chn_attr[i].duty_time, vin_node_attr->lpwm_attr.lpwm_chn_attr[i].threshold,
    vin_node_attr->lpwm_attr.lpwm_chn_attr[i].adjust_step);
  }
}

int HobotMipiCapIml::creat_vin_node(pipe_contex_t *pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	uint32_t hw_id = 0;
	int32_t ret = 0;
	uint32_t chn_id = 0;
	uint64_t vin_attr_ex_mask = 0;
	vin_attr_ex_t vin_attr_ex;
	vp_sensor_config_t& sensor_config = pipe_contex->sensor_config;

	if(pipe_contex->csi_config.mclk_is_not_configed){
		//设备树中没有配置mclk：使用外部晶振
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"csi%d ignore mclk ex attr, because not config mclk.\n",
			pipe_contex->csi_config.index);
		vin_attr_ex.vin_attr_ex_mask = 0x00;
	}else{
		vin_attr_ex.vin_attr_ex_mask = sensor_config.vin_attr_ex->vin_attr_ex_mask;	//bit7 for mclk
		vin_attr_ex.mclk_ex_attr.mclk_freq = sensor_config.vin_attr_ex->mclk_ex_attr.mclk_freq; // 24MHz
		vin_attr_ex_mask = vin_attr_ex.vin_attr_ex_mask;
	}

	hw_id = sensor_config.vin_node_attr->cim_attr.mipi_rx;
  	//sensor_config.vin_node_attr->cim_attr.func.ts_src = hw_id + 1;
	ret = hbn_vnode_open(HB_VIN, hw_id, AUTO_ALLOC_ID, &pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);
	// 设置基本属性
	ret = hbn_vnode_set_attr(pipe_contex->vin_node_handle, sensor_config.vin_node_attr);
    print_lpwm_attr(sensor_config.vin_node_attr);
	ERR_CON_EQ(ret, 0);
	// 设置输入通道的属性
	ret = hbn_vnode_set_ichn_attr(pipe_contex->vin_node_handle, chn_id, sensor_config.vin_ichn_attr);
	ERR_CON_EQ(ret, 0);
	// 设置输出通道的属性
	ret = hbn_vnode_set_ochn_attr(pipe_contex->vin_node_handle, chn_id, sensor_config.vin_ochn_attr);
	ERR_CON_EQ(ret, 0);
	hbn_buf_alloc_attr_t alloc_attr_raw = {0};
    alloc_attr_raw.buffers_num = 3;
	alloc_attr_raw.is_contig = 1;
	alloc_attr_raw.flags = HB_MEM_USAGE_CPU_READ_OFTEN
						| HB_MEM_USAGE_CPU_WRITE_OFTEN
						| HB_MEM_USAGE_CACHED
						| HB_MEM_USAGE_HW_CIM
						| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->vin_node_handle, chn_id, &alloc_attr_raw);
	ERR_CON_EQ(ret, 0);

	// 设置额外属性，for mclk
	vin_attr_ex_mask = vin_attr_ex.vin_attr_ex_mask;
	if (vin_attr_ex_mask) {
		for (uint8_t i = 0; i < VIN_ATTR_EX_INVALID; i ++) {
			if ((vin_attr_ex_mask & (1 << i)) == 0)
				continue;
			vin_attr_ex.ex_attr_type = (vin_attr_ex_type_s)i;
			/*we need to set hbn_vnode_set_attr_ex in a loop*/
			ret = hbn_vnode_set_attr_ex(pipe_contex->vin_node_handle, &vin_attr_ex);
			ERR_CON_EQ(ret, 0);
		}
	}

	return 0;
}


int HobotMipiCapIml::creat_isp_node(pipe_contex_t *pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	hbn_buf_alloc_attr_t alloc_attr = {0};
	uint32_t chn_id = 0;
	int ret = 0;
	vp_sensor_config_t& sensor_config = pipe_contex->sensor_config;

	ret = hbn_vnode_open(HB_ISP, 0, AUTO_ALLOC_ID, &pipe_contex->isp_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_attr(pipe_contex->isp_node_handle, sensor_config.isp_attr);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ochn_attr(pipe_contex->isp_node_handle, chn_id, sensor_config.isp_ochn_attr);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ichn_attr(pipe_contex->isp_node_handle, chn_id, sensor_config.isp_ichn_attr);
	ERR_CON_EQ(ret, 0);
	alloc_attr.buffers_num = 3;
	alloc_attr.is_contig = 1;
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
						| HB_MEM_USAGE_CPU_WRITE_OFTEN
						| HB_MEM_USAGE_CACHED
						| HB_MEM_USAGE_HW_ISP
						| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->isp_node_handle, chn_id, &alloc_attr);
	ERR_CON_EQ(ret, 0);
	
	isp_ichn_attr_t isp_ichn_attr;
	ret = hbn_vnode_get_ichn_attr(pipe_contex->isp_node_handle, chn_id, &isp_ichn_attr);
	ERR_CON_EQ(ret, 0);
	return 0;
}

int HobotMipiCapIml::creat_vse_node(pipe_contex_t *pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	int ret = 0;
	uint32_t chn_id = 0;
	uint32_t hw_id = 0;
	hbn_buf_alloc_attr_t alloc_attr = {0};
	hbn_buf_alloc_attr_t sub_alloc_attr = {0};
	
	vse_attr_t vse_attr = {0};
	vse_ichn_attr_t vse_ichn_attr;
	vse_ochn_attr_t vse_ochn_attr;
	vse_ochn_attr_t sub_vse_ochn_attr;
	memset(&vse_ichn_attr, 0, sizeof(vse_ichn_attr_t));
	memset(&vse_ochn_attr, 0, sizeof(vse_ochn_attr_t));
	memset(&sub_vse_ochn_attr, 0, sizeof(vse_ochn_attr_t));
	int input_width;
	int input_height;
	if (pipe_contex->cap_info_->stream_mode_ == 1) {
		if (pipe_contex->gdc_init_valid_r == 1) {
			gdc_ochn_attr_t gdc_ochn_attr;
			ret = hbn_vnode_get_ochn_attr(pipe_contex->gdc_node_handle_r, chn_id, &gdc_ochn_attr);
			ERR_CON_EQ(ret, 0);
			input_width = gdc_ochn_attr.output_width;
			input_height = gdc_ochn_attr.output_height;
		} else {
			isp_ichn_attr_t isp_ichn_attr;
			ret = hbn_vnode_get_ichn_attr(pipe_contex->isp_node_handle, chn_id, &isp_ichn_attr);
			ERR_CON_EQ(ret, 0);
			input_width = isp_ichn_attr.width;
			input_height = isp_ichn_attr.height;
		}
	} else {
		if (pipe_contex->gdc_init_valid == 1) {
			gdc_ochn_attr_t gdc_ochn_attr;
			ret = hbn_vnode_get_ochn_attr(pipe_contex->gdc_node_handle, chn_id, &gdc_ochn_attr);
			ERR_CON_EQ(ret, 0);
			input_width = gdc_ochn_attr.output_width;
			input_height = gdc_ochn_attr.output_height;
		} else if (pipe_contex->gdc_init_valid_r == 1) {
			gdc_ochn_attr_t gdc_ochn_attr;
			ret = hbn_vnode_get_ochn_attr(pipe_contex->gdc_node_handle_r, chn_id, &gdc_ochn_attr);
			ERR_CON_EQ(ret, 0);
			input_width = gdc_ochn_attr.output_width;
			input_height = gdc_ochn_attr.output_height;
		} else {
			isp_ichn_attr_t isp_ichn_attr;
			ret = hbn_vnode_get_ichn_attr(pipe_contex->isp_node_handle, chn_id, &isp_ichn_attr);
			ERR_CON_EQ(ret, 0);
			input_width = isp_ichn_attr.width;
			input_height = isp_ichn_attr.height;
		}		
	}

	ret = hbn_vnode_open(HB_VSE, hw_id, AUTO_ALLOC_ID, &pipe_contex->vse_node_handle);
	ERR_CON_EQ(ret, 0);
	

	ret = hbn_vnode_set_attr(pipe_contex->vse_node_handle, &vse_attr);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_get_ichn_attr(pipe_contex->vse_node_handle, chn_id, &vse_ichn_attr);
	ERR_CON_EQ(ret, 0);

	vse_ichn_attr.width = input_width;
	vse_ichn_attr.height = input_height;
	vse_ichn_attr.fmt = FRM_FMT_NV12;
	vse_ichn_attr.bit_width = 8;


	ret = hbn_vnode_set_ichn_attr(pipe_contex->vse_node_handle, chn_id, &vse_ichn_attr);
	ERR_CON_EQ(ret, 0);

	vse_ochn_attr.chn_en = CAM_TRUE;
	vse_ochn_attr.roi.x = 0;
	vse_ochn_attr.roi.y = 0;
	vse_ochn_attr.roi.w = input_width;
	vse_ochn_attr.roi.h = input_height;
	vse_ochn_attr.fmt = FRM_FMT_NV12;
	vse_ochn_attr.bit_width = 8;
	if (pipe_contex->cap_info_->stream_mode_ == 1) {
		vse_ochn_attr.target_w = input_width;
		vse_ochn_attr.target_h = input_height;
	} else {
		vse_ochn_attr.target_w = pipe_contex->cap_info_->width;
		vse_ochn_attr.target_h = pipe_contex->cap_info_->height;
	}
	vse_ochn_attr.fps.src = pipe_contex->sensor_config.camera_config->fps;
	vse_ochn_attr.fps.dst = pipe_contex->cap_info_->fps;

	ret = hbn_vnode_set_ochn_attr(pipe_contex->vse_node_handle, 0, &vse_ochn_attr);
	ERR_CON_EQ(ret, 0);
	alloc_attr.buffers_num = 3;
	alloc_attr.is_contig = 1;
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
						| HB_MEM_USAGE_CPU_WRITE_OFTEN
						| HB_MEM_USAGE_CACHED
						| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->vse_node_handle, 0, &alloc_attr);
	ERR_CON_EQ(ret, 0);
	if (pipe_contex->cap_info_->sub_stream_enable_ == true) {
		if ((pipe_contex->cap_info_->sub_width <= input_width) 
				&& (pipe_contex->cap_info_->sub_height <= input_height) 
				&& (pipe_contex->cap_info_->sub_width <= 1920)
				&& (pipe_contex->cap_info_->sub_height <= 1080)) {
			sub_vse_ochn_attr.chn_en = CAM_TRUE;
			sub_vse_ochn_attr.roi.x = 0;
			sub_vse_ochn_attr.roi.y = 0;
			sub_vse_ochn_attr.roi.w = input_width;
			sub_vse_ochn_attr.roi.h = input_height;
			sub_vse_ochn_attr.fmt = FRM_FMT_NV12;
			sub_vse_ochn_attr.bit_width = 8;
			//sub_vse_ochn_attr.target_w = input_width;
			//sub_vse_ochn_attr.target_h = input_height;
			sub_vse_ochn_attr.target_w = pipe_contex->cap_info_->sub_width;
			sub_vse_ochn_attr.target_h = pipe_contex->cap_info_->sub_height;
		
			sub_vse_ochn_attr.fps.src = pipe_contex->sensor_config.camera_config->fps;
			sub_vse_ochn_attr.fps.dst = pipe_contex->cap_info_->fps;
		
			ret = hbn_vnode_set_ochn_attr(pipe_contex->vse_node_handle, 1, &sub_vse_ochn_attr);
			ERR_CON_EQ(ret, 0);
			sub_alloc_attr.buffers_num = 3;
			sub_alloc_attr.is_contig = 1;
			sub_alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
								| HB_MEM_USAGE_CPU_WRITE_OFTEN
								| HB_MEM_USAGE_CACHED
								| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
			ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->vse_node_handle, 1, &sub_alloc_attr);
			ERR_CON_EQ(ret, 0);
			pipe_contex->sub_stream_channel = 1;
		} else if ((pipe_contex->cap_info_->sub_width >= input_width) &&
				(pipe_contex->cap_info_->sub_height >= input_height) &&
				(pipe_contex->cap_info_->sub_width <= input_width * 2) &&
				(pipe_contex->cap_info_->sub_height <= input_height * 2)) {
			sub_vse_ochn_attr.chn_en = CAM_TRUE;
			sub_vse_ochn_attr.roi.x = 0;
			sub_vse_ochn_attr.roi.y = 0;
			sub_vse_ochn_attr.roi.w = input_width;
			sub_vse_ochn_attr.roi.h = input_height;
			sub_vse_ochn_attr.fmt = FRM_FMT_NV12;
			sub_vse_ochn_attr.bit_width = 8;
			//sub_vse_ochn_attr.target_w = input_width;
			//sub_vse_ochn_attr.target_h = input_height;
			sub_vse_ochn_attr.target_w = pipe_contex->cap_info_->sub_width;
			sub_vse_ochn_attr.target_h = pipe_contex->cap_info_->sub_height;
		
			sub_vse_ochn_attr.fps.src = pipe_contex->sensor_config.camera_config->fps;
			sub_vse_ochn_attr.fps.dst = pipe_contex->cap_info_->fps;
		
			ret = hbn_vnode_set_ochn_attr(pipe_contex->vse_node_handle, 5, &sub_vse_ochn_attr);
			ERR_CON_EQ(ret, 0);
			sub_alloc_attr.buffers_num = 3;
			sub_alloc_attr.is_contig = 1;
			sub_alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
								| HB_MEM_USAGE_CPU_WRITE_OFTEN
								| HB_MEM_USAGE_CACHED
								| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
			ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->vse_node_handle, 5, &sub_alloc_attr);
			ERR_CON_EQ(ret, 0);
			pipe_contex->sub_stream_channel = 5;		
		} else {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),"sub stream width = %d and height = %d is error, don't start the sub stream ",
				pipe_contex->cap_info_->sub_width, pipe_contex->cap_info_->sub_height);
			pipe_contex->cap_info_->sub_stream_enable_ = false;
		}
	}

	return 0;
}

int HobotMipiCapIml::creat_gdc_node_r(pipe_contex_t *pipe_contex) {
	if ((pipe_contex == nullptr) || (pipe_contex->gdc_bin_r == nullptr)) {
		return -1;
	}
	int ret = 0;
	uint32_t chn_id = 0;
	isp_ichn_attr_t isp_ichn_attr;
	pipe_contex->gdc_init_valid_r = 0;
	ret = hbn_vnode_get_ichn_attr(pipe_contex->isp_node_handle, chn_id, &isp_ichn_attr);
	ERR_CON_EQ(ret, 0);
	int input_width = isp_ichn_attr.width;
	int input_height = isp_ichn_attr.height;

	uint32_t hw_id = 0;
	ret = hbn_vnode_open(HB_GDC, hw_id, AUTO_ALLOC_ID, &pipe_contex->gdc_node_handle_r);
	ERR_CON_EQ(ret, 0);
	gdc_attr_t gdc_attr = {0};
	gdc_attr.config_addr = pipe_contex->gdc_bin_r->bin_buf->phys_addr;
	gdc_attr.config_size = pipe_contex->gdc_bin_r->bin_buf->size;
	gdc_attr.binary_ion_id = pipe_contex->gdc_bin_r->bin_buf->share_id;
	gdc_attr.binary_offset = pipe_contex->gdc_bin_r->bin_buf->offset;
	gdc_attr.total_planes = 2;
	gdc_attr.div_width = 0;
	gdc_attr.div_height = 0;
	ret = hbn_vnode_set_attr(pipe_contex->gdc_node_handle_r, &gdc_attr);
	ERR_CON_EQ(ret, 0);
	//uint32_t chn_id = 0;

	gdc_ichn_attr_t gdc_ichn_attr = {0};
	gdc_ichn_attr.input_width = input_width;
	gdc_ichn_attr.input_height = input_height;
	gdc_ichn_attr.input_stride = input_width;
	ret = hbn_vnode_set_ichn_attr(pipe_contex->gdc_node_handle_r, chn_id, &gdc_ichn_attr);
	ERR_CON_EQ(ret, 0);

	gdc_ochn_attr_t gdc_ochn_attr = {0};
	if ((pipe_contex->cap_info_->rotation_ == 90.0) || (pipe_contex->cap_info_->rotation_ == 270.0)) {
		gdc_ochn_attr.output_width = input_height;
		gdc_ochn_attr.output_height = input_width;
		gdc_ochn_attr.output_stride = input_height;
	} else {
		gdc_ochn_attr.output_width = input_width;
		gdc_ochn_attr.output_height = input_height;
		gdc_ochn_attr.output_stride = input_width;
	}

	ret = hbn_vnode_set_ochn_attr(pipe_contex->gdc_node_handle_r, chn_id, &gdc_ochn_attr);
	ERR_CON_EQ(ret, 0);
	hbn_buf_alloc_attr_t alloc_attr = {0};
	alloc_attr.buffers_num = 3;
	alloc_attr.is_contig = 1;
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN |
					HB_MEM_USAGE_CPU_WRITE_OFTEN |
					HB_MEM_USAGE_CACHED;
	ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->gdc_node_handle_r, chn_id, &alloc_attr);
	ERR_CON_EQ(ret, 0);
	pipe_contex->gdc_init_valid_r = 1;

	return 0;
}

int HobotMipiCapIml::creat_gdc_node(pipe_contex_t *pipe_contex) {
	if ((pipe_contex == nullptr) || (pipe_contex->gdc_bin == nullptr)) {
		return -1;
	}
	int ret = 0;
	uint32_t chn_id = 0;
	isp_ichn_attr_t isp_ichn_attr;
	pipe_contex->gdc_bin_buf_is_valid = 0;
	pipe_contex->gdc_init_valid = 0;

	int input_width;
	int input_height;
	int output_width;
	int output_height;

	if (pipe_contex->cap_info_->stream_mode_ == 1) {
		vse_ochn_attr_t vse_ochn_attr;
		ret = hbn_vnode_get_ochn_attr(pipe_contex->vse_node_handle, chn_id, &vse_ochn_attr);
		ERR_CON_EQ(ret, 0);
		input_width = vse_ochn_attr.target_w;
		input_height = vse_ochn_attr.target_h;
		output_width = pipe_contex->cap_info_->width;
		output_height = pipe_contex->cap_info_->height;
	} else {
		isp_ichn_attr_t isp_ichn_attr;
		ret = hbn_vnode_get_ichn_attr(pipe_contex->isp_node_handle, chn_id, &isp_ichn_attr);
		ERR_CON_EQ(ret, 0);
		input_width = isp_ichn_attr.width;
		input_height = isp_ichn_attr.height;
		if (pipe_contex->gdc_resize_enable) {
			output_width = pipe_contex->cap_info_->width;
			output_height = pipe_contex->cap_info_->height;
		} else {
			if ((pipe_contex->cap_info_->rotation_ == 90.0) || (pipe_contex->cap_info_->rotation_ == 270.0)) {
				output_width = input_height;
				output_height = input_width;
			} else {
				output_width = input_width;
				output_height = input_height;
			}
		}		
	}


	pipe_contex->gdc_bin_buf_is_valid = 1;

	uint32_t hw_id = 0;
	ret = hbn_vnode_open(HB_GDC, hw_id, AUTO_ALLOC_ID, &pipe_contex->gdc_node_handle);
	ERR_CON_EQ(ret, 0);
	gdc_attr_t gdc_attr = {0};
	gdc_attr.config_addr = pipe_contex->gdc_bin->bin_buf->phys_addr;
	gdc_attr.config_size = pipe_contex->gdc_bin->bin_buf->size;
	gdc_attr.binary_ion_id = pipe_contex->gdc_bin->bin_buf->share_id;
	gdc_attr.binary_offset = pipe_contex->gdc_bin->bin_buf->offset;
	gdc_attr.total_planes = 2;
	gdc_attr.div_width = 0;
	gdc_attr.div_height = 0;
	ret = hbn_vnode_set_attr(pipe_contex->gdc_node_handle, &gdc_attr);
	ERR_CON_EQ(ret, 0);
	//uint32_t chn_id = 0;

	gdc_ichn_attr_t gdc_ichn_attr = {0};
	gdc_ichn_attr.input_width = input_width;
	gdc_ichn_attr.input_height = input_height;
	gdc_ichn_attr.input_stride = input_width;
	ret = hbn_vnode_set_ichn_attr(pipe_contex->gdc_node_handle, chn_id, &gdc_ichn_attr);
	ERR_CON_EQ(ret, 0);

	gdc_ochn_attr_t gdc_ochn_attr = {0};
	//gdc_ochn_attr.output_width = input_width;
	//gdc_ochn_attr.output_height = input_height;
	//gdc_ochn_attr.output_stride = input_width;
	gdc_ochn_attr.output_width = output_width;
	gdc_ochn_attr.output_height = output_height;
	gdc_ochn_attr.output_stride = output_width;
	ret = hbn_vnode_set_ochn_attr(pipe_contex->gdc_node_handle, chn_id, &gdc_ochn_attr);
	ERR_CON_EQ(ret, 0);
	hbn_buf_alloc_attr_t alloc_attr = {0};
	alloc_attr.buffers_num = 3;
	alloc_attr.is_contig = 1;
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN |
					HB_MEM_USAGE_CPU_WRITE_OFTEN |
					HB_MEM_USAGE_CACHED;
	ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->gdc_node_handle, chn_id, &alloc_attr);
	ERR_CON_EQ(ret, 0);
	pipe_contex->gdc_init_valid = 1;

	return 0;
}

int HobotMipiCapIml::create_and_run_vflow(pipe_contex_t *pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	int32_t ret = 0;
	pipe_contex->sensor_config.isp_attr->input_mode = 2;
	if (pipe_contex->cap_info_->lpwm_enable_) {
		pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
		pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
		int fps_rate = (1000000 / pipe_contex->cap_info_->fps);
		pipe_contex->sensor_config.camera_config->sensor_mode = 6;
		pipe_contex->sensor_config.vin_node_attr->lpwm_attr.enable = 1;
		for (auto& attr : pipe_contex->sensor_config.vin_node_attr->lpwm_attr.lpwm_chn_attr) {
			attr.period = fps_rate;
		}
	} else {
		//pipe_contex->sensor_config.camera_config->sensor_mode = 1;
		pipe_contex->sensor_config.vin_node_attr->lpwm_attr.enable = 0;
		pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
		pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
	}
	// 创建pipeline中的每个node
	ret = creat_camera_node(pipe_contex->sensor_config.camera_config, &pipe_contex->cam_fd);
	ERR_CON_EQ(ret, 0);
	//调用AWB OTP配置函数
	ret = config_awb_otp(pipe_contex);
	ERR_CON_EQ(ret, 0);
	ret = creat_vin_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	ret = creat_isp_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	if (pipe_contex->cap_info_->stream_mode_ == 1) {
		creat_gdc_node_r(pipe_contex);
		ret = creat_vse_node(pipe_contex);
		ERR_CON_EQ(ret, 0);
		if (cap_info_.gdc_enable_) {
			creat_gdc_node(pipe_contex);
		}
	} else {
		if (cap_info_.gdc_enable_) {
			creat_gdc_node(pipe_contex);
		}
		creat_gdc_node_r(pipe_contex);
		ret = creat_vse_node(pipe_contex);
		ERR_CON_EQ(ret, 0);
	}

	// 创建HBN flow
	ret = hbn_vflow_create(&pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle);
	ERR_CON_EQ(ret, 0);
	if (pipe_contex->gdc_init_valid_r == 1) {
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->gdc_node_handle_r);
		ERR_CON_EQ(ret, 0);
	}
	if (pipe_contex->gdc_init_valid == 1) {
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->gdc_node_handle);
		ERR_CON_EQ(ret, 0);
	}
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->vse_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->vin_node_handle,
							0,
							pipe_contex->isp_node_handle,
							0);
	ERR_CON_EQ(ret, 0);
	if (pipe_contex->cap_info_->stream_mode_ == 1) {
		if ((pipe_contex->gdc_init_valid_r == 1) && (pipe_contex->gdc_init_valid == 1)) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc rotation and cal.\n");
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->isp_node_handle,
								0,
								pipe_contex->gdc_node_handle_r,
								0);
			ERR_CON_EQ(ret, 0);
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->gdc_node_handle_r,
								0,
								pipe_contex->vse_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->vse_node_handle,
								0,
								pipe_contex->gdc_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->gdc_node_handle;
		} else if (pipe_contex->gdc_init_valid_r == 1) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc rotation.\n");
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->isp_node_handle,
								0,
								pipe_contex->gdc_node_handle_r,
								0);
			ERR_CON_EQ(ret, 0);
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->gdc_node_handle_r,
								0,
								pipe_contex->vse_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->vse_node_handle;
		} else if (pipe_contex->gdc_init_valid == 1) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc cal.\n");
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->isp_node_handle,
								0,
								pipe_contex->vse_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->vse_node_handle,
								0,
								pipe_contex->gdc_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->gdc_node_handle;
		} else {
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->isp_node_handle,
								0,
								pipe_contex->vse_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->vse_node_handle;
		}	
		pipe_contex->sub_stream_handle = pipe_contex->vse_node_handle;	
	} else {
		if (pipe_contex->gdc_init_valid == 1) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc rotation and cal.\n");
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->isp_node_handle,
								0,
								pipe_contex->gdc_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->gdc_node_handle,
								0,
								pipe_contex->vse_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->vse_node_handle;
		} else if (pipe_contex->gdc_init_valid_r == 1) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc rotation.\n");
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->isp_node_handle,
								0,
								pipe_contex->gdc_node_handle_r,
								0);
			ERR_CON_EQ(ret, 0);
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->gdc_node_handle_r,
								0,
								pipe_contex->vse_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->vse_node_handle;
		} else {
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->isp_node_handle,
								0,
								pipe_contex->vse_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->vse_node_handle;
		}
		pipe_contex->sub_stream_handle = pipe_contex->vse_node_handle;
	}


	ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd,
							pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);
	//if(strcasecmp(pipe_contex->sensor_config.sensor_name, "sc230ai-30fps") == 0) {
	//  ret = hbn_camera_change_fps(pipe_contex->cam_fd, pipe_contex->sensor_config.camera_config->fps);
	//  ERR_CON_EQ(ret, 0);
	//}
	return 0;
}

// 封装AWB OTP配置函数（HBN接口方式）
int HobotMipiCapIml::config_awb_otp(pipe_contex_t *pipe_contex) {
	if (pipe_contex->cam_fd < 0) {
		//RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"), "Invalid cam_fd: %ld for AWB OTP config", cam_fd);
        return -1;
	}

	if (!pipe_contex->awb_otp_data) {
		return 0;
	}

	//初始化AWB OTP配置结构体
	sensor_otp_t pdata = {0};
	pdata.otp_awb_enable = 1;          // 启用AWB OTP配置
    pdata.awb_ct_num = 3;              // AWB色温配置数量
    pdata.awb_golden_ct_num = 3;       // AWB黄金色温配置数量

    // 配置3100K色温参数
    pdata.awb_data[0].color_temperature = ::COLOR_TEMPERATURE_3100K; 
    pdata.awb_data[0].r =   0;        //pipe_contex->awb_otp_data.awb_data[0].r;
    pdata.awb_data[0].gr =  0;           //pipe_contex->awb_otp_data.awb_data[0].gr;
    pdata.awb_data[0].gb =  0;                  //pipe_contex->awb_otp_data.awb_data[0].gb;
    pdata.awb_data[0].b =  0;                    //pipe_contex->awb_otp_data.awb_data[0].b;
    pdata.awb_data[0].rg_ratio = pipe_contex->awb_otp_data->awb_data[0].rg_ratio;
    pdata.awb_data[0].bg_ratio = pipe_contex->awb_otp_data->awb_data[0].bg_ratio;

    pdata.awb_golden_data[0].color_temperature = ::COLOR_TEMPERATURE_3100K;
    pdata.awb_golden_data[0].r = 0;
    pdata.awb_golden_data[0].gr = 0;
    pdata.awb_golden_data[0].gb = 0;
    pdata.awb_golden_data[0].b = 0;
    pdata.awb_golden_data[0].rg_ratio = pipe_contex->awb_otp_data->awb_golden_data[0].rg_ratio;
    pdata.awb_golden_data[0].bg_ratio = pipe_contex->awb_otp_data->awb_golden_data[0].bg_ratio;

    // 配置4000K色温参数
    pdata.awb_data[1].color_temperature = ::COLOR_TEMPERATURE_4000K;
    pdata.awb_data[1].r =    0;             //pipe_contex->awb_otp_data.awb_data[0].r;
    pdata.awb_data[1].gr =     0;         //pipe_contex->awb_otp_data.awb_data[0].gr;
    pdata.awb_data[1].gb =   0;                //pipe_contex->awb_otp_data.awb_data[0].gb;
    pdata.awb_data[1].b =   0;               //pipe_contex->awb_otp_data.awb_data[0].b;
    pdata.awb_data[1].rg_ratio = pipe_contex->awb_otp_data->awb_data[1].rg_ratio;
    pdata.awb_data[1].bg_ratio = pipe_contex->awb_otp_data->awb_data[1].bg_ratio;

    pdata.awb_golden_data[1].color_temperature = ::COLOR_TEMPERATURE_4000K;
    pdata.awb_golden_data[1].r = 0;
    pdata.awb_golden_data[1].gr = 0;
    pdata.awb_golden_data[1].gb = 0;
    pdata.awb_golden_data[1].b = 0;
    pdata.awb_golden_data[1].rg_ratio = pipe_contex->awb_otp_data->awb_golden_data[1].rg_ratio;
    pdata.awb_golden_data[1].bg_ratio = pipe_contex->awb_otp_data->awb_golden_data[1].bg_ratio;

    // 配置5800K色温参数
    pdata.awb_data[2].color_temperature = ::COLOR_TEMPERATURE_5800K;
    pdata.awb_data[2].r =      0;            //pipe_contex->awb_otp_data.awb_data[0].r;
    pdata.awb_data[2].gr =   0;                   //pipe_contex->awb_otp_data.awb_data[0].gr;
    pdata.awb_data[2].gb =   0;                  //pipe_contex->awb_otp_data.awb_data[0].gb;
    pdata.awb_data[2].b =    0;                  //pipe_contex->awb_otp_data.awb_data[0].b;
    pdata.awb_data[2].rg_ratio = pipe_contex->awb_otp_data->awb_data[2].rg_ratio;
    pdata.awb_data[2].bg_ratio = pipe_contex->awb_otp_data->awb_data[2].bg_ratio;

    pdata.awb_golden_data[2].color_temperature = ::COLOR_TEMPERATURE_5800K;
    pdata.awb_golden_data[2].r = 0;
    pdata.awb_golden_data[2].gr = 0;
    pdata.awb_golden_data[2].gb = 0;
    pdata.awb_golden_data[2].b = 0;
    pdata.awb_golden_data[2].rg_ratio = pipe_contex->awb_otp_data->awb_golden_data[2].rg_ratio;
    pdata.awb_golden_data[2].bg_ratio = pipe_contex->awb_otp_data->awb_golden_data[2].bg_ratio;

    // 打印日志+调用HBN接口启用AWB OTP
	//printf("awb otp enable\n");
    //RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"), "AWB OTP config enable, cam_fd: %ld", pipe_contex->cam_fd);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "=> ================== all awb otp data ==================");
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_golden_data[0].rg_ratio: %ld", pdata.awb_golden_data[0].rg_ratio);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_golden_data[0].bg_ratio: %ld",  pdata.awb_golden_data[0].bg_ratio);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_golden_data[1].rg_ratio: %ld",  pdata.awb_golden_data[1].rg_ratio);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_golden_data[1].bg_ratio: %ld",  pdata.awb_golden_data[1].bg_ratio);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_golden_data[2].rg_ratio: %ld",  pdata.awb_golden_data[2].rg_ratio);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_golden_data[2].bg_ratio: %ld",  pdata.awb_golden_data[2].bg_ratio);

	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].rg_ratio: %ld", pipe_contex->awb_otp_data->awb_data[0].rg_ratio);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].bg_ratio: %ld",  pipe_contex->awb_otp_data->awb_data[0].bg_ratio);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[1].rg_ratio: %ld",  pipe_contex->awb_otp_data->awb_data[1].rg_ratio);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[1].bg_ratio: %ld",  pipe_contex->awb_otp_data->awb_data[1].bg_ratio);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[2].rg_ratio: %ld",  pipe_contex->awb_otp_data->awb_data[2].rg_ratio);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[2].bg_ratio: %ld",  pipe_contex->awb_otp_data->awb_data[2].bg_ratio);

	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].r: %ld",  pipe_contex->awb_otp_data->awb_data[0].r);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].gr: %ld",  pipe_contex->awb_otp_data->awb_data[0].gr);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].gb: %ld",  pipe_contex->awb_otp_data->awb_data[0].gb);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].b: %ld",  pipe_contex->awb_otp_data->awb_data[0].b);

	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].r: %ld",  pdata.awb_data[0].r);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].gr: %ld",  pdata.awb_data[0].gr);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].gb: %ld",  pdata.awb_data[0].gb);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].b: %ld",  pdata.awb_data[0].b);
	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "=> ================== all awb otp data ==================");

    int32_t ret = hbn_camera_enable_otp(pipe_contex->cam_fd, &pdata);                    ////cam_fd由hbn_camera_create创建
	ERR_CON_EQ(ret, 0);
    return 0;
}

void HobotMipiCapIml::listMipiHost(std::vector<int> &mipi_hosts, 
    std::vector<int> &started, std::vector<int> &stoped) {
  std::vector<int> host;
  std::string board_type_str = "";
  for (int num : mipi_hosts) {
    std::string mipi_host = "/sys/class/vps/mipi_host" + std::to_string(num) + "/status/cfg";
    std::ifstream mipi_host_fd(mipi_host);
    board_type_str = "";
    if (mipi_host_fd.is_open()) {
      std::getline(mipi_host_fd, board_type_str);
      if (board_type_str == "not inited") {
        stoped.push_back(num);
      } else {
        started.push_back(num);
      }
      mipi_host_fd.close();
    }
  }
}

bool HobotMipiCapIml::detectSensor(SENSOR_ID_T &sensor_info, int i2c_bus) {
  char cmd[256];
  char result[1024];
  memset(cmd, '\0', sizeof(cmd));
  memset(result, '\0', sizeof(result));
  if (sensor_info.i2c_addr_width == I2C_ADDR_8) {
    sprintf(cmd, "i2ctransfer -y -f %d w1@0x%x 0x%x r1 2>&1",
            i2c_bus,
            sensor_info.i2c_dev_addr,
            sensor_info.det_reg);
  } else if (sensor_info.i2c_addr_width == I2C_ADDR_16) {
    sprintf(cmd,
            "i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1",
            i2c_bus,
            sensor_info.i2c_dev_addr,
            sensor_info.det_reg >> 8,
            sensor_info.det_reg & 0xFF);
  } else {
    return false;
  }
  exec_cmd_ex(cmd, result, sizeof(result));
  if (strstr(result, "Error") == NULL && strstr(result, "error") == NULL) {
    // 返回结果中不带Error, 说明sensor找到了
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
          "match sensor:%s\n", sensor_info.sensor_name);
    return true;
  }
  return false;
}


bool HobotMipiCapIml::analysis_board_config() {
  std::string board_type;
  bool auto_detect = false;
  std::ifstream som_name("/sys/class/socinfo/board_id");
  if (som_name.is_open()) {
    if (!getline(som_name, board_type)) {
      som_name.close();
      return false;
    }
  } else {
    return false;
  }

  std::ifstream board_config("/etc/board_config.json");
  if (!board_config.is_open()) {
    return false;
  }
  std::string  board_name = "board_" + board_type;
  Json::Value root;
  board_config >> root;
  std::string reset;
  int i2c_bus;
  int mipi_host;
  int gpio_num;
  int reset_level;
  std::regex regexPattern(R"((\d+):(\w+))");
  try {
    int camera_num = root[board_name]["camera_num"].asInt();
    for (int i = 0; i < camera_num; i++) {
      mipi_host = root[board_name]["cameras"][i]["mipi_host"].asInt();
      i2c_bus = root[board_name]["cameras"][i]["i2c_bus"].asInt();
      board_config_m_[mipi_host].mipi_host = mipi_host;
      board_config_m_[mipi_host].i2c_bus = i2c_bus;
      board_config_m_[mipi_host].reset_flag = false;
      if (root[board_name]["cameras"][i].isMember("reset")){
        reset = root[board_name]["cameras"][i]["reset"].asString();
        std::smatch matches;
        if (std::regex_search(reset, matches, regexPattern)) {
          board_config_m_[mipi_host].reset_gpio = std::stoi(matches[1]);
          if (matches[2] == "low") {
            board_config_m_[mipi_host].reset_level = 1;
          } else {
            board_config_m_[mipi_host].reset_level = 0;
          } 
          board_config_m_[mipi_host].reset_flag = true;
        } 
      }
    }
  }catch (std::runtime_error& e) {
    return false;
  }
  return true;
}

int HobotMipiCapIml::selectSensor(std::string &sensor, int &host, int &i2c_bus) {

  // mipi sensor的信息数组
  SENSOR_ID_T sensor_id_list[] = {
    {1, 0x40, I2C_ADDR_8, 0x0B, "F37"},        // F37
    {1, 0x1a, I2C_ADDR_16, 0x0000, "imx415"},  // imx415
    {1, 0x29, I2C_ADDR_16, 0x03f0, "GC4663"},  // GC4663
    {1, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {1, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {1, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {1, 0x1a, I2C_ADDR_16, 0x0000, "imx586"},  // imx586
    {1, 0x29, I2C_ADDR_16, 0x0000, "gc4c33"},  // gc4c33
  };
  std::vector<int> i2c_buss= {0,1,2,3,4,5,6};

  SENSOR_ID_T *sensor_ptr = nullptr;
  for (auto sensor_id : sensor_id_list) {
    if(strcasecmp(sensor_id.sensor_name, sensor.c_str()) == 0) {
      sensor_ptr = &sensor_id;
      break;
    }
  }
  bool sensor_flag = false;
  if (sensor_ptr) {
    if (board_config_m_.size() > 0) {
      for (auto board : board_config_m_) {
        std::vector<int>::iterator it = std::find(mipi_stoped_.begin(), mipi_stoped_.end(), board.second.mipi_host);
        if (it == mipi_stoped_.end()) {
           continue;
        }
        if (detectSensor(*sensor_ptr, board.second.i2c_bus)) {
          host = board.second.mipi_host;
          i2c_bus = board.second.i2c_bus;
          sensor_flag = true;
          return 0;
        }
      }
    } else {
      for (auto num : i2c_buss) {
        if (detectSensor(*sensor_ptr, num)) {
          // host = mipi_stoped_[0];
          i2c_bus = num;
          sensor_flag = true;
          return 0;
        }
      }
    }
  }
  if (board_config_m_.size() > 0) {
    for (auto board : board_config_m_) {
      if (board.second.mipi_host == host) {
        for (auto sensor_id : sensor_id_list) {
          if (detectSensor(sensor_id, board.second.i2c_bus)) {
            host = board.second.mipi_host;
            i2c_bus = board.second.i2c_bus;
            sensor = sensor_id.sensor_name;
            sensor_flag = true;
            return 0;
          }
        }
      }
    }
  }
  if (board_config_m_.size() > 0) {
    for (auto board : board_config_m_) {
      std::vector<int>::iterator it = std::find(mipi_stoped_.begin(), mipi_stoped_.end(), board.second.mipi_host);
      if (it == mipi_stoped_.end()) {
          continue;
      }
      for (auto sensor_id : sensor_id_list) {
        if (detectSensor(sensor_id, board.second.i2c_bus)) {
          host = board.second.mipi_host;
          i2c_bus = board.second.i2c_bus;
          sensor = sensor_id.sensor_name;
          sensor_flag = true;
          return 0;
        }
      }
    }
  }
  for (auto num : i2c_buss) {
    for (auto sensor_id : sensor_id_list) {
      if (detectSensor(sensor_id, num)) {
        // host = mipi_stoped_[0];
        i2c_bus = num;
        sensor = sensor_id.sensor_name;
        sensor_flag = true;
        return 0;
      }
    }
  }
  return -1;
}

std::shared_ptr<GdcBinBuf_ST> HobotMipiCapIml::get_gdc_bin(std::string gdc_bin_file) {
	int64_t alloc_flags = 0;
	int ret = 0;
	int offset = 0;
	char *cfg_buf = NULL;

	if (gdc_bin_file.empty()) {
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"gdc_bin_file is empty\n");
		return nullptr;
	}
	rcpputils::fs::path file_path = gdc_bin_file;
	if (!(rcpputils::fs::exists(file_path) && rcpputils::fs::is_regular_file(file_path))) {
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"The gdc bin file %s, isn't exist or is path\n", gdc_bin_file.c_str());
		return nullptr;
	}

	FILE *fp = fopen(gdc_bin_file.c_str(), "r");
	if (fp == NULL) {
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"The gdc bin file %s open failed\n", gdc_bin_file.c_str());
		return nullptr;
	}
	fseek(fp, 0, SEEK_END);
	long file_size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	cfg_buf = (char*)malloc(file_size);
	int n = fread(cfg_buf, 1, file_size, fp);
	if (n != file_size) {
        free(cfg_buf);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"Read file size failed\n");
        fclose(fp);
        return nullptr;
	}
	fclose(fp);

	hb_mem_common_buf_t *bin_buf = new hb_mem_common_buf_t;
	memset(bin_buf, 0, sizeof(hb_mem_common_buf_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
	ret = hb_mem_alloc_com_buf(file_size, alloc_flags, bin_buf);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        free(cfg_buf);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}

	memcpy(bin_buf->virt_addr, cfg_buf, file_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, file_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        free(cfg_buf);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	auto bin_buf_ptr = std::make_shared<GdcBinBuf_ST>();
	bin_buf_ptr->bin_buf = bin_buf;
	bin_buf_ptr->bin_buf_size = file_size;
    free(cfg_buf);
	return bin_buf_ptr;
}

double  width_tmp;
double  heigh_tmp;

static int save_gdc_bin = 0;

std::vector<std::shared_ptr<GdcBinBuf_ST>> HobotMipiCapIml::gen_gdc_bin_stereo(int in_width, int in_height,int out_width, int out_height,
		std::vector<sensor_msgs::msg::CameraInfo> &cam_info, std::vector<sensor_msgs::msg::CameraInfo> &cal_cam_info,
		double rotation, double cal_rotate, double cal_alpha, bool pre_rotation) {
	std::vector<std::shared_ptr<GdcBinBuf_ST>> gdc_bin_buf;
	if (in_width <= 0 || in_height<= 0 || out_width <= 0 || out_height <= 0 || cam_info.size() != 2) {
		return gdc_bin_buf;
	}
	if (!((rotation == 0.0) || (rotation == 90.0) || (rotation == 180.0) || (rotation == 270.0) ||
	   (cal_rotate == 0.0) || (cal_rotate == 90.0) || (cal_rotate == 180.0) || (cal_rotate == 270.0))) {
		return gdc_bin_buf;
	}

	float gdc_width_scale, gdc_height_scale;
	int in_gdc_width, in_gdc_height;

	double rotation_diff = rotation > cal_rotate ? rotation - cal_rotate : 360 + rotation - cal_rotate;
	int out_gdc_width, out_gdc_height;

	if (pre_rotation) {
		if ((rotation_diff == 90.0) || (rotation_diff == 270.0)) {
			in_gdc_width = in_height;
			in_gdc_height = in_width;
			out_gdc_width = out_height;
			out_gdc_height = out_width;

		} else {
			in_gdc_width = in_width;
			in_gdc_height = in_height;	
			out_gdc_width = out_width;
			out_gdc_height = out_height;	
		}	
	} else {
		if ((cal_rotate == 90.0) || (cal_rotate == 270.0)) {
			in_gdc_width = in_height;
			in_gdc_height = in_width;
		} else {
			in_gdc_width = in_width;
			in_gdc_height = in_height;	
		}

		if ((rotation_diff == 90.0) || (rotation_diff == 270.0)) {
			out_gdc_width = out_height;
			out_gdc_height = out_width;
		} else {
			out_gdc_width = out_width;
			out_gdc_height = out_height;	
		}	
	}


    // cam param
	cal_cam_info.clear();
    cv::Mat Rl, Rr, Pl, Pr, Q;
    cv::Mat Kl, Kr, Dl, Dr, R_rl, t_rl;
    cv::Mat undistmap1l, undistmap2l, undistmap1r, undistmap2r;
	gdc_width_scale = in_gdc_width / static_cast<float>(cam_info[0].width);
	gdc_height_scale = in_gdc_height / static_cast<float>(cam_info[0].height);

	Dl = cv::Mat(1, cam_info[0].d.size(), CV_64F, cam_info[0].d.data()).clone();
	Kl = cv::Mat(3, 3, CV_64F, cam_info[0].k.data()).clone();
	cv::Mat tRl = cv::Mat(3, 3, CV_64F, cam_info[0].r.data()).clone();
	cv::Mat tPl = cv::Mat(3, 4, CV_64F, cam_info[0].p.data()).clone();

	Dr = cv::Mat(1, cam_info[1].d.size(), CV_64F, cam_info[1].d.data()).clone();
	Kr = cv::Mat(3, 3, CV_64F, cam_info[1].k.data()).clone();
	cv::Mat tRr = cv::Mat(3, 3, CV_64F, cam_info[1].r.data()).clone();
	cv::Mat tPr = cv::Mat(3, 4, CV_64F, cam_info[1].p.data()).clone();

	R_rl = cv::Mat::zeros(3, 3, CV_64F);
	t_rl = cv::Mat::zeros(3, 1, CV_64F);

	cv::Mat Kr_inv = Kr.inv();
	cv::Mat RT = Kr_inv * tPr;
    cv::Mat tTr = RT(cv::Rect(3, 0, 1, 3)).clone();
	R_rl = tRr;
	t_rl = tTr;

	Kl.at<double>(0, 0) *= gdc_width_scale;
	Kl.at<double>(0, 2) *= gdc_width_scale;
	Kl.at<double>(1, 1) *= gdc_height_scale;
	Kl.at<double>(1, 2) *= gdc_height_scale;
	Kr.at<double>(0, 0) *= gdc_width_scale;
	Kr.at<double>(0, 2) *= gdc_width_scale;
	Kr.at<double>(1, 1) *= gdc_height_scale;
	Kr.at<double>(1, 2) *= gdc_height_scale;

	RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_cap"),"===stetreo calibration===" 
		<< "\ngdc_width_scale : " << gdc_width_scale
		<< "\ngdc_height_scale : " << gdc_height_scale
		<< "\nKl : \n" << Kl
		<< "\nDl : \n" << Dl
		<< "\nKr : \n" << Kr
		<< "\nDr : \n" << Dr
		<< "\nR_rl : \n" << R_rl
		<< "\nt_rl : \n" << t_rl
		<< "\n===================="
	);

	double target_hfov = 0.0; // 目标FOV（可从配置/外部输入）
	double actual_hfov_l, actual_hfov_r;
	// 调用核心函数计算alpha
	double alpha = computeStereoAlphaFromFOV(
    target_hfov,
    Kl, Dl, Kr, Dr, R_rl, t_rl,
    in_gdc_width, in_gdc_height,
    out_gdc_width, out_gdc_height,
    cam_info[0].distortion_model,
    actual_hfov_l, actual_hfov_r);
	//double alpha = 0.0;
	if (alpha <= 0) {
		alpha = 0;
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "Use default alpha=0.0 (target FOV invalid)");
	} else {
		// RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
        // 	"Auto compute alpha: %f\n , Left actual FOV: %f\n, Right actual FOV: %f\n ", alpha, actual_hfov_l, actual_hfov_r);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"), "Auto compute alpha: %f", alpha);
	}
	// TODO: Set alpha from config
	// cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height), R_rl, t_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 0.391 ,cv::Size(out_gdc_width, out_gdc_height));
	if (cam_info[0].distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
		double fov_scale = find_best_fov_scale(Kl, Dl, Kr, Dr, R_rl, t_rl, cv::Size(in_gdc_width, in_gdc_height), cv::Size(out_gdc_width, out_gdc_height));
		RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_cap"), "best fov scale: " << fov_scale);
		cv::fisheye::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height), R_rl, t_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, cv::Size(out_gdc_width, out_gdc_height), 0.0, fov_scale);
		cv::fisheye::initUndistortRectifyMap(Kl, Dl, Rl, Pl, cv::Size(out_gdc_width, out_gdc_height), CV_32FC1, undistmap1l, undistmap2l);
		cv::fisheye::initUndistortRectifyMap(Kr, Dr, Rr, Pr, cv::Size(out_gdc_width, out_gdc_height), CV_32FC1, undistmap1r, undistmap2r);
	} else {
		cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height), R_rl, t_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, alpha ,cv::Size(out_gdc_width, out_gdc_height));
		cv::initUndistortRectifyMap(Kl, Dl, Rl, Pl, cv::Size(out_gdc_width, out_gdc_height), CV_32FC1, undistmap1l, undistmap2l);
		cv::initUndistortRectifyMap(Kr, Dr, Rr, Pr, cv::Size(out_gdc_width, out_gdc_height), CV_32FC1, undistmap1r, undistmap2r);
	}
	int rotation_diff_int = rotation_diff;
	cv::Mat tmp;
	cv::Mat rotation_1l;
	cv::Mat rotation_2l;
	cv::Mat rotation_1r;
	cv::Mat rotation_2r;
    switch(rotation_diff_int) {	
        case 90:
    		cv::transpose(undistmap1l, tmp);
    		cv::flip(tmp, rotation_1l, 1); // 垂直翻转
    		cv::transpose(undistmap2l, tmp);
    		cv::flip(tmp, rotation_2l, 1); // 垂直翻转

    		cv::transpose(undistmap1r, tmp);
    		cv::flip(tmp, rotation_1r, 1); // 垂直翻转
    		cv::transpose(undistmap2r, tmp);
    		cv::flip(tmp, rotation_2r, 1); // 垂直翻转
            break;
        case 180:
			cv::flip(undistmap1l, rotation_1l, -1);
			cv::flip(undistmap2l, rotation_2l, -1);

			cv::flip(undistmap1r, rotation_1r, -1);
			cv::flip(undistmap2r, rotation_2r, -1);
			break;
        case 270:
    		cv::transpose(undistmap1l, tmp);
    		cv::flip(tmp, rotation_1l, 0); // 垂直翻转
    		cv::transpose(undistmap2l, tmp);
    		cv::flip(tmp, rotation_2l, 0); // 垂直翻转

			cv::transpose(undistmap1r, tmp);
    		cv::flip(tmp, rotation_1r, 0); // 垂直翻转
    		cv::transpose(undistmap2r, tmp);
    		cv::flip(tmp, rotation_2r, 0); // 垂直翻转
			break;
		default:
			rotation_1l = undistmap1l;
			rotation_2l = undistmap2l;

			rotation_1r = undistmap1r;
			rotation_2r = undistmap2r;
			break;
    }

	RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_cap"),"===Corrected parameters===" 
		<< "\nRl : \n" << Rl
		<< "\nRr : \n" << Rr
		<< "\nPl : \n" << Pl
		<< "\nPr : \n" << Pr
		<< "\n===================="
	);

	param_t gdc_param;
	memset(&gdc_param, 0, sizeof(param_t));
	gdc_param.format = FMT_SEMIPLANAR_420;
	gdc_param.in.w = in_width;
	gdc_param.in.h = in_height;
	gdc_param.out.w = out_width;
	gdc_param.out.h = out_height;
	gdc_param.x_offset = 0;
	gdc_param.y_offset = 0;
	gdc_param.diameter = in_height;
	gdc_param.fov = 180;

	window_t  wnds;
	memset(&wnds, 0, sizeof(window_t));
	wnds.strength = 1.0;
	wnds.strengthY = 1.0;
	wnds.angle = rotation;
	//wnds.angle = 0;
	wnds.elevation = 0;
	wnds.azimuth = 0;
	wnds.keep_ratio = 1;
	wnds.FOV_h = 90;
	wnds.FOV_w = 90;
	wnds.cylindricity_y = 0;
	wnds.cylindricity_x = 0;
	wnds.trapezoid_left_angle = 90;
	wnds.trapezoid_right_angle = 90;

	wnds.out_r.x = 0;
	wnds.out_r.y = 0;
	wnds.out_r.w = out_width;
	wnds.out_r.h = out_height;
	wnds.input_roi_r.x = 0;
	wnds.input_roi_r.y = 0;
	wnds.input_roi_r.w = in_width;
	wnds.input_roi_r.h = in_height;
	wnds.pan = 0;
	wnds.tilt = 0;
	wnds.zoom = 1;

	wnds.transform = CUSTOM;
	wnds.custom.full_tile_calc = 1;
	wnds.custom.tile_incr_x = 50;
	wnds.custom.tile_incr_y = 50;
	wnds.custom.w = out_width-1;
	wnds.custom.h = out_height-1;
	wnds.custom.centerx = out_width / 2 - 1;
	wnds.custom.centery = out_height / 2 - 1;

	std::vector<point_t> bin_map(out_width * out_height);
	width_tmp = in_width;
	heigh_tmp = in_height;
	int cal_rotate_int = pre_rotation ? rotation_diff : cal_rotate;
    switch(cal_rotate_int) {	
        case 90:
            std::transform(rotation_1l.ptr<float>(), rotation_1l.ptr<float>() + rotation_1l.total(),
				rotation_2l.ptr<float>(), bin_map.begin(),
				[](float x, float y) {
					point_t p;
					p.x = static_cast<double>(y);
					p.y = heigh_tmp - static_cast<double>(x)-1;
					p.x = p.x<0?0:p.x;
					p.y = p.y<0?0:p.y;
					return p;
				});
            break;
        case 180:
            std::transform(rotation_1l.ptr<float>(), rotation_1l.ptr<float>() + rotation_1l.total(),
				rotation_2l.ptr<float>(), bin_map.begin(),
				[](float x, float y) {
					point_t p;
					p.x = width_tmp - static_cast<double>(x)-1;
					p.y = heigh_tmp - static_cast<double>(y)-1;
					p.x = p.x<0?0:p.x;
					p.y = p.y<0?0:p.y;
					return p;
				});
			break;
        case 270:
			std::transform(rotation_1l.ptr<float>(), rotation_1l.ptr<float>() + rotation_1l.total(),
				rotation_2l.ptr<float>(), bin_map.begin(),
				[](float x, float y) {
					point_t p;
					p.x = width_tmp - static_cast<double>(y)-1;
					p.y = static_cast<double>(x);
					p.x = p.x<0?0:p.x;
					p.y = p.y<0?0:p.y;
					return p;
				});
			break;
		default:
			std::transform(rotation_1l.ptr<float>(), rotation_1l.ptr<float>() + rotation_1l.total(),
			rotation_2l.ptr<float>(), bin_map.begin(),
			[](float x, float y) {
				point_t p;
				p.x = static_cast<double>(x<0?0:x);
				p.y = static_cast<double>(y<0?0:y);
				return p;
			});
			break;
    }
	wnds.custom.points = bin_map.data();
	uint32_t *bin_buf_ptr = nullptr;
	uint64_t bin_buf_size;
	int64_t alloc_flags = 0;
	int offset = 0;
	auto ret = hbn_gen_gdc_bin(&gdc_param, &wnds, 1, (uint32_t**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_gen_gdc_bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"left_camera_gdc bin_buf_size = %d\n", bin_buf_size);
	if (save_gdc_bin) {
		std::ofstream outfile_;
		if (!outfile_.is_open()) {
			outfile_.open("./left_camera_gdc.bin", std::ios::app | std::ios::out | std::ios::binary);
		}
		if (outfile_.is_open()) {
			outfile_.write(reinterpret_cast<char *>(bin_buf_ptr), bin_buf_size);
		}
		outfile_.close();
	}
    hb_mem_common_buf_t *bin_buf = new hb_mem_common_buf_t;
	memset(bin_buf, 0, sizeof(hb_mem_common_buf_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
	ret = hb_mem_alloc_com_buf(bin_buf_size, alloc_flags, bin_buf);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        hbn_free_gdc_bin(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        hbn_free_gdc_bin(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
	hbn_free_gdc_bin(bin_buf_ptr);
	auto gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;
	gdc_bin_buf.push_back(gdc_bin_ptr);


    switch(cal_rotate_int) {	
        case 90:
            std::transform(rotation_1r.ptr<float>(), rotation_1r.ptr<float>() + rotation_1r.total(),
				rotation_2r.ptr<float>(), bin_map.begin(),
				[](float x, float y) {
					point_t p;
					p.x = static_cast<double>(y);
					p.y = heigh_tmp - static_cast<double>(x)-1;
					p.x = p.x<0?0:p.x;
					p.y = p.y<0?0:p.y;
					return p;
				});
            break;
        case 180:
            std::transform(rotation_1r.ptr<float>(), rotation_1r.ptr<float>() + rotation_1r.total(),
				rotation_2r.ptr<float>(), bin_map.begin(),
				[](float x, float y) {
					point_t p;
					p.x = width_tmp - static_cast<double>(x)-1;
					p.y = heigh_tmp - static_cast<double>(y)-1;
					p.x = p.x<0?0:p.x;
					p.y = p.y<0?0:p.y;
					return p;
				});
			break;
        case 270:
			std::transform(rotation_1r.ptr<float>(), rotation_1r.ptr<float>() + rotation_1r.total(),
				rotation_2r.ptr<float>(), bin_map.begin(),
				[](float x, float y) {
					point_t p;
					p.x = width_tmp - static_cast<double>(y)-1;
					p.y = static_cast<double>(x);
					p.x = p.x<0?0:p.x;
					p.y = p.y<0?0:p.y;
					return p;
				});
			break;
		default:
			std::transform(rotation_1r.ptr<float>(), rotation_1r.ptr<float>() + rotation_1r.total(),
			rotation_2r.ptr<float>(), bin_map.begin(),
			[](float x, float y) {
				point_t p;
				p.x = static_cast<double>(x<0?0:x);
				p.y = static_cast<double>(y<0?0:y);
				return p;
			});
			break;
    }

	wnds.custom.points = bin_map.data();
	bin_buf_ptr = nullptr;
	ret = hbn_gen_gdc_bin(&gdc_param, &wnds, 1, (uint32_t**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_gen_gdc_bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"right_camera_gdc bin_buf_size = %d\n", bin_buf_size);
	if (save_gdc_bin) {
		std::ofstream outfile_;
		if (!outfile_.is_open()) {
			outfile_.open("./right_camera_gdc.bin", std::ios::app | std::ios::out | std::ios::binary);
		}
		if (outfile_.is_open()) {
			outfile_.write(reinterpret_cast<char *>(bin_buf_ptr), bin_buf_size);
		}
		outfile_.close();
	}
    bin_buf = new hb_mem_common_buf_t;
	memset(bin_buf, 0, sizeof(hb_mem_common_buf_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
	ret = hb_mem_alloc_com_buf(bin_buf_size, alloc_flags, bin_buf);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        hbn_free_gdc_bin(bin_buf_ptr);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}

	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        hbn_free_gdc_bin(bin_buf_ptr);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
	hbn_free_gdc_bin(bin_buf_ptr);
	gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;
	gdc_bin_buf.push_back(gdc_bin_ptr);
	
	float camera_cx, camera_cy, camera_fx, camera_fy, base_line;
	camera_fx = Q.at<double>(2, 3);
	camera_fy = Q.at<double>(2, 3);
	camera_cx = -Q.at<double>(0, 3);
	camera_cy = -Q.at<double>(1, 3);
	base_line = std::abs(1 / Q.at<double>(3, 2));

	cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
	K.at<double>(0, 0) = camera_fx;
	K.at<double>(0, 2) = camera_cx;
	K.at<double>(1, 1) = camera_fy;
	K.at<double>(1, 2) = camera_cy;
	K.at<double>(2, 2) = 1;

	double tmp_t = 0;
    switch(rotation_diff_int) {	
        case 90:
		case 270:
			tmp_t = K.at<double>(0,0);
			K.at<double>(0,0) = K.at<double>(1,1);
			K.at<double>(1,1) = tmp_t;
			tmp_t = K.at<double>(0,2);
			K.at<double>(0,2) = out_height - K.at<double>(1,2);
			K.at<double>(1,2) = tmp_t;
            break;
		default:
			break;
    }

#if 0
	RT = cv::Mat::eye(3, 4, CV_64F);
	cv::Mat P = K * RT;

	sensor_msgs::msg::CameraInfo tmp_cam_info; 
	tmp_cam_info.width = out_width;
	tmp_cam_info.height = out_height;
	tmp_cam_info.d.resize(5, 0.0);
	memcpy(tmp_cam_info.k.data(), K.data, sizeof(tmp_cam_info.k));

	tmp_cam_info.r[0] = 1.0;
    tmp_cam_info.r[1] = 0.0;
    tmp_cam_info.r[2] = 0.0;
    tmp_cam_info.r[3] = 0.0;
    tmp_cam_info.r[4] = 1.0;
    tmp_cam_info.r[5] = 0.0;
    tmp_cam_info.r[6] = 0.0;
    tmp_cam_info.r[7] = 0.0;
    tmp_cam_info.r[8] = 1.0;

	memcpy(tmp_cam_info.p.data(), P.data, sizeof(tmp_cam_info.p));
	cal_cam_info.push_back(tmp_cam_info);

	RT.at<double>(0, 3) = base_line;
	P = K * RT;
	memcpy(tmp_cam_info.p.data(), P.data, sizeof(tmp_cam_info.p));
	cal_cam_info.push_back(tmp_cam_info);

#else
	cv::Mat S = cv::Mat::eye(3, 3, CV_64F);

    switch(rotation_diff_int) {	
        case 90:
			S.at<double>(0,0) = 0;
			S.at<double>(0,1) = -1;
			S.at<double>(0,2) = (double)(out_height - 1);
			S.at<double>(1,0) = 1;
			S.at<double>(1,1) = 0;
			S.at<double>(1,2) = 0;
			break;
		case 180:
			S.at<double>(0,0) = -1;
			S.at<double>(0,1) = 0;
			S.at<double>(0,2) = (double)(out_width - 1);
			S.at<double>(1,0) = 0;
			S.at<double>(1,1) = -1;
			S.at<double>(1,2) = (double)(out_height - 1);
			break;			
		case 270:
			S.at<double>(0,0) = 0;
			S.at<double>(0,1) = 1;
			S.at<double>(0,2) = 0;
			S.at<double>(1,0) = -1;
			S.at<double>(1,1) = 0;
			S.at<double>(1,2) = (double)(out_width - 1);
            break;
		default:
			break;
    }

	sensor_msgs::msg::CameraInfo tmp_cam_info; 
	tmp_cam_info.width = out_width;
	tmp_cam_info.height = out_height;
	tmp_cam_info.d.resize(5, 0.0);
	memcpy(tmp_cam_info.k.data(), K.data, sizeof(tmp_cam_info.k));
	cv::Mat new_Rl = S * Rl;
	memcpy(tmp_cam_info.r.data(), new_Rl.data, sizeof(tmp_cam_info.r));
	cv::Mat new_Pl = S * Pl;
	memcpy(tmp_cam_info.p.data(), new_Pl.data, sizeof(tmp_cam_info.p));
	cal_cam_info.push_back(tmp_cam_info);

	cv::Mat new_Rr = S * Rr;
	memcpy(tmp_cam_info.r.data(), new_Rr.data, sizeof(tmp_cam_info.r));
	cv::Mat new_Pr = S * Pr;
	memcpy(tmp_cam_info.p.data(), new_Pr.data, sizeof(tmp_cam_info.p));
	cal_cam_info.push_back(tmp_cam_info);
#endif


	RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_cap"),"===Corrected stetreo calibration ===" 
		<< "\nKl : \n" << Kl 
		<< "\nDl : \n" << Dl 
		<< "\nKr : \n" << Kr
		<< "\nDr : \n" << Dr
		<< "\nR_rl : \n" << R_rl
		<< "\nt_rl : \n" << t_rl
		<< "\ncalib file width, height : " << cam_info[0].width << "," << cam_info[0].height
		<< "\ngdc_width_scale, gdc_height_scale : " << gdc_width_scale << ", " << gdc_height_scale
		<< "\nrectify [f, cx, cy, baseline]: " << "[" << camera_fx << ", " << camera_cx << ", " << camera_cy << ", " << base_line << "]"
		<< "\n===================="
	);

	return gdc_bin_buf;
}

std::pair<double, double> HobotMipiCapIml::calculatePinholeFOV(const cv::Mat& K_rect, int width, int hight) {
	if (K_rect.type() != CV_64F || K_rect.rows != 3 || K_rect.cols != 3) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"), "Invalid Camera matrix!");
		return {0.0, 0.0};
	}
	double fx = K_rect.at<double>(0, 0);
	double fy = K_rect.at<double>(1, 1);
	double h_fov = 2 * atan2(static_cast<double>(width)/2, fx) * 180.0 / CV_PI;
	double v_fov = 2 * atan2(static_cast<double>(hight)/2, fy) * 180.0 / CV_PI;
	return {h_fov, v_fov};
}

std::pair<double, double> HobotMipiCapIml::calculateFisheyeFOV(const cv::Mat& K_rect, int width, int hight) {
	return calculatePinholeFOV(K_rect, width, hight);
}

double HobotMipiCapIml::computeInitAlpha(double target_hfov, double fov_min, double fov_max) {
	if (target_hfov <= fov_min - 1e-3) return 0.0;
    if (target_hfov >= fov_max + 1e-3) return 1.0;
    return (target_hfov - fov_min) / (fov_max - fov_min);
}

double HobotMipiCapIml::computeStereoAlphaFromFOV(
    double target_hfov,
    const cv::Mat& Kl, const cv::Mat& Dl,
    const cv::Mat& Kr, const cv::Mat& Dr,
    const cv::Mat& R_rl, const cv::Mat& t_rl,
    int in_gdc_width, int in_gdc_height,
    int out_gdc_width, int out_gdc_height,
    const std::string& distortion_model,
    double& actual_hfov_l, double& actual_hfov_r) {

	// ------------ 步骤1：计算alpha=0时的双目FOV（FOV_min） ------------
    cv::Mat Rl0, Rr0, Pl0, Pr0, Q0;
    if (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
        cv::fisheye::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                                   R_rl, t_rl, Rl0, Rr0, Pl0, Pr0, Q0,
                                   cv::CALIB_ZERO_DISPARITY, cv::Size(out_gdc_width, out_gdc_height), 0.0);
    } else {
        cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                          R_rl, t_rl, Rl0, Rr0, Pl0, Pr0, Q0,
                          cv::CALIB_ZERO_DISPARITY, 0.0, cv::Size(out_gdc_width, out_gdc_height));
    }
    // 提取校正后内参（投影矩阵Pl/Pr的前3x3）
    cv::Mat K_l0 = Pl0(cv::Rect(0,0,3,3)).clone();
    cv::Mat K_r0 = Pr0(cv::Rect(0,0,3,3)).clone();
    // 计算alpha=0时的FOV
    auto [fov_l0, _] = (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) 
                        ? calculateFisheyeFOV(K_l0, out_gdc_width, out_gdc_height)
                        : calculatePinholeFOV(K_l0, out_gdc_width, out_gdc_height);
    auto [fov_r0, __] = (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT)
                        ? calculateFisheyeFOV(K_r0, out_gdc_width, out_gdc_height)
                        : calculatePinholeFOV(K_r0, out_gdc_width, out_gdc_height);

	// ------------ 步骤2：计算alpha=1时的双目FOV（FOV_max） ------------
    cv::Mat Rl1, Rr1, Pl1, Pr1, Q1;
    if (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
        cv::fisheye::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                                   R_rl, t_rl, Rl1, Rr1, Pl1, Pr1, Q1,
                                   cv::CALIB_ZERO_DISPARITY, cv::Size(out_gdc_width, out_gdc_height), 1.0);
    } else {
        cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                          R_rl, t_rl, Rl1, Rr1, Pl1, Pr1, Q1,
                          cv::CALIB_ZERO_DISPARITY, 1.0, cv::Size(out_gdc_width, out_gdc_height));
    }
    cv::Mat K_l1 = Pl1(cv::Rect(0,0,3,3)).clone();
    cv::Mat K_r1 = Pr1(cv::Rect(0,0,3,3)).clone();
    auto [fov_l1, ___] = (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT)
                        ? calculateFisheyeFOV(K_l1, out_gdc_width, out_gdc_height)
                        : calculatePinholeFOV(K_l1, out_gdc_width, out_gdc_height);
    auto [fov_r1, ____] = (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT)
                        ? calculateFisheyeFOV(K_r1, out_gdc_width, out_gdc_height)
                        : calculatePinholeFOV(K_r1, out_gdc_width, out_gdc_height);

	// ------------ 步骤3：确定双目FOV的有效交集 ------------
    double fov_min = std::max(fov_l0, fov_r0); // 双目最小FOV（取较大值）
    double fov_max = std::min(fov_l1, fov_r1); // 双目最大FOV（取较小值）
    if (target_hfov < fov_min - 1e-3 || target_hfov > fov_max + 1e-3) {
        RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
            "Target FOV %.2f° out of valid range [%.2f°, %.2f°]",
            target_hfov, fov_min, fov_max);
        actual_hfov_l = actual_hfov_r = 0.0;
        return -1.0;
    }

	// ------------ 步骤4：迭代微调alpha（保证FOV精度） ------------
    const double eps = 3.0; // FOV允许误差（°）
    const int max_iter = 10; // 最大迭代次数
    double alpha = computeInitAlpha(target_hfov, fov_min, fov_max);
    double left_alpha = 0.0, right_alpha = 1.0;
    int iter = 0;

    while (iter < max_iter) {
        // 用当前alpha计算校正后FOV
        cv::Mat Rl, Rr, Pl, Pr, Q;
        if (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
            cv::fisheye::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                                       R_rl, t_rl, Rl, Rr, Pl, Pr, Q,
                                       cv::CALIB_ZERO_DISPARITY, cv::Size(out_gdc_width, out_gdc_height), alpha);
        } else {
            cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                              R_rl, t_rl, Rl, Rr, Pl, Pr, Q,
                              cv::CALIB_ZERO_DISPARITY, alpha, cv::Size(out_gdc_width, out_gdc_height));
        }
        cv::Mat K_l = Pl(cv::Rect(0,0,3,3)).clone();
        cv::Mat K_r = Pr(cv::Rect(0,0,3,3)).clone();
        auto [hfov_l, _] = (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT)
                            ? calculateFisheyeFOV(K_l, out_gdc_width, out_gdc_height)
                            : calculatePinholeFOV(K_l, out_gdc_width, out_gdc_height);
        auto [hfov_r, __] = (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT)
                            ? calculateFisheyeFOV(K_r, out_gdc_width, out_gdc_height)
                            : calculatePinholeFOV(K_r, out_gdc_width, out_gdc_height);
        
        // 验证双目FOV是否接近目标
        double avg_hfov = (hfov_l + hfov_r) / 2;
        if (fabs(avg_hfov - target_hfov) < eps) {
            actual_hfov_l = hfov_l;
            actual_hfov_r = hfov_r;
            return alpha;
        }

        // 二分法调整alpha
        if (avg_hfov < target_hfov) {
            left_alpha = alpha;
        } else {
            right_alpha = alpha;
        }
        alpha = (left_alpha + right_alpha) / 2;
        iter++;
    }

	// ------------ 步骤5：返回最终alpha并输出实际FOV ------------
    cv::Mat Rl_final, Rr_final, Pl_final, Pr_final, Q_final;
    if (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
        cv::fisheye::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                                   R_rl, t_rl, Rl_final, Rr_final, Pl_final, Pr_final, Q_final,
                                   cv::CALIB_ZERO_DISPARITY, cv::Size(out_gdc_width, out_gdc_height), alpha);
    } else {
        cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                          R_rl, t_rl, Rl_final, Rr_final, Pl_final, Pr_final, Q_final,
                          cv::CALIB_ZERO_DISPARITY, alpha, cv::Size(out_gdc_width, out_gdc_height));
    }
    cv::Mat K_l_final = Pl_final(cv::Rect(0,0,3,3)).clone();
    cv::Mat K_r_final = Pr_final(cv::Rect(0,0,3,3)).clone();
    actual_hfov_l = (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT)
                    ? calculateFisheyeFOV(K_l_final, out_gdc_width, out_gdc_height).first
                    : calculatePinholeFOV(K_l_final, out_gdc_width, out_gdc_height).first;
    actual_hfov_r = (distortion_model == sensor_msgs::distortion_models::EQUIDISTANT)
                    ? calculateFisheyeFOV(K_r_final, out_gdc_width, out_gdc_height).first
                    : calculatePinholeFOV(K_r_final, out_gdc_width, out_gdc_height).first;

    return alpha;										
}

double HobotMipiCapIml::find_best_fov_scale(const cv::Mat& Kl, const cv::Mat& Dl,
                                            const cv::Mat& Kr, const cv::Mat& Dr,
                                            const cv::Mat& R_rl, const cv::Mat& t_rl,
                                            cv::Size in_size,
                                            cv::Size out_size) {
  double best_scale = 1.0;
  for (double fov = 1.0; fov >= 0.3; fov -= 0.02) {
	cv::Mat Rl, Rr, Pl, Pr, Q;
	cv::fisheye::stereoRectify(
		Kl, Dl, Kr, Dr,
		in_size,
		R_rl, t_rl,
		Rl, Rr, Pl, Pr, Q,
		cv::CALIB_ZERO_DISPARITY,
		out_size,
		0.0,
		fov);
    cv::Mat mapx_l, mapy_l;
    cv::Mat mapx_r, mapy_r;
    cv::fisheye::initUndistortRectifyMap(Kl, Dl, Rl, Pl, out_size, CV_32FC1, mapx_l, mapy_l);
    cv::fisheye::initUndistortRectifyMap(Kr, Dr, Rr, Pr, out_size, CV_32FC1, mapx_r, mapy_r);
    int invalid = 0;
    for (int y = 0; y < mapx_l.rows; y++) {
        const float* ptrx_l = mapx_l.ptr<float>(y);
        const float* ptry_l = mapy_l.ptr<float>(y);
        const float* ptrx_r = mapx_r.ptr<float>(y);
        const float* ptry_r = mapy_r.ptr<float>(y);
        for (int x = 0; x < mapx_l.cols; x++) {
            float sx_l = ptrx_l[x];
            float sy_l = ptry_l[x];
            float sx_r = ptrx_r[x];
            float sy_r = ptry_r[x];
            if (sx_l < 1 || sx_l >= in_size.width - 2 ||
                sy_l < 1 || sy_l >= in_size.height - 2 ||
                sx_r < 1 || sx_r >= in_size.width - 2 ||
                sy_r < 1 || sy_r >= in_size.height - 2) {
                    invalid++;
                }
            }
        }

        if (invalid == 0) {
            best_scale = fov;
            break;
        }
    }

    return best_scale;
}

std::shared_ptr<GdcBinBuf_ST> HobotMipiCapIml::gen_gdc_bin(int in_width, int in_height,int out_width, int out_height,
       sensor_msgs::msg::CameraInfo *cam_info, sensor_msgs::msg::CameraInfo *cal_cam_info,
	   double rotation, double cal_rotate, double cal_alpha, bool pre_rotation) {
	if (in_width <= 0 || in_height<= 0 || out_width <= 0 || out_height <= 0 ||  cam_info == nullptr || cal_cam_info == nullptr) {
		return nullptr;
	}
	cv::Mat rot_mat;

	if (!((rotation == 0.0) || (rotation == 90.0) || (rotation == 180.0) || (rotation == 270.0) ||
	   (cal_rotate == 0.0) || (cal_rotate == 90.0) || (cal_rotate == 180.0) || (cal_rotate == 270.0))) {
		return nullptr;
	}
	float gdc_width_scale, gdc_height_scale;
	int in_gdc_width, in_gdc_height;

	double rotation_diff = rotation > cal_rotate ? rotation - cal_rotate : 360 + rotation - cal_rotate;
	int out_gdc_width, out_gdc_height;

	if (pre_rotation) {
		if ((rotation_diff == 90.0) || (rotation_diff == 270.0)) {
			in_gdc_width = in_height;
			in_gdc_height = in_width;
			out_gdc_width = out_height;
			out_gdc_height = out_width;

		} else {
			in_gdc_width = in_width;
			in_gdc_height = in_height;	
			out_gdc_width = out_width;
			out_gdc_height = out_height;	
		}	

	} else {
		if ((cal_rotate == 90.0) || (cal_rotate == 270.0)) {
			in_gdc_width = in_height;
			in_gdc_height = in_width;
		} else {
			in_gdc_width = in_width;
			in_gdc_height = in_height;	
		}

		if ((rotation_diff == 90.0) || (rotation_diff == 270.0)) {
			out_gdc_width = out_height;
			out_gdc_height = out_width;
		} else {
			out_gdc_width = out_width;
			out_gdc_height = out_height;	
		}	
	}

	gdc_width_scale = in_gdc_width / static_cast<float>(cam_info->width);
	gdc_height_scale = in_gdc_height / static_cast<float>(cam_info->height);
	cv::Mat K, D, R, T, P;
	D = cv::Mat(1, cam_info->d.size(), CV_64F, cam_info->d.data()).clone();
	K = cv::Mat(3, 3, CV_64F, cam_info->k.data()).clone();
	R = cv::Mat(3, 3, CV_64F, cam_info->r.data()).clone();
	P = cv::Mat(3, 4, CV_64F, cam_info->p.data()).clone();
	cv::Mat K_inv = K.inv();
	cv::Mat RT = K_inv * P;
	T = RT(cv::Rect(3, 0, 1, 3)).clone();
	K.at<double>(0, 0) *= gdc_width_scale;
	K.at<double>(0, 2) *= gdc_width_scale;
	K.at<double>(1, 1) *= gdc_height_scale;
	K.at<double>(1, 2) *= gdc_height_scale;	


	RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_cap"),"===stetreo calibration===" 
		<< "\ngdc_width_scale : " << gdc_width_scale
		<< "\ngdc_height_scale : " << gdc_height_scale
		<< "\nK : \n" << K
		<< "\nD : \n" << D
		<< "\nR : \n" << R
		<< "\nT : \n" << T
		<< "\n===================="
	);

    param_t gdc_param;
	memset(&gdc_param, 0, sizeof(param_t));
	gdc_param.format = FMT_SEMIPLANAR_420;
	gdc_param.in.w = in_width;
	gdc_param.in.h = in_height;
	gdc_param.out.w = out_width;
	gdc_param.out.h = out_height;
	gdc_param.x_offset = 0;
	gdc_param.y_offset = 0;
	gdc_param.diameter = in_height;
	gdc_param.fov = 180;

	window_t  wnds;
	memset(&wnds, 0, sizeof(window_t));
	wnds.strength = 1.0;
	wnds.strengthY = 1.0;
	wnds.angle = rotation;
	wnds.elevation = 0;
	wnds.azimuth = 0;
	wnds.keep_ratio = 1;
	wnds.FOV_h = 90;
	wnds.FOV_w = 90;
	wnds.cylindricity_y = 0;
	wnds.cylindricity_x = 0;
	wnds.trapezoid_left_angle = 90;
	wnds.trapezoid_right_angle = 90;

	wnds.out_r.x = 0;
	wnds.out_r.y = 0;
	wnds.out_r.w = out_width;
	wnds.out_r.h = out_height;
	wnds.input_roi_r.x = 0;
	wnds.input_roi_r.y = 0;
	wnds.input_roi_r.w = in_width;
	wnds.input_roi_r.h = in_height;
	wnds.pan = 0;
	wnds.tilt = 0;
	wnds.zoom = 1;
	wnds.transform = CUSTOM;
	wnds.custom.full_tile_calc = 1;
	wnds.custom.tile_incr_x = 50;
	wnds.custom.tile_incr_y = 50;
	wnds.custom.w = out_width - 1;
	wnds.custom.h = out_height - 1;
	wnds.custom.centerx = out_width / 2 - 1;
	wnds.custom.centery = out_height / 2 - 1;

	double alpha = 0.0;
	if (cal_alpha <= 1.0) {
		alpha = cal_alpha;
	}

	cv::Mat undistmap1l, undistmap2l;
	cv::Mat new_K;
	if (cam_info->distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
		double balance = alpha;
		cv::Mat tmp_P;
		cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, cv::Size(in_width, in_height), cv::Matx33d::eye(), tmp_P, alpha, cv::Size(out_gdc_width, out_gdc_height));
		new_K = tmp_P(cv::Rect(0, 0, 3, 3)).clone();
	} else {
		new_K = cv::getOptimalNewCameraMatrix(K, D, cv::Size(in_width, in_height), alpha, cv::Size(out_gdc_width, out_gdc_height), nullptr, true);
	}

	
	cv::initUndistortRectifyMap(K, D, cv::Mat(), new_K, cv::Size(out_gdc_width, out_gdc_height), CV_32FC1, undistmap1l, undistmap2l);
	

	int rotation_diff_int = rotation_diff;
	cv::Mat tmp;
	cv::Mat rotation_1;
	cv::Mat rotation_2;
    switch(rotation_diff_int) {	
        case 90:
    		cv::transpose(undistmap1l, tmp);
    		cv::flip(tmp, rotation_1, 1); // 垂直翻转
    		cv::transpose(undistmap2l, tmp);
    		cv::flip(tmp, rotation_2, 1); // 垂直翻转
            break;
        case 180:
			cv::flip(undistmap1l, rotation_1, -1);
			cv::flip(undistmap2l, rotation_2, -1);
			break;
        case 270:
    		cv::transpose(undistmap1l, tmp);
    		cv::flip(tmp, rotation_1, 0); // 垂直翻转
    		cv::transpose(undistmap2l, tmp);
    		cv::flip(tmp, rotation_2, 0); // 垂直翻转
			break;
		default:
			rotation_1 = undistmap1l;
			rotation_2 = undistmap2l;
			break;
    }
	std::vector<point_t> bin_map(out_width * out_height);
	width_tmp = in_width;
	heigh_tmp = in_height;
	int cal_rotate_int = pre_rotation ? rotation_diff : cal_rotate;
    switch(cal_rotate_int) {	
        case 90:
            std::transform(rotation_1.ptr<float>(), rotation_1.ptr<float>() + rotation_1.total(),
				rotation_2.ptr<float>(), bin_map.begin(),
				[](float x, float y) {
					point_t p;
					p.x = static_cast<double>(y);
					p.y = heigh_tmp - static_cast<double>(x)-1;
					p.x = p.x<0?0:p.x;
					p.y = p.y<0?0:p.y;
					return p;
				});
            break;
        case 180:
            std::transform(rotation_1.ptr<float>(), rotation_1.ptr<float>() + rotation_1.total(),
				rotation_2.ptr<float>(), bin_map.begin(),
				[](float x, float y) {
					point_t p;
					p.x = width_tmp - static_cast<double>(x)-1;
					p.y = heigh_tmp - static_cast<double>(y)-1;
					p.x = p.x<0?0:p.x;
					p.y = p.y<0?0:p.y;
					return p;
				});
			break;
        case 270:
			std::transform(rotation_1.ptr<float>(), rotation_1.ptr<float>() + rotation_1.total(),
				rotation_2.ptr<float>(), bin_map.begin(),
				[](float x, float y) {
					point_t p;
					p.x = width_tmp - static_cast<double>(y)-1;
					p.y = static_cast<double>(x);
					p.x = p.x<0?0:p.x;
					p.y = p.y<0?0:p.y;
					return p;
				});
			break;
		default:
			std::transform(rotation_1.ptr<float>(), rotation_1.ptr<float>() + rotation_1.total(),
			rotation_2.ptr<float>(), bin_map.begin(),
			[](float x, float y) {
				point_t p;
				p.x = static_cast<double>(x<0?0:x);
				p.y = static_cast<double>(y<0?0:y);
				return p;
			});
			break;
    }


	wnds.custom.points = bin_map.data();

#if 0
	std::ofstream outFile("./custom_config.txt");
    // 检查文件是否成功打开
    if (outFile) {
		std::ostringstream stream;
		stream << "1" << std::endl;
		stream << "50 50" << std::endl;
		stream << out_height << " " << out_width << std::endl;
		stream << wnds.custom.centery << " " << wnds.custom.centerx << std::endl;
		point_t *tmp_ptr = bin_map.data();
		for (int i = 0; i < out_height; i++) {
			for (int j = 0; j < out_width; j++) {
				stream << tmp_ptr->y << ":" << tmp_ptr->x << " ";
				tmp_ptr++;
			}
			stream << std::endl;
		}
		outFile << stream.str();
		outFile.close();
    }
#endif

	uint32_t *bin_buf_ptr = nullptr;
	uint64_t bin_buf_size;
	int64_t alloc_flags = 0;
	int offset = 0;

	auto ret = hbn_gen_gdc_bin(&gdc_param, &wnds, 1, (uint32_t**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_gen_gdc_bin failed, ret = %d\n", ret);
		return nullptr;
	}

    hb_mem_common_buf_t *bin_buf = new hb_mem_common_buf_t;
	memset(bin_buf, 0, sizeof(hb_mem_common_buf_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
	ret = hb_mem_alloc_com_buf(bin_buf_size, alloc_flags, bin_buf);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        hbn_free_gdc_bin(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        hbn_free_gdc_bin(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	hbn_free_gdc_bin(bin_buf_ptr);
	auto gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;

	cal_cam_info->width = out_width;
    cal_cam_info->height = out_height;
    cal_cam_info->d.resize(cam_info->d.size(),0.0);


	double tmp_t = 0;
    switch(rotation_diff_int) {	
        case 90:
		case 270:
			tmp_t = new_K.at<double>(0,0);
			new_K.at<double>(0,0) = new_K.at<double>(1,1);
			new_K.at<double>(1,1) = tmp_t;
			tmp_t = new_K.at<double>(0,2);
			new_K.at<double>(0,2) = out_height - new_K.at<double>(1,2);
			new_K.at<double>(1,2) = tmp_t;
            break;
		default:
			break;
    }

	// new_K.at<double>(0, 0) *= out_width_scale;
	// new_K.at<double>(0, 2) *= out_width_scale;
	// new_K.at<double>(1, 1) *= out_height_scale;
	// new_K.at<double>(1, 2) *= out_height_scale;	
	std::copy(new_K.ptr<double>(0), new_K.ptr<double>(0) + new_K.total(), cal_cam_info->k.begin());
    
	cal_cam_info->r[0] = 1.0;
    cal_cam_info->r[1] = 0.0;
    cal_cam_info->r[2] = 0.0;
    cal_cam_info->r[3] = 0.0;
    cal_cam_info->r[4] = 1.0;
    cal_cam_info->r[5] = 0.0;
    cal_cam_info->r[6] = 0.0;
    cal_cam_info->r[7] = 0.0;
    cal_cam_info->r[8] = 1.0;

	RT = cv::Mat::eye(3, 4, CV_64F);
	cv::Mat new_P = new_K * RT;
	std::copy(new_P.ptr<double>(0), new_P.ptr<double>(0) + new_P.total(), cal_cam_info->p.begin());
	return gdc_bin_ptr;
}


std::shared_ptr<GdcBinBuf_ST> HobotMipiCapIml::gen_gdc_bin_rotation(int gdc_width, int gdc_height,int out_width, int out_height,
       double rotation) {
	if (gdc_width <= 0 || gdc_height<= 0 || out_width <= 0 || out_height <= 0) {
		return nullptr;
	}
	//int out_width = 0, out_height = 0;
	if ((rotation == 90.0) ||(rotation == 270.0)) {
		out_width = gdc_height;
		out_height = gdc_width;
	} else if (rotation == 180.0) {
		out_width = gdc_width;
		out_height = gdc_height;
	} else {
		return nullptr;
	}
	RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_cap"), "gen_gdc_bin_rotation---gdc_width:"<<gdc_width<<",gdc_height:"<<gdc_height<<",out_width:"<<out_width<<",out_height:"<<out_height<<",rotation:"<<rotation<<std::endl);
    param_t gdc_param;
	memset(&gdc_param, 0, sizeof(param_t));
	gdc_param.format = FMT_SEMIPLANAR_420;
	gdc_param.in.w = gdc_width;
	gdc_param.in.h = gdc_height;
	gdc_param.out.w = out_width;
	gdc_param.out.h = out_height;
	gdc_param.x_offset = 0;
	gdc_param.y_offset = 0;
	gdc_param.diameter = gdc_height;
	gdc_param.fov = 180;

	window_t  wnds;
	memset(&wnds, 0, sizeof(window_t));
	wnds.strength = 1.0;
	wnds.strengthY = 1.0;
	wnds.angle = rotation;
	wnds.elevation = 0;
	wnds.azimuth = 0;
	wnds.keep_ratio = 1;
	wnds.FOV_h = 90;
	wnds.FOV_w = 90;
	wnds.cylindricity_y = 0;
	wnds.cylindricity_x = 0;
	wnds.trapezoid_left_angle = 90;
	wnds.trapezoid_right_angle = 90;

	wnds.out_r.x = 0;
	wnds.out_r.y = 0;
	wnds.out_r.w = out_width;
	wnds.out_r.h = out_height;
	wnds.input_roi_r.x = 0;
	wnds.input_roi_r.y = 0;
	wnds.input_roi_r.w = gdc_width;
	wnds.input_roi_r.h = gdc_height;
	wnds.pan = 0;
	wnds.tilt = 0;
	wnds.zoom = 1;
	wnds.transform = AFFINE;

	uint32_t *bin_buf_ptr = nullptr;
	uint64_t bin_buf_size;
	int64_t alloc_flags = 0;
	int offset = 0;

	auto ret = hbn_gen_gdc_bin(&gdc_param, &wnds, 1, (uint32_t**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_gen_gdc_bin failed, ret = %d\n", ret);
		return nullptr;
	}

    hb_mem_common_buf_t *bin_buf = new hb_mem_common_buf_t;
	memset(bin_buf, 0, sizeof(hb_mem_common_buf_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
	ret = hb_mem_alloc_com_buf(bin_buf_size, alloc_flags, bin_buf);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        hbn_free_gdc_bin(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        hbn_free_gdc_bin(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	hbn_free_gdc_bin(bin_buf_ptr);
	auto gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;
	return gdc_bin_ptr;
}

std::shared_ptr<GdcBinBuf_ST> HobotMipiCapIml::gen_gdc_bin_json(std::string file) {
	uint32_t *bin_buf_ptr = nullptr;
	uint64_t bin_buf_size;
	int64_t alloc_flags = 0;
	int offset = 0;
	
	auto ret = hbn_gen_gdc_bin_json(file.c_str(), NULL, (uint32_t**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0|| bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_gen_gdc_bin_json failed, ret = %d\n", ret);
		return nullptr;
	}

	hb_mem_common_buf_t *bin_buf = new hb_mem_common_buf_t;
	memset(bin_buf, 0, sizeof(hb_mem_common_buf_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
	ret = hb_mem_alloc_com_buf(bin_buf_size, alloc_flags, bin_buf);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        hbn_free_gdc_bin(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        hbn_free_gdc_bin(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	hbn_free_gdc_bin(bin_buf_ptr);
	auto gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;
	return gdc_bin_ptr;
}

}  // namespace mipi_cam
