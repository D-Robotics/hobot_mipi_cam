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
#include "hbn_isp_api.h"

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
	if (cap_info_.stream_mode_ == 0)
	{
		for (auto cal_info : cal_cam_info_)
		{
			cal_cam_info_sub_.push_back(cal_info);
		}
	}
	else
	{
		for (auto cal_info : cam_info_)
		{
			cal_cam_info_sub_.push_back(cal_info);
		}
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
  if (pipe_contex.size() > 1 &&
      (pipe_contex[0].cap_info_->sync_awb_
          || pipe_contex[0].cap_info_->sync_ccm_
		  || pipe_contex[0].cap_info_->sync_ae_
          || pipe_contex[0].cap_info_->print_isp_log_)) {
    awb_ae_sync_task_ = std::make_shared<std::thread>(
        std::bind(&HobotMipiCapIml::sync_awb_ae_task, this));
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
  // Safety cleanup: ensure threads are joined before destruction
  started_ = false;
  if (multi_frame_task_ && multi_frame_task_->joinable()) {
    multi_frame_task_->join();
  }
  if (sync_task_ && sync_task_->joinable()) {
    sync_task_->join();
  }
  if (sub_multi_frame_task_ && sub_multi_frame_task_->joinable()) {
    sub_multi_frame_task_->join();
  }
  if (sub_sync_task_ && sub_sync_task_->joinable()) {
    sub_sync_task_->join();
  }
  if (awb_ae_sync_task_ && awb_ae_sync_task_->joinable()) {
    awb_ae_sync_task_->join();
  }
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
	if ((channel == "sub_single") || (channel == "sub_left") || (channel == "sub_right") || (channel == "sub_combine")) {
		loop = (1000 / cap_info_.sub_fps + 100) / 10;
	}
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
	int vse_fps = pipe_contex->cap_info_->fps == 1.0 ? 1 : ceil(pipe_contex->cap_info_->fps * 30 / pipe_contex->sensor_config.camera_config->fps);
	int sub_vse_fps = pipe_contex->cap_info_->sub_fps == 1.0 ? 1 : ceil(pipe_contex->cap_info_->sub_fps * 30 / pipe_contex->sensor_config.camera_config->fps);
	if (pipe_contex->cap_info_->lpwm_enable_) {
		vse_ochn_attr.fps.src = 0;
		vse_ochn_attr.fps.dst = 0;
	} else {
		vse_ochn_attr.fps.src = 0;
		vse_ochn_attr.fps.dst = vse_fps;
	}

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
		
			if (pipe_contex->cap_info_->lpwm_enable_) {
				sub_vse_ochn_attr.fps.src = 0;
				sub_vse_ochn_attr.fps.dst = sub_vse_fps;
			} else {
				sub_vse_ochn_attr.fps.src = 0;
				sub_vse_ochn_attr.fps.dst = sub_vse_fps;
			}

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
		
			if (pipe_contex->cap_info_->lpwm_enable_) {
				sub_vse_ochn_attr.fps.src = 0;
				sub_vse_ochn_attr.fps.dst = sub_vse_fps;
			} else {
				sub_vse_ochn_attr.fps.src = 0;
				sub_vse_ochn_attr.fps.dst = sub_vse_fps;
			}
		
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
		//pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
		//pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
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
    pdata.awb_data[0].color_temperature = COLOR_TEMPERATURE_3100K; 
    pdata.awb_data[0].r =   0;        //pipe_contex->awb_otp_data.awb_data[0].r;
    pdata.awb_data[0].gr =  0;           //pipe_contex->awb_otp_data.awb_data[0].gr;
    pdata.awb_data[0].gb =  0;                  //pipe_contex->awb_otp_data.awb_data[0].gb;
    pdata.awb_data[0].b =  0;                    //pipe_contex->awb_otp_data.awb_data[0].b;
    pdata.awb_data[0].rg_ratio = pipe_contex->awb_otp_data->awb_info_l_r_.rg_ratio_3100K;
    pdata.awb_data[0].bg_ratio = pipe_contex->awb_otp_data->awb_info_l_r_.bg_ratio_3100K;

    pdata.awb_golden_data[0].color_temperature = COLOR_TEMPERATURE_3100K;
    pdata.awb_golden_data[0].r = 0;
    pdata.awb_golden_data[0].gr = 0;
    pdata.awb_golden_data[0].gb = 0;
    pdata.awb_golden_data[0].b = 0;
    pdata.awb_golden_data[0].rg_ratio = pipe_contex->awb_otp_data->awb_info_golden_.rg_ratio_3100K;
    pdata.awb_golden_data[0].bg_ratio = pipe_contex->awb_otp_data->awb_info_golden_.bg_ratio_3100K;

    // 配置4000K色温参数
    pdata.awb_data[1].color_temperature = COLOR_TEMPERATURE_4000K;
    pdata.awb_data[1].r =    0;             //pipe_contex->awb_otp_data.awb_data[0].r;
    pdata.awb_data[1].gr =     0;         //pipe_contex->awb_otp_data.awb_data[0].gr;
    pdata.awb_data[1].gb =   0;                //pipe_contex->awb_otp_data.awb_data[0].gb;
    pdata.awb_data[1].b =   0;               //pipe_contex->awb_otp_data.awb_data[0].b;
    pdata.awb_data[1].rg_ratio = pipe_contex->awb_otp_data->awb_info_l_r_.rg_ratio_4000K;
    pdata.awb_data[1].bg_ratio = pipe_contex->awb_otp_data->awb_info_l_r_.bg_ratio_4000K;

    pdata.awb_golden_data[1].color_temperature = COLOR_TEMPERATURE_4000K;
    pdata.awb_golden_data[1].r = 0;
    pdata.awb_golden_data[1].gr = 0;
    pdata.awb_golden_data[1].gb = 0;
    pdata.awb_golden_data[1].b = 0;
    pdata.awb_golden_data[1].rg_ratio = pipe_contex->awb_otp_data->awb_info_golden_.rg_ratio_4000K;
    pdata.awb_golden_data[1].bg_ratio = pipe_contex->awb_otp_data->awb_info_golden_.bg_ratio_4000K;

    // 配置5800K色温参数
    pdata.awb_data[2].color_temperature = COLOR_TEMPERATURE_5800K;
    pdata.awb_data[2].r =      0;            //pipe_contex->awb_otp_data.awb_data[0].r;
    pdata.awb_data[2].gr =   0;                   //pipe_contex->awb_otp_data.awb_data[0].gr;
    pdata.awb_data[2].gb =   0;                  //pipe_contex->awb_otp_data.awb_data[0].gb;
    pdata.awb_data[2].b =    0;                  //pipe_contex->awb_otp_data.awb_data[0].b;
    pdata.awb_data[2].rg_ratio = pipe_contex->awb_otp_data->awb_info_l_r_.rg_ratio_5800K;
    pdata.awb_data[2].bg_ratio = pipe_contex->awb_otp_data->awb_info_l_r_.bg_ratio_5800K;

    pdata.awb_golden_data[2].color_temperature = COLOR_TEMPERATURE_5800K;
    pdata.awb_golden_data[2].r = 0;
    pdata.awb_golden_data[2].gr = 0;
    pdata.awb_golden_data[2].gb = 0;
    pdata.awb_golden_data[2].b = 0;
    pdata.awb_golden_data[2].rg_ratio = pipe_contex->awb_otp_data->awb_info_golden_.rg_ratio_5800K;
    pdata.awb_golden_data[2].bg_ratio = pipe_contex->awb_otp_data->awb_info_golden_.bg_ratio_5800K;

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

	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].rg_ratio: %ld", pipe_contex->awb_otp_data->awb_data[0].rg_ratio);
	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].bg_ratio: %ld",  pipe_contex->awb_otp_data->awb_data[0].bg_ratio);
	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[1].rg_ratio: %ld",  pipe_contex->awb_otp_data->awb_data[1].rg_ratio);
	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[1].bg_ratio: %ld",  pipe_contex->awb_otp_data->awb_data[1].bg_ratio);
	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[2].rg_ratio: %ld",  pipe_contex->awb_otp_data->awb_data[2].rg_ratio);
	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[2].bg_ratio: %ld",  pipe_contex->awb_otp_data->awb_data[2].bg_ratio);

	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].r: %ld",  pipe_contex->awb_otp_data->awb_data[0].r);
	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].gr: %ld",  pipe_contex->awb_otp_data->awb_data[0].gr);
	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].gb: %ld",  pipe_contex->awb_otp_data->awb_data[0].gb);
	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].b: %ld",  pipe_contex->awb_otp_data->awb_data[0].b);

	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].r: %ld",  pdata.awb_data[0].r);
	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].gr: %ld",  pdata.awb_data[0].gr);
	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].gb: %ld",  pdata.awb_data[0].gb);
	// RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "pdata.awb_data[0].b: %ld",  pdata.awb_data[0].b);
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

void HobotMipiCapIml::sync_awb_ae_task() {
  int ret;
  int vin_fd = -1;
  int isp_fd = -1;
  fd_set read_fds;
  bool sync_awb; bool sync_ccm; bool sync_ae; bool print_isp_log;
  hbn_vnode_handle_t master_isp_handle, slave_isp_handle, slave_vin_handle;
  hbn_isp_exposure_attr_t master_exp_attr, slave_exp_attr;
  hbn_isp_awb_attr_t master_awb_attr, slave_awb_attr;
  hbn_isp_ccm_attr_t master_ccm_attr, slave_ccm_attr;
  hbn_isp_2dnr_attr_t master_2dnr_attr, slave_2dnr_attr;

  if (pipe_contex.size() < 2) return;

  sync_awb = pipe_contex[0].cap_info_->sync_awb_;
  sync_ccm = pipe_contex[0].cap_info_->sync_ccm_;
  sync_ae = pipe_contex[0].cap_info_->sync_ae_;
  print_isp_log = pipe_contex[0].cap_info_->print_isp_log_;

  if (!sync_awb && !sync_ccm && !sync_ae && !print_isp_log) return;

  master_isp_handle = pipe_contex[0].isp_node_handle;
  slave_isp_handle  = pipe_contex[1].isp_node_handle;
  slave_vin_handle  = pipe_contex[1].vin_node_handle;

  ret = hbn_vnode_get_fd(slave_vin_handle, 0, &vin_fd);
  if (ret != 0) {
    std::cout << "vin hbn_vnode_get_fd chn: 0, failed: " << ret << std::endl;
  }

  ret = hbn_vnode_get_fd(slave_isp_handle, 0, &isp_fd);
  if (ret != 0) {
    std::cout << "isp hbn_vnode_get_fd chn: 0, failed: " << ret << std::endl;
  }
  // 先等isp出流第一帧再开始AE同步，以免出现异常打印
  //  start to sync awb or ae after we get the first frame from ISP module, in odrder to avoid inormal cases.
  {
    FD_ZERO(&read_fds);
    FD_SET(vin_fd, &read_fds);
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    select(vin_fd + 1, &read_fds, NULL, NULL, &timeout);
  }

  while(rclcpp::ok()) {
    FD_ZERO(&read_fds);
    FD_SET(vin_fd, &read_fds);
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    int activity = select(vin_fd + 1, &read_fds, NULL, NULL, &timeout);
    if (activity < 0) {
      std::cout << "select fail" << std::endl;
      break;
    } else if (activity == 0) {
      std::cout << "select fail over 1s." << std::endl;
      continue;
    }

    auto rt0 = std::chrono::high_resolution_clock::now();
    ret = hbn_isp_get_exposure_attr(master_isp_handle, &master_exp_attr);
    if (ret != 0) {
      std::cout << "hbn_isp_get_exposure_attr failed: " << ret <<  ",  for master_isp_handle" << std::endl;
      std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
      continue;
    }
    auto rt1 = std::chrono::high_resolution_clock::now();

    ret = hbn_isp_get_awb_attr(master_isp_handle, &master_awb_attr);
    if (ret != 0) {
      std::cout << "hbn_isp_get_awb_attr failed: " << ret <<  ",  for master_isp_handle" << std::endl;
      std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
      continue;
    }
    auto rt2 = std::chrono::high_resolution_clock::now();

    auto dt1 = std::chrono::duration_cast<std::chrono::microseconds>(rt1 - rt0).count() * 1e-3;
    auto dt2 = std::chrono::duration_cast<std::chrono::microseconds>(rt2 - rt1).count() * 1e-3;
    if (print_isp_log) {
      printf("======== master awb==========\n");
      printf("AWB[version: %u, mode: %d, manual_attr_gain: rgain: %f, grgain: %f, gbgain: %f, bgain: %f, consume: %.1fms], "
             "EXP[version: %u, mode: %d, manual_attr_gain: exp_time: %f, again: %f, dgain: %f, ispgain: %f, ae_exp: %f, cur_lux: %u, consume: %.1fms]\n",
             master_awb_attr.version, master_awb_attr.mode, master_awb_attr.manual_attr.gain.rgain, master_awb_attr.manual_attr.gain.grgain, master_awb_attr.manual_attr.gain.gbgain, master_awb_attr.manual_attr.gain.bgain, dt1,
             master_exp_attr.version, master_exp_attr.mode, master_exp_attr.manual_attr.exp_time, master_exp_attr.manual_attr.again, master_exp_attr.manual_attr.dgain, master_exp_attr.manual_attr.ispgain, master_exp_attr.manual_attr.ae_exp, master_exp_attr.manual_attr.cur_lux, dt2);
      printf("======== master awb==========\n");
    }
    ret = hbn_isp_get_exposure_attr(slave_isp_handle, &slave_exp_attr);
    auto rt3 = std::chrono::high_resolution_clock::now();
    if (ret != 0) {
      std::cout << "hbn_isp_get_exposure_attr failed: " << ret <<  ",  for master_isp_handle" << std::endl;
      std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
      continue;
    }
    ret = hbn_isp_get_awb_attr(slave_isp_handle, &slave_awb_attr);
    auto rt4 = std::chrono::high_resolution_clock::now();
    if (ret != 0) {
      std::cout << "hbn_isp_get_awb_attr failed: " << ret <<  ",  for master_isp_handle" << std::endl;
      std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
      continue;
    }
    auto dt3 = std::chrono::duration_cast<std::chrono::microseconds>(rt3 - rt2).count() * 1e-3;
    auto dt4 = std::chrono::duration_cast<std::chrono::microseconds>(rt4 - rt3).count() * 1e-3;
    if (print_isp_log) {
      printf("======== slave awb before sync==========\n");
      printf(
          "AWB[version: %u, mode: %d, manual_attr_gain: rgain: %f, grgain: %f, gbgain: %f, bgain: %f, consume: %.1fms], "
          "EXP[version: %u, mode: %d, manual_attr_gain: exp_time: %f, again: %f, dgain: %f, ispgain: %f, ae_exp: %f, cur_lux: %u, consume: %.1fms]\n",
          slave_awb_attr.version,
          slave_awb_attr.mode,
          slave_awb_attr.manual_attr.gain.rgain,
          slave_awb_attr.manual_attr.gain.grgain,
          slave_awb_attr.manual_attr.gain.gbgain,
          slave_awb_attr.manual_attr.gain.bgain,
          dt3,
          slave_exp_attr.version,
          slave_exp_attr.mode,
          slave_exp_attr.manual_attr.exp_time,
          slave_exp_attr.manual_attr.again,
          slave_exp_attr.manual_attr.dgain,
          slave_exp_attr.manual_attr.ispgain,
          slave_exp_attr.manual_attr.ae_exp,
          slave_exp_attr.manual_attr.cur_lux,
          dt4);
      printf("======== slave awb before sync==========\n");
    }
    if (!sync_awb && !sync_ccm && !sync_ae) {
      continue;
    }

    if (sync_ae) {
      master_exp_attr.mode = HBN_ISP_MODE_MANUAL;
      ret = hbn_isp_set_exposure_attr(slave_isp_handle, &master_exp_attr);
      if (ret != 0) {
        std::cout << "hbn_isp_set_exposure_attr failed: " << ret <<  ",  for slave_isp_handel" << std::endl;
        std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
        continue;
      }
    }
    auto rt5 = std::chrono::high_resolution_clock::now();
    if (sync_awb) {
      master_awb_attr.mode = HBN_ISP_MODE_MANUAL;
      ret = hbn_isp_set_awb_attr(slave_isp_handle, &master_awb_attr);
      if (ret != 0) {
        std::cout << "hbn_isp_set_awb_attr failed: " << ret <<  ",  for slave_isp_handel" << std::endl;
        std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
        continue;
      }
    }
    auto rt6 = std::chrono::high_resolution_clock::now();

    ret = hbn_isp_get_exposure_attr(slave_isp_handle, &slave_exp_attr);
    if (ret != 0) {
      std::cout << "hbn_isp_get_exposure_attr failed: " << ret <<  ",  for master_isp_handle" << std::endl;
      std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
      continue;
    }
    auto rt7 = std::chrono::high_resolution_clock::now();

    ret = hbn_isp_get_awb_attr(slave_isp_handle, &slave_awb_attr);
    if (ret != 0) {
      std::cout << "hbn_isp_get_awb_attr failed: " << ret <<  ",  for master_isp_handle" << std::endl;
      std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
      continue;
    }
    auto dt5 = std::chrono::duration_cast<std::chrono::microseconds>(rt5 - rt4).count() * 1e-3;
    auto dt6 = std::chrono::duration_cast<std::chrono::microseconds>(rt6 - rt5).count() * 1e-3;
    if (print_isp_log) {
      printf("======== slave awb after sync==========\n");
      printf(
          "AWB[version: %u, mode: %d, manual_attr_gain: rgain: %f, grgain: %f, gbgain: %f, bgain: %f, set consume: %.1fms], "
          "EXP[version: %u, mode: %d, manual_attr_gain: exp_time: %f, again: %f, dgain: %f, ispgain: %f, ae_exp: %f, cur_lux: %u, set consume: %.1fms]\n",
          slave_awb_attr.version,
          slave_awb_attr.mode,
          slave_awb_attr.manual_attr.gain.rgain,
          slave_awb_attr.manual_attr.gain.grgain,
          slave_awb_attr.manual_attr.gain.gbgain,
          slave_awb_attr.manual_attr.gain.bgain,
          dt5,
          slave_exp_attr.version,
          slave_exp_attr.mode,
          slave_exp_attr.manual_attr.exp_time,
          slave_exp_attr.manual_attr.again,
          slave_exp_attr.manual_attr.dgain,
          slave_exp_attr.manual_attr.ispgain,
          slave_exp_attr.manual_attr.ae_exp,
          slave_exp_attr.manual_attr.cur_lux,
          dt6);
      printf("======== slave awb after sync==========\n");
    }
    auto rt8 = std::chrono::high_resolution_clock::now();
    ret = hbn_isp_get_ccm_attr(master_isp_handle, &master_ccm_attr);
    if (ret != 0) {
      std::cout << "hbn_isp_get_ccm_attr failed: " << ret <<  ",  for master_isp_handle" << std::endl;
      std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
      continue;
    }
    auto rt9 = std::chrono::high_resolution_clock::now();
    ret = hbn_isp_get_ccm_attr(slave_isp_handle, &slave_ccm_attr);
    if (ret != 0) {
      std::cout << "hbn_isp_get_ccm_attr failed: " << ret <<  ",  for slave_isp_handle" << std::endl;
      std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
      continue;
    }
    auto rt10 = std::chrono::high_resolution_clock::now();

    auto dt7 = std::chrono::duration_cast<std::chrono::microseconds>(rt9 - rt8).count() * 1e-3;
    auto dt8 = std::chrono::duration_cast<std::chrono::microseconds>(rt10 - rt9).count() * 1e-3;
    if (print_isp_log) {
      printf("======== ccm after sync==========\n");
      printf("master ccm configmode: %d\n"
             "consume: %.1fms\n"
             "cc_matrix:\n"
             "[%f, %f, %f]\n"
             "[%f, %f, %f]\n"
             "[%f, %f, %f]\n"
             "cc_offset:\n"
             "[%f, %f, %f]\n",
             master_ccm_attr.configMode,
             dt7,
             master_ccm_attr.manual_attr.cc_matrix[0],
             master_ccm_attr.manual_attr.cc_matrix[1],
             master_ccm_attr.manual_attr.cc_matrix[2],
             master_ccm_attr.manual_attr.cc_matrix[3],
             master_ccm_attr.manual_attr.cc_matrix[4],
             master_ccm_attr.manual_attr.cc_matrix[5],
             master_ccm_attr.manual_attr.cc_matrix[6],
             master_ccm_attr.manual_attr.cc_matrix[7],
             master_ccm_attr.manual_attr.cc_matrix[8],
             master_ccm_attr.manual_attr.cc_offset[0],
             master_ccm_attr.manual_attr.cc_offset[1],
             master_ccm_attr.manual_attr.cc_offset[2]);
      printf("slave ccm configmode: %d\n"
             "consume: %.1fms\n"
             "cc_matrix:\n"
             "[%f, %f, %f]\n"
             "[%f, %f, %f]\n"
             "[%f, %f, %f]\n"
             "cc_offset:\n"
             "[%f, %f, %f]\n",
             slave_ccm_attr.configMode,
             dt8,
             slave_ccm_attr.manual_attr.cc_matrix[0],
             slave_ccm_attr.manual_attr.cc_matrix[1],
             slave_ccm_attr.manual_attr.cc_matrix[2],
             slave_ccm_attr.manual_attr.cc_matrix[3],
             slave_ccm_attr.manual_attr.cc_matrix[4],
             slave_ccm_attr.manual_attr.cc_matrix[5],
             slave_ccm_attr.manual_attr.cc_matrix[6],
             slave_ccm_attr.manual_attr.cc_matrix[7],
             slave_ccm_attr.manual_attr.cc_matrix[8],
             slave_ccm_attr.manual_attr.cc_offset[0],
             slave_ccm_attr.manual_attr.cc_offset[1],
             slave_ccm_attr.manual_attr.cc_offset[2]);
      printf("======== ccm after sync==========\n");
    }
    if (sync_ccm) {
		master_ccm_attr.configMode = HBN_ISP_MODE_MANUAL;
		ret = hbn_isp_set_ccm_attr(slave_isp_handle, &master_ccm_attr);
		if (ret != 0) {
		  std::cout << "hbn_isp_set_ccm_attr failed: " << ret <<  ",  for slave_isp_handel" << std::endl;
		  std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
		  continue;
		}
	}

    ret = hbn_isp_get_2dnr_attr(master_isp_handle, &master_2dnr_attr);
    auto rt11 = std::chrono::high_resolution_clock::now();
    if (ret != 0) {
      std::cout << "hbn_isp_get_2dnr_attr failed: " << ret <<  ", for master_isp_handle" << std::endl;
      std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
      continue;
    }
    ret = hbn_isp_get_2dnr_attr(slave_isp_handle, &slave_2dnr_attr);
    auto rt12 = std::chrono::high_resolution_clock::now();
    if (ret != 0) {
      std::cout << "hbn_isp_get_2dnr_attr failed: " << ret <<  ",  for slave_isp_handle" << std::endl;
      std::cout << "error info: " << hbn_err_info(ret) <<  ", error type: " << hbn_err_type(ret) << std::endl;
      continue;
    }
    auto dt9 = std::chrono::duration_cast<std::chrono::microseconds>(rt11 - rt10).count() * 1e-3;
    auto dt10 = std::chrono::duration_cast<std::chrono::microseconds>(rt12 - rt11).count() * 1e-3;
    if (print_isp_log) {
      printf("======== 2dnr after sync==========\n");
      printf("master blend static: %f, motion: %f, vst_factor: %f, consume: %.1fms\n",
             master_2dnr_attr.manual_attr.blend_static,
             master_2dnr_attr.manual_attr.blend_motion,
             master_2dnr_attr.manual_attr.vst_factor,
             dt9);
      printf("slave blend static: %f, motion: %f, vst_factor: %f, consume: %.1fms\n",
             slave_2dnr_attr.manual_attr.blend_static,
             slave_2dnr_attr.manual_attr.blend_motion,
             slave_2dnr_attr.manual_attr.vst_factor,
             dt10);
      printf("======== 2dnr after sync==========\n");
      std::cout << std::flush;
    }
  }
}
}  // namespace mipi_cam
