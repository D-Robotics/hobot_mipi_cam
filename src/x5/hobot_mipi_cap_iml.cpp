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
#if 0
  if (mipi_started_.size() > 0) { //暂时不能同时启动多个mipi_cam进程
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "The device has already been started\n");
    return -1;
  }
#endif
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
		auto gdc_bin = gen_gdc_bin_rotation(sensor_conf->isp_ichn_attr->width, sensor_conf->isp_ichn_attr->height, cap_info_.width, cap_info_.height, cap_info_.rotation_);
		if (gdc_bin) {
			gdc_bin_buf_.push_back(gdc_bin);
			pipe_contex[0].gdc_bin_r = gdc_bin;
			pipe_contex[1].gdc_bin_r = gdc_bin;
		}
	}
	if (cam_info_.size() != 2) {
		if (!getDualCamCalibrationFromEeprom()) {
			if(strcasecmp(sensor_cof->sensor_name, "sc230ai-30fps") == 0) {
				getDualCamCalibrationFromEeprom_230ai();
			}
		}
	}
	if (cap_info_.gdc_enable_) {
		std::vector<std::shared_ptr<GdcBinBuf_ST>> gdc_bin;
		if (cap_info_.stream_mode_ == 1) {
			gdc_bin = gen_gdc_bin_stereo(cap_info_.width, cap_info_.height, cap_info_.width,
				cap_info_.height, cam_info_, cal_cam_info_, cap_info_.rotation_, cap_info_.cal_rotation_,
				cap_info_.cal_alpha_, !gdc_bin_buf_.empty());
		} else {
			gdc_bin = gen_gdc_bin_stereo(sensor_cof->isp_ichn_attr->width, sensor_cof->isp_ichn_attr->height, cap_info_.width,
				cap_info_.height, cam_info_, cal_cam_info_, cap_info_.rotation_, cap_info_.cal_rotation_,
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
    //n2d_pipe_contex.cap_info_ = &cap_info_;
	//ret = create_and_run_n2d_vflow(&n2d_pipe_contex);
	//ERR_CON_EQ(ret, 0);

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
				gdc_bin = gen_gdc_bin(sensor_cof->isp_ichn_attr->width, sensor_cof->isp_ichn_attr->height, cap_info_.width, cap_info_.height,
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
    //ret = hbn_camera_start(contex.cam_fd);
    //ERR_CON_EQ(ret, 0);
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
  if ((cap_info_.device_mode_.compare("dual") == 0) && 
   	 ((cap_info_.dual_combine_ == 1) || (cap_info_.dual_combine_ == 2))){
	
		{
			std::unique_lock<std::mutex> lk(queue_mtx_);
			for (int j = 0; j < 7; j++) {
				auto buffer_ptr = std::make_shared<VideoBuffer_ST>();
				buffer_ptr->buff_size = cap_info_.width * cap_info_.height * 1.5;
				buffer_ptr->buff = malloc(buffer_ptr->buff_size);
				q_buff_empty_.push(buffer_ptr);
			}
			for (int j = 0; j < 4; j++) {
				auto buffer_ptr = std::make_shared<VideoBuffer_ST>();
				buffer_ptr->buff_size = cap_info_.width * cap_info_.height * 2 * 1.5;
				buffer_ptr->buff = malloc(buffer_ptr->buff_size);
				q_combine_buff_empty_.push(buffer_ptr);
			}
			q_v_buff_.resize(2);
		}

		dual_frame_task_ = std::make_shared<std::thread>(
					std::bind(&HobotMipiCapIml::dualFrameTask, this));
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

int HobotMipiCapIml::getFrame(std::string channel, int* nVOutW, int* nVOutH,
        void* frame_buf, unsigned int bufsize, unsigned int* len,
        uint64_t &timestamp, bool gray) {
  int ret = -1;
  if (!started_) {
     RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "x5 camera isn't started");
    return -1;
  }

  int loop = (1000 / cap_info_.fps + 30) / 10;
  if (dual_frame_task_) {
	do {
		if (!rclcpp::ok()) break;
		{
			std::shared_ptr<VideoBuffer_ST> buff_ptr = nullptr;      
			std::unique_lock<std::mutex> lk(queue_mtx_);
			if (channel == "combine") {
				if (!q_combine_buff_.empty()) {
					buff_ptr = q_combine_buff_.front();    
					q_combine_buff_.pop();
				}
			} else if (channel == "right") {
        if (!q_v_buff_[1].empty()) {
					buff_ptr = q_v_buff_[1].front();      
					q_v_buff_[1].pop();
				}
			} else {
				if (!q_v_buff_[0].empty()) {
					buff_ptr = q_v_buff_[0].front();       
					q_v_buff_[0].pop();
				}
			}
			if (buff_ptr) {
				if (buff_ptr->data_size > bufsize) {
					q_buff_empty_.push(buff_ptr);
					return -1;
				}
				timestamp = buff_ptr->timestamp;
				*nVOutW = buff_ptr->width;
				*nVOutH = buff_ptr->height;
				*len = buff_ptr->data_size;
				if (gray == true) {
					*len = buff_ptr->width * buff_ptr->height;
					memcpy(frame_buf, buff_ptr->buff, *len);
				} else {
					memcpy(frame_buf, buff_ptr->buff, buff_ptr->data_size);
				}
				if (channel == "combine") {
					q_combine_buff_empty_.push(buff_ptr);
				} else {
					q_buff_empty_.push(buff_ptr);
				}
        
				return 0;
			} 
		}
		usleep(10 * 1000);
	} while ((loop-- > 0) && started_);

  } else {
	int stride = 0;
	int nChnID = 0;
	uint32_t frameId = 0;
	int stream_channel = 0;
	if ((channel == "right") && (pipe_contex.size() == 2)) {
		nChnID = 1;
	} else if ((channel == "left") || (channel == "single")) {
		nChnID = 0;
	} else if (channel == "sub_single") {
		if (sub_stream_ == false) {
			return -1;
		}
		nChnID = 0;
		stream_channel = 1;
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
		"x5 camera isn't support channel : %s", channel);
		return -1;
	}
	
	ret = getVnodeFrame(pipe_contex[nChnID].stream_handle, stream_channel, nVOutW, nVOutH, &stride,
						frame_buf, bufsize, len, &timestamp, &frameId, gray);
	if (ret != 0) {
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"), "hbn_vnode_getframe VSE channel 0 failed nChnID = %d,ret = %d\n", nChnID,ret);
		return -1;
	}
  }
  return ret;
}


int HobotMipiCapIml::getVnodeFrame(hbn_vnode_handle_t handle, int channel, int* width,
		int* height, int* stride, void* frame_buf, unsigned int bufsize, unsigned int* len,
        uint64_t *timestamp, uint32_t* frame_id, bool gray) {
	
	if ((width == nullptr) || (height == nullptr) || (stride == nullptr) || (frame_id == nullptr) || 
	    (frame_buf == nullptr) || (len == nullptr) || (timestamp == nullptr)) {

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
  
  *frame_id = out_img.info.frame_id;
  if ("realtime" == cap_info_.frame_ts_type_) {
	*timestamp = out_img.info.sys_timestamps;
  } else {
	*timestamp = out_img.info.timestamps;
  }                       
                          
  RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
            "capture a frame, handle: %llu, id: %d, timestamps: %f, current uptime: %f, sys_timestamps: %f, HW timestamp: %f, trig timestamp: %f,"
            "current timestamp: %f, laps ms: %fms, exposure_time: %fms.", 
			                        handle, *frame_id, timestamps, current_uptime_ts, sys_timestamps, hw_timestamp, tri_timestamp,
                              current_ts, (current_ts - sys_timestamps) * 1e3, (double)exposure_time * 1e-6);

	//std::cout << "getVnodeFrame--system time sec:" << tv.tv_sec << ", image time sec:" << out_img.info.tv.tv_sec
	//          << ", trig time sec:" << out_img.info.trig_tv.tv_sec 
	//		  << ", image timestamps(/1e9) sec:" << out_img.info.timestamps / 1e9 <<  std::endl;

	//std::cout << "getVnodeFrame--system time sec:" << tv.tv_sec << ", timestamp time sec:" << (int)(*timestamp / 1e9) <<  std::endl;
	
	*stride = out_img.buffer.stride;
	*width = out_img.buffer.width;
	*height = out_img.buffer.height;
	if (gray == true) {
		*len = out_img.buffer.size[0];
	} else {
		*len = out_img.buffer.size[0] + out_img.buffer.size[1];
	}
	if (bufsize < *len) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
			"buf size(%d) < frame size(%d)", bufsize, *len);
		hbn_vnode_releaseframe(handle, channel, &out_img);
		*len = 0;
		return -1;
	}
	memcpy(frame_buf, out_img.buffer.virt_addr[0], out_img.buffer.size[0]);
	if (gray == false) {
		memcpy(frame_buf + out_img.buffer.size[0], out_img.buffer.virt_addr[1], out_img.buffer.size[1]);
	}
	hbn_vnode_releaseframe(handle, channel, &out_img);
	return 0;
}

void HobotMipiCapIml::dualFrameTask() {
  if (!started_) {
     RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "x5 camera isn't started");
    return;
  }
  if (pipe_contex.size() != 2) {
	return;
  }

  std::shared_ptr<VideoBuffer_ST> combine_buff_ptr = nullptr;
  std::vector<std::shared_ptr<VideoBuffer_ST>> buff_ptr;
  buff_ptr.resize(2);

  int ret = 0;
  fd_set readfds;
  struct timeval timeout;
  int result;
  int max_handle;
  int left_fd;
  int right_fd;
  std::vector<int> ochn_fd;
  ochn_fd.resize(2);
  std::vector<int> loss_cnt = {0,0};
	
  int diff_time = 500000000 / cap_info_.fps;

  hbn_vnode_get_fd(pipe_contex[0].stream_handle, 0, &ochn_fd[0]);
  hbn_vnode_get_fd(pipe_contex[1].stream_handle, 0, &ochn_fd[1]);

  while (started_) {

		max_handle = 0;
		FD_ZERO(&readfds);
		FD_SET(ochn_fd[0], &readfds);
		FD_SET(ochn_fd[1], &readfds);

		timeout.tv_sec = 2;
		timeout.tv_usec = 0;
		loss_cnt[0]++;
		loss_cnt[1]++;

		max_handle = max_handle > ochn_fd[0]?max_handle : ochn_fd[0];
		max_handle = max_handle > ochn_fd[1]?max_handle : ochn_fd[1];

    result = select(max_handle + 1, &readfds, nullptr, nullptr, &timeout);
    if (result == -1) {
			std::cerr << "Select error" << std::endl;
			break;
    } else if (result == 0) {
			// 超时
			std::cout << "Timeout occurred" << std::endl;
			std::unique_lock<std::mutex> lk(queue_mtx_);
			for (int i = 0; i < buff_ptr.size(); i++) {
				if (buff_ptr[i]) {
					q_v_buff_[i].push(buff_ptr[i]);
					buff_ptr[i] = nullptr;
				}
			}
			continue;
    } else {
		for (int i = 0; i < ochn_fd.size(); i++) {
			if (!rclcpp::ok()) break;
			if (FD_ISSET(ochn_fd[i], &readfds)) {
				if (buff_ptr[i] == nullptr) {
					{
						std::unique_lock<std::mutex> lk(queue_mtx_);
						if (q_v_buff_[i].size() >= 3) {
							buff_ptr[i] = q_v_buff_[i].front();
							q_v_buff_[i].pop();
						} else if (q_buff_empty_.size() > 0) {
							buff_ptr[i] = q_buff_empty_.front();
							q_buff_empty_.pop();
						}
					}

					if (buff_ptr[i]) {
						loss_cnt[i] = 0;
						ret = getVnodeFrame(pipe_contex[i].stream_handle, 0, &buff_ptr[i]->width, &buff_ptr[i]->height, &buff_ptr[i]->stride,
								buff_ptr[i]->buff, buff_ptr[i]->buff_size, &buff_ptr[i]->data_size, &buff_ptr[i]->timestamp, &buff_ptr[i]->frame_id);
						if (ret != 0) {
							RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_vnode_getframe VSE channel = %d failed ,ret = %d\n", i,ret);
							std::unique_lock<std::mutex> lk(queue_mtx_);
							q_buff_empty_.push(buff_ptr[i]);
							buff_ptr[i] = nullptr;
						}
					}
				}
			}
		}

		if (buff_ptr[0] && buff_ptr[1]  && (buff_ptr[0]->width == buff_ptr[1]->width) &&
	   			(buff_ptr[0]->height == buff_ptr[1]->height)) {

			if (std::abs((int)(buff_ptr[0]->timestamp - buff_ptr[1]->timestamp)) < diff_time) {
				{
					std::unique_lock<std::mutex> lk(queue_mtx_);
					if (q_combine_buff_.size() >= 3) {
						combine_buff_ptr = q_combine_buff_.front();
						q_combine_buff_.pop();
					} else if (q_combine_buff_empty_.size() > 0) {
						combine_buff_ptr = q_combine_buff_empty_.front();
						q_combine_buff_empty_.pop();
					}
				}
				if (combine_buff_ptr) {
					if (combine_buff_ptr->buff_size >= buff_ptr[0]->data_size * 2) {
#if 0
						combine_buff_ptr->timestamp = buff_ptr[0]->timestamp;
						combine_buff_ptr->width = buff_ptr[0]->width * 2;
						combine_buff_ptr->height = buff_ptr[0]->height;
						combine_buff_ptr->stride = buff_ptr[0]->stride * 2;
						combine_buff_ptr->data_size = buff_ptr[0]->data_size * 2;
						for (int i = 0; i < buff_ptr[0]->height; i++) {
							memcpy(combine_buff_ptr->buff + i * combine_buff_ptr->width, 
									buff_ptr[0]->buff + i * buff_ptr[0]->width, buff_ptr[0]->width);
							memcpy(combine_buff_ptr->buff + i * combine_buff_ptr->width + buff_ptr[0]->width, 
									buff_ptr[1]->buff + i * buff_ptr[1]->width, buff_ptr[1]->width);
						}
						char* combine_uv_ptr = (char *)(combine_buff_ptr->buff +  combine_buff_ptr->width * combine_buff_ptr->height);
						char* left_uv_ptr = (char *)(buff_ptr[0]->buff +  buff_ptr[0]->width * buff_ptr[0]->height);
						char* right_uv_ptr = (char *)(buff_ptr[1]->buff +  buff_ptr[1]->width * buff_ptr[1]->height);
						int uv_height = buff_ptr[0]->height / 2;

						for (int i = 0; i < uv_height; i++) {
							memcpy(combine_uv_ptr + i * combine_buff_ptr->width, 
									left_uv_ptr + i * buff_ptr[0]->width, buff_ptr[0]->width);
							memcpy(combine_uv_ptr + i * combine_buff_ptr->width + buff_ptr[0]->width, 
									right_uv_ptr + i * buff_ptr[1]->width, buff_ptr[1]->width);
						}
#else
						combine_buff_ptr->timestamp = buff_ptr[0]->timestamp;
						combine_buff_ptr->width = buff_ptr[0]->width;
						combine_buff_ptr->height = buff_ptr[0]->height * 2;
						combine_buff_ptr->stride = buff_ptr[0]->stride;
						combine_buff_ptr->data_size = buff_ptr[0]->data_size * 2;
						int y_size = buff_ptr[0]->width * buff_ptr[0]->height;
						int uv_size = y_size / 2;
						//copy y
						memcpy(combine_buff_ptr->buff, buff_ptr[0]->buff, y_size);
						memcpy(combine_buff_ptr->buff + y_size, buff_ptr[1]->buff, y_size);
                        
						//copy uv
						memcpy(combine_buff_ptr->buff + y_size * 2, buff_ptr[0]->buff + y_size, uv_size);
						memcpy(combine_buff_ptr->buff + y_size * 2 + uv_size, buff_ptr[1]->buff + y_size, uv_size);

#endif
						std::unique_lock<std::mutex> lk(queue_mtx_);                                          
						q_combine_buff_.push(combine_buff_ptr);

					} else {
						std::unique_lock<std::mutex> lk(queue_mtx_);
						q_combine_buff_empty_.push(combine_buff_ptr);
					}
					combine_buff_ptr = nullptr;
				}
				std::unique_lock<std::mutex> lk(queue_mtx_);
				q_v_buff_[0].push(buff_ptr[0]);
				buff_ptr[0] = nullptr;
				q_v_buff_[1].push(buff_ptr[1]);
				buff_ptr[1] = nullptr;
			} else {
				std::unique_lock<std::mutex> lk(queue_mtx_);
				if (buff_ptr[0]->timestamp < buff_ptr[1]->timestamp) {
					q_v_buff_[0].push(buff_ptr[0]);
					buff_ptr[0] = nullptr;
				} else {
					q_v_buff_[1].push(buff_ptr[1]);
					buff_ptr[1] = nullptr;
				}
			} 
		}
		std::unique_lock<std::mutex> lk(queue_mtx_);
		for (int i = 0; i < buff_ptr.size(); i++) {
			if ((buff_ptr[i]) && (loss_cnt[i] > 0)) {
				q_v_buff_[i].push(buff_ptr[i]);
				buff_ptr[i] = nullptr;
				loss_cnt[i] = 0;
			}
		}
	}
  }
  return;
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
	//vse_ochn_attr.target_w = input_width;
	//vse_ochn_attr.target_h = input_height;
	vse_ochn_attr.target_w = pipe_contex->cap_info_->width;
	vse_ochn_attr.target_h = pipe_contex->cap_info_->height;

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
		sub_stream_ = true;
		pipe_contex->cap_info_->sub_stream_flag_ = true;
	} else if ((pipe_contex->cap_info_->sub_width >= input_width) 
		  && (pipe_contex->cap_info_->sub_height >= input_height)) {
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
		sub_stream_ = true;
		pipe_contex->cap_info_->sub_stream_flag_ = true;			

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

	if (pipe_contex->gdc_init_valid_r == 1) {
		vse_ochn_attr_t vse_ochn_attr;
		ret = hbn_vnode_get_ochn_attr(pipe_contex->vse_node_handle, chn_id, &vse_ochn_attr);
		ERR_CON_EQ(ret, 0);
		input_width = vse_ochn_attr.target_w;
		input_height = vse_ochn_attr.target_h;
	} else {
		isp_ichn_attr_t isp_ichn_attr;
		ret = hbn_vnode_get_ichn_attr(pipe_contex->isp_node_handle, chn_id, &isp_ichn_attr);
		ERR_CON_EQ(ret, 0);
		input_width = isp_ichn_attr.width;
		input_height = isp_ichn_attr.height;
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
	gdc_ochn_attr.output_width = pipe_contex->cap_info_->width;
	gdc_ochn_attr.output_height = pipe_contex->cap_info_->height;
	gdc_ochn_attr.output_stride = pipe_contex->cap_info_->width;
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
    if (pipe_contex->cap_info_->device_mode_.compare("dual") == 0) {
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
			pipe_contex->sensor_config.camera_config->sensor_mode = 1;
			pipe_contex->sensor_config.vin_node_attr->lpwm_attr.enable = 0;
			pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
		    pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
		}
	} else {
		pipe_contex->sensor_config.isp_attr->input_mode = 2;
		pipe_contex->sensor_config.vin_node_attr->lpwm_attr.enable = 0;
		pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
		pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
	}
	// 创建pipeline中的每个node
	ret = creat_camera_node(pipe_contex->sensor_config.camera_config, &pipe_contex->cam_fd);
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
	gdc_width_scale = in_gdc_width / static_cast<float>(out_width);
	gdc_height_scale = in_gdc_height / static_cast<float>(out_height);

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

	double alpha = 0.0;
	if (cal_alpha <= 1.0) {
		alpha = cal_alpha;
	}

	// TODO: Set alpha from config
	// cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height), R_rl, t_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 0.391 ,cv::Size(out_gdc_width, out_gdc_height));
	if (cam_info[0].distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
		double balance = alpha;
		cv::fisheye::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height), R_rl, t_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, cv::Size(out_gdc_width, out_gdc_height), balance);
	} else {
		cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height), R_rl, t_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, alpha ,cv::Size(out_gdc_width, out_gdc_height));
	}
	cv::initUndistortRectifyMap(Kl, Dl, Rl, Pl, cv::Size(out_gdc_width, out_gdc_height), CV_32FC1, undistmap1l, undistmap2l);
	cv::initUndistortRectifyMap(Kr, Dr, Rr, Pr, cv::Size(out_gdc_width, out_gdc_height), CV_32FC1, undistmap1r, undistmap2r);
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

bool HobotMipiCapIml::readEeprom16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, char* buf, int bufsize) {
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


int HobotMipiCapIml::detectEeprom_lianhe(std::string &device, int &i2c_bus, uint16_t &i2c_addr) {

  // mipi sensor的信息数组
  EEPROM_ID_T eeprom_id_list[] = {
    {1, 0x50, I2C_ADDR_16, 0x21, 0x01, "P24C64G-C4H-MIR"},  // P24C64G-C4H-MIR
  };
  std::vector<int> i2c_buss= {0,1,2,3,4,5,6,7,8,9,10};

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

int HobotMipiCapIml::detectEeprom_drobot(std::string &device, int &i2c_bus, uint16_t &i2c_addr) {

  // mipi sensor的信息数组
  EEPROM_DETECT_T eeprom_detect_list[] = {
    {1, 0x50, I2C_ADDR_16, 0x00, "SZYGSJKJ", "yuguang"},  // P24C64G-C4H-MIR
	{1, 0x50, I2C_ADDR_16, 0x00, "UNION", "union"},  // P24C64G-C4H-MIR
	{1, 0x50, I2C_ADDR_16, 0x00, "SZ_ABHAM", "sz_abham"},  // P24C64G-C4H-MIR
  };
  std::vector<int> i2c_buss= {0,1,2,3,4,5,6,7,8,9,10};

  char buf[9] = {0};
  std::vector<char> buf_type;
  buf_type.resize(0x1f);
  char check_0;
  char checksum;
  std::string chip_type;
  
  for (auto num : i2c_buss) {
    for (auto eeprom_id : eeprom_detect_list) {
      if (readEeprom16(num, eeprom_id.i2c_dev_addr, eeprom_id.det_reg, buf, 8)) {
		std::string buf_str = buf;
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),"i2c bus: %d, EEPROM FLAG: %s\n", num, buf_str.c_str());
		if (eeprom_id.check_str == buf_str) {
			i2c_bus = num;
			i2c_addr = eeprom_id.i2c_dev_addr;
			device = eeprom_id.device_name;
			return 0;
		}
      }
    }
  }
  return -1;
}


bool HobotMipiCapIml::getDualCamCalibrationFromEeprom() {
  int i2c_bus;
  uint16_t i2c_addr;
  std::string device;
  std::vector<char> i2c_buf;
  i2c_buf.resize(sizeof(CalDualCamInfo_ST));
  char chech_value;
  if (detectEeprom_drobot(device, i2c_bus, i2c_addr) == -1) {
	return false;
  }
  eeprom_name_ = device;
  if (device == "yuguang") {
	getDualCamCalibration_yugang(i2c_bus, i2c_addr);
  } else if (device == "union") {
	getDualCamCalibration_union(i2c_bus, i2c_addr);
  } else if (device == "sz_abham") {
	getDualCamCalibration_abham(i2c_bus, i2c_addr);
  } 
  return true;
}

bool HobotMipiCapIml::getDualCamCalibration_yugang(int i2c_bus, uint16_t i2c_addr) {
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
		cap_info_.cal_rotation_ = 0.0;
	} else if (head_buf_ptr->angle == 0x01) {
		cap_info_.cal_rotation_ = 90.0;
	} else if (head_buf_ptr->angle == 0x02) {
		cap_info_.cal_rotation_ = 180.0;
	} else if (head_buf_ptr->angle == 0x03) {
		cap_info_.cal_rotation_ = 270.0;
	}



	if (head_buf_ptr->camType == 0x01) {
		cam_info_.resize(2);
		if (head_buf_ptr->cal_tpye == 0x01) {
			cam_info_[0].distortion_model = cam_info_[1].distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
		} else {
			cam_info_[0].distortion_model = cam_info_[1].distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
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

		cam_info_[0].width = m_d_info_l.width;
		cam_info_[0].height = m_d_info_l.height;
		cam_info_[1].width = m_d_info_r.width;
		cam_info_[1].height = m_d_info_r.height;

		cv::Mat l_k= cv::Mat::zeros(3,3,CV_64F);
		l_k.at<double>(0,0) = m_d_info_l.fx;
		l_k.at<double>(0,2) = m_d_info_l.cx;
		l_k.at<double>(1,1) = m_d_info_l.fy;
		l_k.at<double>(1,2) = m_d_info_l.cy;
		l_k.at<double>(2,2) = 1;
		std::copy(l_k.ptr<double>(0), l_k.ptr<double>(0) + l_k.total(), cam_info_[0].k.begin());
		
		int d_num = 8;
		if (head_buf_ptr->d_num <= 0 && head_buf_ptr->d_num >=4) {
			d_num = head_buf_ptr->d_num;
		}
		cam_info_[0].d.resize(d_num);
		for (int i = 0; i < d_num; i++) {
			cam_info_[0].d[i] = m_d_info_l.d[i];
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
		std::copy(l_r_eye.ptr<double>(0), l_r_eye.ptr<double>(0) + l_r_eye.total(), cam_info_[0].r.begin());

		cv::Mat l_p_eye = cv::Mat::eye(3, 4, CV_64F);
		cv::Mat l_p = l_k * l_p_eye;
		std::copy(l_p.ptr<double>(0), l_p.ptr<double>(0) + l_p.total(), cam_info_[0].p.begin());



		cv::Mat r_k= cv::Mat::zeros(3,3,CV_64F);
		r_k.at<double>(0,0) = m_d_info_r.fx;
		r_k.at<double>(0,2) = m_d_info_r.cx;
		r_k.at<double>(1,1) = m_d_info_r.fy;
		r_k.at<double>(1,2) = m_d_info_r.cy;
		r_k.at<double>(2,2) = 1;
		std::copy(r_k.ptr<double>(0), r_k.ptr<double>(0) + r_k.total(), cam_info_[1].k.begin());

		// cam_info_[1].d.resize(8);
		// cam_info_[1].d[0] = m_d_info_r.k1;
		// cam_info_[1].d[1] = m_d_info_r.k2;
		// cam_info_[1].d[2] = m_d_info_r.p1;
		// cam_info_[1].d[3] = m_d_info_r.p2;
		// cam_info_[1].d[4] = m_d_info_r.k3;
		// cam_info_[1].d[5] = m_d_info_r.k4;
		// cam_info_[1].d[6] = m_d_info_r.k5;
		// cam_info_[1].d[7] = m_d_info_r.k6;

		cam_info_[1].d.resize(d_num);
		for (int i = 0; i < d_num; i++) {
			cam_info_[1].d[i] = m_d_info_r.d[i];
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
		std::copy(R.ptr<double>(0), R.ptr<double>(0) + R.total(), cam_info_[1].r.begin());
		std::copy(P.ptr<double>(0), P.ptr<double>(0) + P.total(), cam_info_[1].p.begin());
		return true;
	}

  }
  return false;
}

bool HobotMipiCapIml::getDualCamCalibration_union(int i2c_bus, uint16_t i2c_addr) {
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
		  cap_info_.cal_rotation_ = 0.0;
	  } else if (head_buf_ptr->angle == 0x01) {
		  cap_info_.cal_rotation_ = 90.0;
	  } else if (head_buf_ptr->angle == 0x02) {
		  cap_info_.cal_rotation_ = 180.0;
	  } else if (head_buf_ptr->angle == 0x03) {
		  cap_info_.cal_rotation_ = 270.0;
	  }
  
	  if ((head_buf_ptr->camType == 0x01) || (head_buf_ptr->camType == 0x11)) {
		  cam_info_.resize(2);
		  if (head_buf_ptr->cal_tpye == 0x01) {
			cam_info_[0].distortion_model = cam_info_[1].distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
		  } else {
			cam_info_[0].distortion_model = cam_info_[1].distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
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
  
		  cam_info_[0].width = w_h_info.width;
		  cam_info_[0].height = w_h_info.height;
		  cam_info_[1].width = w_h_info.width;
		  cam_info_[1].height = w_h_info.height;
  
		  cv::Mat l_k= cv::Mat::zeros(3,3,CV_64F);
		  l_k.at<double>(0,0) = m_d_info_l.fx;
		  l_k.at<double>(0,2) = m_d_info_l.cx;
		  l_k.at<double>(1,1) = m_d_info_l.fy;
		  l_k.at<double>(1,2) = m_d_info_l.cy;
		  l_k.at<double>(2,2) = 1;
		  std::copy(l_k.ptr<double>(0), l_k.ptr<double>(0) + l_k.total(), cam_info_[0].k.begin());
		  
		  int d_num = 8;
		  if (head_buf_ptr->d_num <= 0 && head_buf_ptr->d_num >=4) {
			  d_num = head_buf_ptr->d_num;
		  }
		  cam_info_[0].d.resize(d_num);
		  for (int i = 0; i < d_num; i++) {
			  cam_info_[0].d[i] = m_d_info_l.d[i];
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
		  std::copy(l_r_eye.ptr<double>(0), l_r_eye.ptr<double>(0) + l_r_eye.total(), cam_info_[0].r.begin());
  
		  cv::Mat l_p_eye = cv::Mat::eye(3, 4, CV_64F);
		  cv::Mat l_p = l_k * l_p_eye;
		  std::copy(l_p.ptr<double>(0), l_p.ptr<double>(0) + l_p.total(), cam_info_[0].p.begin());
  
  
  
		  cv::Mat r_k= cv::Mat::zeros(3,3,CV_64F);
		  r_k.at<double>(0,0) = m_d_info_r.fx;
		  r_k.at<double>(0,2) = m_d_info_r.cx;
		  r_k.at<double>(1,1) = m_d_info_r.fy;
		  r_k.at<double>(1,2) = m_d_info_r.cy;
		  r_k.at<double>(2,2) = 1;
		  std::copy(r_k.ptr<double>(0), r_k.ptr<double>(0) + r_k.total(), cam_info_[1].k.begin());
  
		  // cam_info_[1].d.resize(8);
		  // cam_info_[1].d[0] = m_d_info_r.k1;
		  // cam_info_[1].d[1] = m_d_info_r.k2;
		  // cam_info_[1].d[2] = m_d_info_r.p1;
		  // cam_info_[1].d[3] = m_d_info_r.p2;
		  // cam_info_[1].d[4] = m_d_info_r.k3;
		  // cam_info_[1].d[5] = m_d_info_r.k4;
		  // cam_info_[1].d[6] = m_d_info_r.k5;
		  // cam_info_[1].d[7] = m_d_info_r.k6;
  
		  cam_info_[1].d.resize(d_num);
		  for (int i = 0; i < d_num; i++) {
			  cam_info_[1].d[i] = m_d_info_r.d[i];
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
		  std::copy(R.ptr<double>(0), R.ptr<double>(0) + R.total(), cam_info_[1].r.begin());
		  std::copy(P.ptr<double>(0), P.ptr<double>(0) + P.total(), cam_info_[1].p.begin());
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

		  }
		  return true;
	  }
  
	}
	return false;
}
    

bool HobotMipiCapIml::getDualCamCalibration_abham(int i2c_bus, uint16_t i2c_addr) {
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
		  cap_info_.cal_rotation_ = 0.0;
	  } else if (head_buf_ptr->angle == 0x01) {
		  cap_info_.cal_rotation_ = 90.0;
	  } else if (head_buf_ptr->angle == 0x02) {
		  cap_info_.cal_rotation_ = 180.0;
	  } else if (head_buf_ptr->angle == 0x03) {
		  cap_info_.cal_rotation_ = 270.0;
	  }
  
	  if (head_buf_ptr->camType == 0x01) {
		  cam_info_.resize(2);
		  if (head_buf_ptr->cal_tpye == 0x01) {
			cam_info_[0].distortion_model = cam_info_[1].distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
		  } else {
			cam_info_[0].distortion_model = cam_info_[1].distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
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
  
		  cam_info_[0].width = m_d_info_l.width;
		  cam_info_[0].height = m_d_info_l.height;
		  cam_info_[1].width = m_d_info_r.width;
		  cam_info_[1].height = m_d_info_r.height;
  
		  cv::Mat l_k= cv::Mat::zeros(3,3,CV_64F);
		  l_k.at<double>(0,0) = m_d_info_l.fx;
		  l_k.at<double>(0,2) = m_d_info_l.cx;
		  l_k.at<double>(1,1) = m_d_info_l.fy;
		  l_k.at<double>(1,2) = m_d_info_l.cy;
		  l_k.at<double>(2,2) = 1;
		  std::copy(l_k.ptr<double>(0), l_k.ptr<double>(0) + l_k.total(), cam_info_[0].k.begin());
		  
		  int d_num = 8;
		  if (head_buf_ptr->d_num <= 0 && head_buf_ptr->d_num >=4) {
			  d_num = head_buf_ptr->d_num;
		  }
		  cam_info_[0].d.resize(d_num);
		  for (int i = 0; i < d_num; i++) {
			  cam_info_[0].d[i] = m_d_info_l.d[i];
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
		  std::copy(l_r_eye.ptr<double>(0), l_r_eye.ptr<double>(0) + l_r_eye.total(), cam_info_[0].r.begin());
  
		  cv::Mat l_p_eye = cv::Mat::eye(3, 4, CV_64F);
		  cv::Mat l_p = l_k * l_p_eye;
		  std::copy(l_p.ptr<double>(0), l_p.ptr<double>(0) + l_p.total(), cam_info_[0].p.begin());
  
  
  
		  cv::Mat r_k= cv::Mat::zeros(3,3,CV_64F);
		  r_k.at<double>(0,0) = m_d_info_r.fx;
		  r_k.at<double>(0,2) = m_d_info_r.cx;
		  r_k.at<double>(1,1) = m_d_info_r.fy;
		  r_k.at<double>(1,2) = m_d_info_r.cy;
		  r_k.at<double>(2,2) = 1;
		  std::copy(r_k.ptr<double>(0), r_k.ptr<double>(0) + r_k.total(), cam_info_[1].k.begin());
  
		  // cam_info_[1].d.resize(8);
		  // cam_info_[1].d[0] = m_d_info_r.k1;
		  // cam_info_[1].d[1] = m_d_info_r.k2;
		  // cam_info_[1].d[2] = m_d_info_r.p1;
		  // cam_info_[1].d[3] = m_d_info_r.p2;
		  // cam_info_[1].d[4] = m_d_info_r.k3;
		  // cam_info_[1].d[5] = m_d_info_r.k4;
		  // cam_info_[1].d[6] = m_d_info_r.k5;
		  // cam_info_[1].d[7] = m_d_info_r.k6;
  
		  cam_info_[1].d.resize(d_num);
		  for (int i = 0; i < d_num; i++) {
			  cam_info_[1].d[i] = m_d_info_r.d[i];
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
		  std::copy(R.ptr<double>(0), R.ptr<double>(0) + R.total(), cam_info_[1].r.begin());
		  std::copy(P.ptr<double>(0), P.ptr<double>(0) + P.total(), cam_info_[1].p.begin());
		  return true;
	  }
  
	}
	return false;
  }


bool HobotMipiCapIml::getDualCamCalibrationFromEeprom_230ai() {
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
	cam_info_.resize(2);
	cam_info_[0].distortion_model = cam_info_[1].distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
	CalDualCamInfo_ST* i2c_buf_ptr = (CalDualCamInfo_ST *)i2c_buf.data();
	int width = (i2c_buf_ptr->h_v[0] << 8) | i2c_buf_ptr->h_v[1];
	int height = (i2c_buf_ptr->h_v[2] << 8) | i2c_buf_ptr->h_v[3];

	cam_info_[0].width = width;
    cam_info_[0].height = height;
	cam_info_[1].width = width;
    cam_info_[1].height = height;

	cv::Mat l_k= cv::Mat::zeros(3,3,CV_64F);
	l_k.at<double>(0,0) = i2c_buf_ptr->fxl;
	l_k.at<double>(0,2) = i2c_buf_ptr->cxl;
	l_k.at<double>(1,1) = i2c_buf_ptr->fyl;
	l_k.at<double>(1,2) = i2c_buf_ptr->cyl;
	l_k.at<double>(2,2) = 1;
	std::copy(l_k.ptr<double>(0), l_k.ptr<double>(0) + l_k.total(), cam_info_[0].k.begin());

	cam_info_[0].d.resize(8);
	cam_info_[0].d[0] = i2c_buf_ptr->k1l;
	cam_info_[0].d[1] = i2c_buf_ptr->k2l;
	cam_info_[0].d[2] = i2c_buf_ptr->p1l;
	cam_info_[0].d[3] = i2c_buf_ptr->p2l;
	cam_info_[0].d[4] = i2c_buf_ptr->k3l;
	cam_info_[0].d[5] = i2c_buf_ptr->k4l;
	cam_info_[0].d[6] = i2c_buf_ptr->k5l;
	cam_info_[0].d[7] = i2c_buf_ptr->k6l;

	cv::Mat l_r_eye = cv::Mat::eye(3, 3, CV_64F);
    std::copy(l_r_eye.ptr<double>(0), l_r_eye.ptr<double>(0) + l_r_eye.total(), cam_info_[0].r.begin());

	cv::Mat l_p_eye = cv::Mat::eye(3, 4, CV_64F);
    cv::Mat l_p = l_k * l_p_eye;
    std::copy(l_p.ptr<double>(0), l_p.ptr<double>(0) + l_p.total(), cam_info_[0].p.begin());

	cv::Mat r_k= cv::Mat::zeros(3,3,CV_64F);
	r_k.at<double>(0,0) = i2c_buf_ptr->fxr;
	r_k.at<double>(0,2) = i2c_buf_ptr->cxr;
	r_k.at<double>(1,1) = i2c_buf_ptr->fyr;
	r_k.at<double>(1,2) = i2c_buf_ptr->cyr;
	r_k.at<double>(2,2) = 1;
	std::copy(r_k.ptr<double>(0), r_k.ptr<double>(0) + r_k.total(), cam_info_[1].k.begin());

	cam_info_[1].d.resize(8);
	cam_info_[1].d[0] = i2c_buf_ptr->k1r;
	cam_info_[1].d[1] = i2c_buf_ptr->k2r;
	cam_info_[1].d[2] = i2c_buf_ptr->p1r;
	cam_info_[1].d[3] = i2c_buf_ptr->p2r;
	cam_info_[1].d[4] = i2c_buf_ptr->k3r;
	cam_info_[1].d[5] = i2c_buf_ptr->k4r;
	cam_info_[1].d[6] = i2c_buf_ptr->k5r;
	cam_info_[1].d[7] = i2c_buf_ptr->k6r;

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
    std::copy(R.ptr<double>(0), R.ptr<double>(0) + R.total(), cam_info_[1].r.begin());
    std::copy(P.ptr<double>(0), P.ptr<double>(0) + P.total(), cam_info_[1].p.begin());
	return true;
  }
  return false;
}

}  // namespace mipi_cam
