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

#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cap_iml.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "opencv2/opencv.hpp"

#include "hb_media_codec.h"
#include "hb_media_error.h"

#include <rclcpp/rclcpp.hpp>
#include <json/json.h>

#define ERR_CON_EQ(ret, a) do {\
		if ((ret) != (a)) {\
			printf("%s(%d) failed, ret %d\n", __func__, __LINE__, (int32_t)(ret));\
			return (ret);\
		}\
	} while(0)\


#define ERR_CON_NE(ret, a) do {\
		if ((ret) == (a)) {\
			printf("%s(%d) failed, ret %ld\n", __func__, __LINE__, (ret));\
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

  RCLCPP_WARN(rclcpp::get_logger("mipi_cam"), "this board support mipi:");
  for (auto host : mipi_hosts) {
	RCLCPP_WARN(rclcpp::get_logger("mipi_cam"), "host %d", host);
  }

  listMipiHost(mipi_hosts, mipi_started_, mipi_stoped_);
  if (mipi_started_.size() > 0) { //暂时不能同时启动多个mipi_cam进程
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "The device has already been started\n");
    return -1;
  }
  if (mipi_stoped_.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "There are no available host.\n");
    return -1;
  }
  if (board_config_m_.size() == 0) {
    if (mipi_started_.size() > 0) {
      return -1;
    }
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
  for (auto i : mipi_stoped_) {
	ret = vp_sensor_detect_2(i, &host_info);
	if (ret == 0) {
		v_host_info_detect.push_back(host_info);
	}
  }
  if (cap_info_.device_mode_.compare("dual") == 0) {
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
	//copy_config(&pipe_contex[0].sensor_config, vp_sensor_config_list[v_host_info[0].sensor_index]);
	memcpy(&pipe_contex[0].sensor_config, vp_sensor_config_list[v_host_info[0].sensor_index], sizeof(vp_sensor_config_t));
	ret = vp_sensor_fixed_mipi_host_1(v_host_info[0].host_num, &pipe_contex[0].sensor_config, &pipe_contex[0].csi_config);
	ERR_CON_EQ(ret, 0);
	if (cap_info_.gdc_enable_) {
		vp_sensor_config_t *sensor_cof = &pipe_contex[0].sensor_config;
		if (cam_info_.size() != 2) {
			getDualCamCalibrationFromEeprom();
		}
		gdc_bin_buf_.clear();
		if (cap_info_.rotation_ != 0) {
			vp_sensor_config_t *sensor_conf = &pipe_contex[0].sensor_config;
			auto gdc_bin = gen_gdc_bin_rotation(sensor_conf->isp_cfg->isp_attr.size.width, sensor_conf->isp_cfg->isp_attr.size.height, cap_info_.width, cap_info_.height, cap_info_.rotation_);
			if (gdc_bin) {
				gdc_bin_buf_.push_back(gdc_bin);
				pipe_contex[0].gdc_bin_r = gdc_bin;
				pipe_contex[1].gdc_bin_r = gdc_bin;
			}
		}
		if (pipe_contex[0].gdc_bin_r != nullptr) {
			auto gdc_bin = gen_gdc_bin_stereo(cap_info_.width, cap_info_.height, cap_info_.width, cap_info_.height, cam_info_, cal_cam_info_);
			if (gdc_bin.size() == 2) {
				gdc_bin_buf_.push_back(gdc_bin[0]);
				gdc_bin_buf_.push_back(gdc_bin[1]);
				pipe_contex[0].gdc_bin = gdc_bin[0];
				pipe_contex[1].gdc_bin = gdc_bin[1];
			}		
		} else {
			auto gdc_bin = gen_gdc_bin_stereo(sensor_cof->isp_cfg->isp_attr.size.width, sensor_cof->isp_cfg->isp_attr.size.height, cap_info_.width, cap_info_.height, cam_info_, cal_cam_info_);
			if (gdc_bin.size() == 2) {
				gdc_bin_buf_.push_back(gdc_bin[0]);
				gdc_bin_buf_.push_back(gdc_bin[1]);
				pipe_contex[0].gdc_bin = gdc_bin[0];
				pipe_contex[1].gdc_bin = gdc_bin[1];
			}			
		}
	}
	ret = create_and_run_vflow(&pipe_contex[0]);
	ERR_CON_EQ(ret, 0);
	//copy_config(&pipe_contex[1].sensor_config, vp_sensor_config_list[v_host_info[1].sensor_index]);
	memcpy(&pipe_contex[1].sensor_config, vp_sensor_config_list[v_host_info[1].sensor_index], sizeof(vp_sensor_config_t));
	ret = vp_sensor_fixed_mipi_host_1(v_host_info[1].host_num, &pipe_contex[1].sensor_config, &pipe_contex[1].csi_config);
	ERR_CON_EQ(ret, 0);
	ret = create_and_run_vflow(&pipe_contex[1]);
	ERR_CON_EQ(ret, 0);

  } else {
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
	if (cap_info_.gdc_enable_) {
		if (cap_info_.rotation_ != 0) {
			vp_sensor_config_t *sensor_conf = &pipe_contex[0].sensor_config;
			auto gdc_bin = gen_gdc_bin_rotation(sensor_conf->isp_cfg->isp_attr.size.width, sensor_conf->isp_cfg->isp_attr.size.height, cap_info_.width, cap_info_.height, cap_info_.rotation_);
			if (gdc_bin) {
				gdc_bin_buf_.push_back(gdc_bin);
				pipe_contex[0].gdc_bin_r = gdc_bin;
			}
		}
		vp_sensor_config_t *sensor_cof = &pipe_contex[0].sensor_config;
		if (cam_info_.size() > 0) {
			sensor_msgs::msg::CameraInfo cal_cam_info;
			if (pipe_contex[0].gdc_bin_r != nullptr) {
				auto gdc_bin = gen_gdc_bin(cap_info_.width, cap_info_.height, cap_info_.width, cap_info_.height, &cam_info_[0], &cal_cam_info);
				//auto gdc_bin = gen_gdc_bin_json("./gdc_bin_custom_config.json");
				if (gdc_bin) {
					gdc_bin_buf_.push_back(gdc_bin);
					pipe_contex[0].gdc_bin = gdc_bin;
					cal_cam_info_.push_back(cal_cam_info);
				}
			} else {
				auto gdc_bin = gen_gdc_bin(sensor_cof->isp_cfg->isp_attr.size.width, sensor_cof->isp_cfg->isp_attr.size.height, cap_info_.width, cap_info_.height, &cam_info_[0], &cal_cam_info);
				//auto gdc_bin = gen_gdc_bin_json("./gdc_bin_custom_config.json");
				if (gdc_bin) {
					gdc_bin_buf_.push_back(gdc_bin);
					pipe_contex[0].gdc_bin = gdc_bin;
					cal_cam_info_.push_back(cal_cam_info);
				}
			}

		}
	}
	ret = create_and_run_vflow(&pipe_contex[0]);
	ERR_CON_EQ(ret, 0);
  }

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
	contex.pym_node_handle = hbn_vflow_get_vnode_handle(contex.vflow_fd, HB_VSE, 0);
	printf("read_pym_data pym_node_handle[%d] = %ld\n",i, contex.pym_node_handle);
	if (contex.pym_node_handle <= 0) {
		printf("get vflow %d pym handle error\n", i);
	}
  }
  started_ = true;
  if ((cap_info_.device_mode_.compare("dual") == 0) && 
   	 ((cap_info_.dual_combine_ == 1) || (cap_info_.dual_combine_ == 2))){
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
				if (q_combine_buff_.size() > 0) {
					buff_ptr = q_combine_buff_.front();
					q_combine_buff_.pop();
				}
			} else if (channel == "right") {
				if (q_v_buff_[1].size() > 0) {
					buff_ptr = q_v_buff_[1].front();
					q_v_buff_[1].pop();
				}
			} else {
				if (q_v_buff_[0].size() > 0) {
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
	if ((channel == "right") && (pipe_contex.size() == 2)) {
		nChnID = 1;
	} else if ((channel == "left") || (channel == "single")) {
		nChnID = 0;
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
		"x5 camera isn't support channel : %s", channel);
		return -1;
	}
	
	ret = getVnodeFrame(pipe_contex[nChnID].pym_node_handle, 0, nVOutW, nVOutH, &stride,
						frame_buf, bufsize, len, &timestamp, &frameId, gray);
	if (ret != 0) {
		printf("hbn_vnode_getframe VSE channel 0 failed nChnID = %d,ret = %d\n", nChnID,ret);
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
	// hbn_vnode_image_t out_img;
	// int ret = hbn_vnode_getframe(handle, channel, 1000, &out_img);

	hbn_vnode_image_group_t out_img;
	int ret = hbn_vnode_getframe_group(handle, channel, 1000, &out_img);

	if (ret != 0) {
		printf("hbn_vnode_getframe VSE channel  = %d,ret = %d failed\n", channel,ret);
		return -1;
	}
	//hb_mem_invalidate_buf_with_vaddr((uint64_t)out_img.buffer.virt_addr[0],out_img.buffer.size[0]);

	//hb_mem_invalidate_buf_with_vaddr((uint64_t)out_img.buffer.virt_addr[1],out_img.buffer.size[1]);

	//*timestamp = out_img.info.trig_tv.tv_sec * 1e9 + out_img.info.trig_tv.tv_usec * 1e3;
	//*timestamp = out_img.info.tv.tv_sec * 1e9 + out_img.info.tv.tv_usec * 1e3;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

	uint64_t timestamp_1 = tv.tv_sec * 1e9 + tv.tv_usec * 1e3;
	uint64_t timestamp_2 = ts.tv_sec * 1e9 + ts.tv_nsec;
    //RCLCPP_DEBUG(rclcpp::get_logger("mipi_cap"),
    //        "capture timestamps= %lu, sys_timestamps= %lu, tv.sec=%d, tv.nsec=%d, trig_tv.sec=%d, trig_tv.nsec=%d", 
	//		out_img.info.timestamps, out_img.info.sys_timestamps, out_img.info.tv.tv_sec, out_img.info.tv.tv_usec, out_img.info.trig_tv.tv_sec, out_img.info.trig_tv.tv_usec);

    RCLCPP_DEBUG(rclcpp::get_logger("mipi_cap"),
            "capture timestamps= %lu, tv.sec=%d, tv.nsec=%d, trig_tv.sec=%d, trig_tv.nsec=%d", 
			out_img.info.timestamps, out_img.info.tv.tv_sec, out_img.info.tv.tv_usec, out_img.info.trig_tv.tv_sec, out_img.info.trig_tv.tv_usec);

	RCLCPP_DEBUG(rclcpp::get_logger("mipi_cap"),
            "system tv.sec=%d, tv.tv_usec=%d, ts.sec=%d, ts.nsec=%d", 
			tv.tv_sec, tv.tv_usec, ts.tv_sec, ts.tv_nsec);

	if ("realtime" == cap_info_.frame_ts_type_) {
		*timestamp = out_img.info.timestamps + (timestamp_1 - timestamp_2);
	} else {
		*timestamp = out_img.info.timestamps;
	} 
	//*timestamp = out_img.info.sys_timestamps;
	//*timestamp = out_img.info.timestamps;
	*frame_id = out_img.info.frame_id;

	RCLCPP_DEBUG(rclcpp::get_logger("mipi_cap"),
		"capture laps ms= %d", ((timestamp_2 - out_img.info.timestamps)/1000000));
	
	//std::cout << "getVnodeFrame--system time sec:" << tv.tv_sec << ", image time sec:" << out_img.info.tv.tv_sec
	//          << ", trig time sec:" << out_img.info.trig_tv.tv_sec 
	//		  << ", image timestamps(/1e9) sec:" << out_img.info.timestamps / 1e9 <<  std::endl;

	//std::cout << "getVnodeFrame--system time sec:" << tv.tv_sec << ", timestamp time sec:" << (int)(*timestamp / 1e9) <<  std::endl;
	
	*stride = out_img.buf_group.graph_group[0].stride;
	*width = out_img.buf_group.graph_group[0].width;
	*height = out_img.buf_group.graph_group[0].height;
	if (gray == true) {
		*len = out_img.buf_group.graph_group[0].size[0];
	} else {
		*len = out_img.buf_group.graph_group[0].size[0] + out_img.buf_group.graph_group[0].size[1];
	}
	if (bufsize < *len) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
			"buf size(%d) < frame size(%d), stride(%d), width(%d), height(%d)", bufsize, *len, *stride, *width, *height);
		hbn_vnode_releaseframe_group(handle, channel, &out_img);
		*len = 0;
		return -1;
	}
	memcpy(frame_buf, out_img.buf_group.graph_group[0].virt_addr[0], out_img.buf_group.graph_group[0].size[0]);
	if (gray == false) {
		memcpy(frame_buf + out_img.buf_group.graph_group[0].size[0], out_img.buf_group.graph_group[0].virt_addr[1], out_img.buf_group.graph_group[0].size[1]);
	}
	hbn_vnode_releaseframe_group(handle, channel, &out_img);
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
  q_v_buff_.resize(2);
  int diff_time = 500000000 / cap_info_.fps;

  hbn_vnode_get_fd(pipe_contex[0].pym_node_handle, 0, &ochn_fd[0]);
  hbn_vnode_get_fd(pipe_contex[1].pym_node_handle, 0, &ochn_fd[1]);

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
						ret = getVnodeFrame(pipe_contex[i].pym_node_handle, 0, &buff_ptr[i]->width, &buff_ptr[i]->height, &buff_ptr[i]->stride,
								buff_ptr[i]->buff, buff_ptr[i]->buff_size, &buff_ptr[i]->data_size, &buff_ptr[i]->timestamp, &buff_ptr[i]->frame_id);
						if (ret != 0) {
							printf("hbn_vnode_getframe VSE channel = %d failed ,ret = %d\n", i,ret);
							std::unique_lock<std::mutex> lk(queue_mtx_);
							q_buff_empty_.push(buff_ptr[i]);
							buff_ptr[i] = nullptr;
						}
						//std::cout << "getVnodeFrame--channel=" << i << ",timestamp=" << buff_ptr[i]->timestamp<<",frame_id="<< buff_ptr[i]->frame_id << std::endl;
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
	printf("creat_camera_node cam_fd = %ld\n", cam_fd);
	return 0;
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

#if 0
	if(pipe_contex->csi_config.mclk_is_not_configed){
		//设备树中没有配置mclk：使用外部晶振
		printf("csi%d ignore mclk ex attr, because not config mclk.\n",
			pipe_contex->csi_config.index);
		vin_attr_ex.vin_attr_ex_mask = 0x00;
	}else{
		vin_attr_ex.vin_attr_ex_mask = 0x80;	//bit7 for mclk
		vin_attr_ex.mclk_ex_attr.mclk_freq = 24000000; // 24MHz
		vin_attr_ex_mask = vin_attr_ex.vin_attr_ex_mask;
	}
#endif

	hw_id = sensor_config.vin_attr->vin_node_attr.cim_attr.mipi_rx;
	std::cout << "mipi_rx:" << hw_id << std::endl;
	if (hw_id != 1) {
		vin_online_isp = 0;
	} else {
		vin_online_isp = 1;
	}

	if(vin_online_isp){
		sensor_config.vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].ddr_en = 0;
		sensor_config.vin_attr->vin_node_attr.cim_attr.cim_isp_flyby = 1;
	}else{
		sensor_config.vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].ddr_en = 1;
		sensor_config.vin_attr->vin_node_attr.cim_attr.cim_isp_flyby = 0;
	}

	ret = hbn_vnode_open(HB_VIN, hw_id, AUTO_ALLOC_ID, &pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);
	// 设置基本属性
	ret = hbn_vnode_set_attr(pipe_contex->vin_node_handle, sensor_config.vin_attr);
	ERR_CON_EQ(ret, 0);
	// 设置输入通道的属性
	ret = hbn_vnode_set_ichn_attr(pipe_contex->vin_node_handle, chn_id, &sensor_config.vin_attr->vin_ichn_attr);
	ERR_CON_EQ(ret, 0);
	// 设置输出通道的属性
	ret = hbn_vnode_set_ochn_attr(pipe_contex->vin_node_handle, chn_id, &sensor_config.vin_attr->vin_ochn_attr[VIN_MAIN_FRAME]);
	ERR_CON_EQ(ret, 0);

	if (sensor_config.vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].ddr_en) {
		hbn_buf_alloc_attr_t alloc_attr_raw = {0};
		alloc_attr_raw.buffers_num = 3;
		alloc_attr_raw.is_contig = 1;
		alloc_attr_raw.flags = HB_MEM_USAGE_CPU_READ_OFTEN
							| HB_MEM_USAGE_CPU_WRITE_OFTEN
							| HB_MEM_USAGE_CACHED;
		ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->vin_node_handle, chn_id, &alloc_attr_raw);
		ERR_CON_EQ(ret, 0);
	}
#if 0
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
#endif
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
	auto vi_hw_id = sensor_config.vin_attr->vin_node_attr.cim_attr.mipi_rx;
	if((vi_hw_id == 0) || (vi_hw_id == 4)){
		vin_online_isp = 0;
		//isp_attr->channel.slot_id = 4; //4 - 11
		sensor_config.isp_cfg->isp_attr.channel.slot_id = 4;
	}else{
		vin_online_isp = 1;
		//isp_attr->channel.slot_id = 0;
		sensor_config.isp_cfg->isp_attr.channel.slot_id = 0;
	}

	//vin_online_isp
	if(vin_online_isp){
		sensor_config.isp_cfg->isp_attr.sched_mode = SCHED_MODE_PASS_THRU;
	}else{
		sensor_config.isp_cfg->isp_attr.sched_mode = SCHED_MODE_MANUAL;
	}

	if(isp_online_ynr){
		sensor_config.isp_cfg->ochn_attr.stream_output_mode = STREAM_OUTPUT_MODE_ENABLE;
		sensor_config.isp_cfg->ochn_attr.axi_output_mode = AXI_OUTPUT_MODE_DISABLE;
	}else{
		printf("isp must online ynr.\n");
		//return -1;
	}

	auto hw_id = sensor_config.isp_cfg->isp_attr.channel.hw_id;

	ret = hbn_vnode_open(HB_ISP, hw_id, AUTO_ALLOC_ID, &pipe_contex->isp_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_attr(pipe_contex->isp_node_handle, sensor_config.isp_cfg);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ochn_attr(pipe_contex->isp_node_handle, chn_id, &sensor_config.isp_cfg->ochn_attr);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ichn_attr(pipe_contex->isp_node_handle, chn_id, &sensor_config.isp_cfg->ichn_attr);
	ERR_CON_EQ(ret, 0);
	if (!isp_online_ynr) {
		alloc_attr.buffers_num = 3;
		alloc_attr.is_contig = 1;
		alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
							| HB_MEM_USAGE_CPU_WRITE_OFTEN
							| HB_MEM_USAGE_CACHED;
		ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->isp_node_handle, chn_id, &alloc_attr);
		ERR_CON_EQ(ret, 0);
	}
	
	isp_ichn_attr_t isp_ichn_attr;
	ret = hbn_vnode_get_ichn_attr(pipe_contex->isp_node_handle, chn_id, &isp_ichn_attr);
	ERR_CON_EQ(ret, 0);
	return 0;
}

int HobotMipiCapIml::creat_ynr_node(pipe_contex_t *pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	hbn_buf_alloc_attr_t alloc_attr = {0};
	uint32_t chn_id = 0;
	int ret = 0;
	vp_sensor_config_t& sensor_config = pipe_contex->sensor_config;

	ret = hbn_vnode_open(HB_YNR, 1, AUTO_ALLOC_ID, &pipe_contex->ynr_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_attr(pipe_contex->ynr_node_handle, sensor_config.ynr_attr);
	ERR_CON_EQ(ret, 0);

#if 0
	struct hobot_ynr_channel_input_config ynr_ichn_cfg;
	ynr_ichn_cfg.ch_img_width = sensor_config.ynr_cfg->ch_img_width;
	ynr_ichn_cfg.ch_img_height = sensor_config.ynr_cfg->ch_img_height;
	ret = hbn_vnode_set_ochn_attr(pipe_contex->ynr_node_handle, chn_id, &ynr_ichn_cfg);
	ERR_CON_EQ(ret, 0);

	struct hobot_ynr_channel_output_config ynr_ochn_cfg;
	ynr_ochn_cfg.ch_nr3d_pix_out_dma_byps = 0;
	ynr_ochn_cfg.ch_nr3d_debug_en = 0;
	ret = hbn_vnode_set_ichn_attr(pipe_contex->ynr_node_handle, chn_id, &ynr_ochn_cfg);
	ERR_CON_EQ(ret, 0);
#else
	struct hobot_ynr_channel_input_config channel_input_cfg = {0};
	ret = hbn_vnode_set_ichn_attr(pipe_contex->ynr_node_handle, 0, &channel_input_cfg);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ichn_attr(pipe_contex->ynr_node_handle, 1, &channel_input_cfg);
	ERR_CON_EQ(ret, 0);

	struct hobot_ynr_channel_output_config channel_output_cfg = {0};
	ret = hbn_vnode_set_ochn_attr(pipe_contex->ynr_node_handle, 0, &channel_output_cfg);
	ERR_CON_EQ(ret, 0);
#endif
	if (sensor_config.ynr_attr->nr3d_en == 1u) {
		alloc_attr.buffers_num = 3;
		alloc_attr.is_contig = 1;
		alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
							| HB_MEM_USAGE_CPU_WRITE_OFTEN
							| HB_MEM_USAGE_CACHED;
		ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->ynr_node_handle, chn_id, &alloc_attr);
		ERR_CON_EQ(ret, 0);
	}
	
	return 0;
}

static int check_pym_config(int src_width, int src_height, int width, int height, 
					int &bl_width, int &bl_height,int &bl_stride, int &roi_sel, int &roi_layer) {
	if ((src_width & 1) || (src_height & 1) || (width & 1) || (height & 1)) {
		std::cout << "width and height isn't charmonium, width:" << width << ",height:" << height << std::endl;
		return -1;
	}
	roi_sel = 0;
	roi_layer = 0;
	bl_width = src_width;
	bl_height = src_height;
	bl_stride = src_width;
	//int bl_width_0 = src_width;
	//int bl_height_0 = src_height;
	int bl_width_2 = (src_width >> 1) & ~1;
	int bl_height_2 = (src_height >> 1) & ~1;
	int bl_width_4 = (src_width >> 2) & ~1;
	int bl_height_4 = (src_height >> 2) & ~1;
	int bl_width_8 = (src_width >> 3) & ~1;
	int bl_height_8 = (src_height >> 3) & ~1;
	int bl_width_16 = (src_width >> 4) & ~1;
	int bl_height_16 = (src_height >> 4) & ~1;
	if ((width > src_width) || (height > src_height) || (width < bl_width_16) || (height < bl_height_16) || (height < 134)) {
		std::cout << "width and height over rang, width:" << width << ",height:" << height << std::endl;
		return -1;
	} else if ((width <= src_width) && (height <= src_height) && (width > bl_width_2) && (height > bl_height_2)) {
		roi_sel = 0;
		roi_layer = 0;
		bl_width = src_width;
		bl_height = src_height;
		bl_stride = src_width;
	} else if ((width <= bl_width_2) && (height <= bl_height_2) && (width > bl_width_4) && (height > bl_height_4)) {
		roi_sel = 1;
		roi_layer = 0;
		bl_width = bl_width_2;
		bl_height = bl_height_2;
		bl_stride = bl_width_2;
	} else if ((width <= bl_width_4) && (height <= bl_height_4) && (width > bl_width_8) && (height > bl_height_8)) {
		roi_sel = 1;
		roi_layer = 1;
		bl_width = bl_width_4;
		bl_height = bl_height_4;
		bl_stride = bl_width_4;
	} else if ((width <= bl_width_8) && (height <= bl_height_8) && (width > bl_width_16) && (height > bl_height_16)) {
		roi_sel = 1;
		roi_layer = 2;
		bl_width = bl_width_8;
		bl_height = bl_height_8;
		bl_stride = bl_width_8;
	} else if ((width <= bl_width_16) && (height <= bl_height_16) && (height >= 134)) {
		roi_sel = 1;
		roi_layer = 3;
		bl_width = bl_width_16;
		bl_height = bl_height_16;
		bl_stride = bl_width_16;
	} else if ((width == bl_width_2) && (height > bl_height_2) && (height < src_height)) {
		roi_sel = 0;
		roi_layer = 0;
		bl_width = src_width-2;
		bl_height = src_height;
		bl_stride = src_width;
	} else if ((height == bl_height_2) && (width > bl_width_2) && (width < src_width)) {
		roi_sel = 0;
		roi_layer = 0;
		bl_width = src_width;
		bl_height = src_height-2;
		bl_stride = src_width;
	} else if ((width == bl_width_4) && (height > bl_height_4) && (height < bl_height_2)) {
		roi_sel = 1;
		roi_layer = 0;
		bl_width = bl_width_2-2;
		bl_height = bl_height_2;
		bl_stride = bl_width_2;
	} else if ((height == bl_height_4) && (width > bl_width_4) && (width < bl_width_2)) {
		roi_sel = 1;
		roi_layer = 0;
		bl_width = bl_width_2;
		bl_height = bl_height_2-2;
		bl_stride = bl_width_2;
	} else if ((width == bl_width_8) && (height > bl_height_8) && (height < bl_height_4)) {
		roi_sel = 1;
		roi_layer = 1;
		bl_width = bl_width_4-2;
		bl_height = bl_height_4;
		bl_stride = bl_width_4;
	} else if ((height == bl_height_8) && (width > bl_width_8) && (width < bl_width_4)) {
		roi_sel = 1;
		roi_layer = 1;
		bl_width = bl_width_4;
		bl_height = bl_height_4-2;
		bl_stride = bl_width_4;
	} else {
		std::cout << "width and height over rang, width:" << width << ",height:" << height << std::endl;
		return -1;
	}
	return 0;
}


int HobotMipiCapIml::creat_pym_node(pipe_contex_t *pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
#if 1

	auto vi_hw_id = pipe_contex->sensor_config.vin_attr->vin_node_attr.cim_attr.mipi_rx;
	if(vi_hw_id == 4){
		pipe_contex->sensor_config.pym_cfg->slot_id = pipe_contex->sensor_config.isp_cfg->isp_attr.channel.slot_id;
		pipe_contex->sensor_config.pym_cfg->hw_id = 1;
		pipe_contex->sensor_config.pym_cfg->pym_mode = 1;
	}

	int src_width = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_width;
	int src_height = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_height;
	int roi_sel = 0;
	int roi_layer = 0;
	int bl_width = src_width;
	int bl_height = src_height;
	int bl_stride = src_width;
	if (check_pym_config(src_width, src_height, pipe_contex->cap_info_->width,
			pipe_contex->cap_info_->height, bl_width, bl_height, bl_stride, roi_sel, roi_layer) == -1) {
		return -1;
	}
	std::cout << "creat_pym_node--roi_sel:" << roi_sel <<",roi_layer:" <<roi_layer<< ",bl_width:"<<bl_width <<",bl_height:"<<bl_height<<std::endl;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_sel[0] = roi_sel;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_layer[0] = roi_layer;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].region_width = bl_width;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].region_height = bl_height;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].wstride_uv = pipe_contex->cap_info_->width;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].wstride_y = pipe_contex->cap_info_->width;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].out_width = pipe_contex->cap_info_->width;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].out_height = pipe_contex->cap_info_->height;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].vstride = pipe_contex->cap_info_->height;
#endif
	int ret = 0;
	uint32_t chn_id = 0;
	uint32_t hw_id = pipe_contex->sensor_config.pym_cfg->hw_id;
	hbn_buf_alloc_attr_t alloc_attr = {0};
	
	ret = hbn_vnode_open(HB_PYM, hw_id, AUTO_ALLOC_ID, &pipe_contex->pym_node_handle);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_attr(pipe_contex->pym_node_handle, pipe_contex->sensor_config.pym_cfg);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ichn_attr(pipe_contex->pym_node_handle, chn_id, pipe_contex->sensor_config.pym_cfg);
	ERR_CON_EQ(ret, 0);



	ret = hbn_vnode_set_ochn_attr(pipe_contex->pym_node_handle, chn_id, pipe_contex->sensor_config.pym_cfg);
	ERR_CON_EQ(ret, 0);
	alloc_attr.buffers_num = 3;
	alloc_attr.is_contig = 1;
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
						| HB_MEM_USAGE_CPU_WRITE_OFTEN
						| HB_MEM_USAGE_CACHED
						| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->pym_node_handle, chn_id, &alloc_attr);
	ERR_CON_EQ(ret, 0);

	return 0;
}

int HobotMipiCapIml::creat_gdc_node_r(pipe_contex_t *pipe_contex) {
	if ((pipe_contex == nullptr) || (pipe_contex->gdc_bin_r == nullptr)) {
		return -1;
	}
	int ret = 0;
	uint32_t chn_id = 0;
	isp_cfg_t isp_attr;
	pipe_contex->gdc_init_valid_r = 0;
	//ret = hbn_vnode_get_attr(pipe_contex->isp_node_handle, chn_id, &isp_attr);
	//ERR_CON_EQ(ret, 0);
	int input_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
	int input_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
	//int input_width;
	//int input_height;

	uint32_t hw_id = 0;
	ret = hbn_vnode_open(HB_GDC, hw_id, AUTO_ALLOC_ID, &pipe_contex->gdc_node_handle_r);
	ERR_CON_EQ(ret, 0);
	gdc_attr_t gdc_attr = {0};
	gdc_attr.config_addr = pipe_contex->gdc_bin_r->bin_buf->phys_addr;
	gdc_attr.config_size = pipe_contex->gdc_bin_r->bin_buf->size;
	gdc_attr.binary_ion_id = pipe_contex->gdc_bin_r->bin_buf->share_id;
	gdc_attr.binary_offset = pipe_contex->gdc_bin_r->bin_buf->offset;
	gdc_attr.total_planes = 2;
	gdc_attr.div_width = input_width;
	gdc_attr.div_height = input_height;
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
	//gdc_ochn_attr.output_width = input_width;
	//gdc_ochn_attr.output_height = input_height;
	//gdc_ochn_attr.output_stride = input_width;
	gdc_ochn_attr.output_width = pipe_contex->cap_info_->width;
	gdc_ochn_attr.output_height = pipe_contex->cap_info_->height;
	gdc_ochn_attr.output_stride = pipe_contex->cap_info_->width;
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
	if (pipe_contex == nullptr) {
		return -1;
	}
	int ret = 0;
	uint32_t chn_id = 0;
	isp_ichn_attr_t isp_ichn_attr;
	pipe_contex->gdc_bin_buf_is_valid = 0;
	pipe_contex->gdc_init_valid = 0;
#if 0
    const char* gdc_bin_file = vp_gdc_get_bin_file(vp_vflow_contex->gdc_info.sensor_name);
	if(gdc_bin_file == NULL){
		SC_LOGE("%s is enable gdc, but gdc bin file is not set.", vp_vflow_contex->gdc_info.sensor_name);
		return -1;
	}
#endif

	int input_width;
	int input_height;
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
		//input_width = isp_ichn_attr.width;
		//input_height = isp_ichn_attr.height;
	}

    auto gdc_bin = get_gdc_bin(pipe_contex->cap_info_->gdc_bin_file_);
	if (gdc_bin == nullptr && pipe_contex->gdc_bin == nullptr) {
		return -1;
	}
	if (gdc_bin) {
		pipe_contex->gdc_bin = gdc_bin;
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
	int isp_bind = 0;
	if (pipe_contex == nullptr) {
		return -1;
	}
	int32_t ret = 0;
#if 0
    if (pipe_contex->cap_info_->device_mode_.compare("dual") == 0) {
		//pipe_contex->sensor_config.isp_attr->input_mode = 2;
		isp_bind = 0;
		if (pipe_contex->cap_info_->lpwm_enable_) {
			pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
			pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
			int fps_rate = (1000000 / pipe_contex->cap_info_->fps);
			pipe_contex->sensor_config.camera_config->sensor_mode = 6;
			//pipe_contex->sensor_config.vin_node_attr->lpwm_attr.enable = 1;
			for (auto& attr : pipe_contex->sensor_config.vin_node_attr->lpwm_attr.lpwm_chn_attr) {
				attr.period = fps_rate;
			}
		} else {
			pipe_contex->sensor_config.camera_config->sensor_mode = 1;
			//pipe_contex->sensor_config.vin_node_attr->lpwm_attr.enable = 0;
		}
	} else {
		pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
		pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
		//int fps_rate = (1000000 / pipe_contex->cap_info_->fps);
		//for (auto& attr : pipe_contex->sensor_config.vin_node_attr->lpwm_attr.lpwm_chn_attr) {
		//	attr.period = fps_rate;
		//}
	}
#endif
	// 创建pipeline中的每个node
	ret = creat_camera_node(pipe_contex->sensor_config.camera_config, &pipe_contex->cam_fd);
	ERR_CON_EQ(ret, 0);
	ret = creat_vin_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	ret = creat_isp_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	if (isp_online_ynr) {
		ret = creat_ynr_node(pipe_contex);
		ERR_CON_EQ(ret, 0);
	}
	if (cap_info_.gdc_enable_) {
		creat_gdc_node_r(pipe_contex);
		creat_gdc_node(pipe_contex);
	}
	ret = creat_pym_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	// 创建HBN flow
	ret = hbn_vflow_create(&pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle);
	ERR_CON_EQ(ret, 0);
	if (isp_online_ynr) {
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
								pipe_contex->ynr_node_handle);
		ERR_CON_EQ(ret, 0);
	}

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
							pipe_contex->pym_node_handle);
	ERR_CON_EQ(ret, 0);

	if (vin_online_isp) {
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->vin_node_handle,
								1,
								pipe_contex->isp_node_handle,
								0);
		ERR_CON_EQ(ret, 0);
	} else {
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->vin_node_handle,
								0,
								pipe_contex->isp_node_handle,
								0);
		ERR_CON_EQ(ret, 0);
	}

#if 0
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
							pipe_contex->gdc_node_handle,
							0);
		ERR_CON_EQ(ret, 0);
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->gdc_node_handle,
							0,
							pipe_contex->pym_node_handle,
							0);
		ERR_CON_EQ(ret, 0);
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
							pipe_contex->pym_node_handle,
							0);
		ERR_CON_EQ(ret, 0);
	} else if (pipe_contex->gdc_init_valid == 1) {
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc cal.\n");
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							0,
							pipe_contex->gdc_node_handle,
							0);
		ERR_CON_EQ(ret, 0);
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->gdc_node_handle,
							0,
							pipe_contex->pym_node_handle,
							0);
		ERR_CON_EQ(ret, 0);
	} else {
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							0,
							pipe_contex->pym_node_handle,
							0);
		ERR_CON_EQ(ret, 0);
	}
#else
	if (isp_online_ynr) {
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							1,
							pipe_contex->ynr_node_handle,
							0);
		ERR_CON_EQ(ret, 0);
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->ynr_node_handle,
							1,
							pipe_contex->pym_node_handle,
							0);
		ERR_CON_EQ(ret, 0);
	} else {
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							0,
							pipe_contex->pym_node_handle,
							0);
		ERR_CON_EQ(ret, 0);
	}
#endif

	ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd,
							pipe_contex->vin_node_handle);
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

	FILE *fp = fopen(gdc_bin_file.c_str(), "r");
	if (fp == NULL) {
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),"gdc bin file %s open failed\n", gdc_bin_file);
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

std::vector<std::shared_ptr<GdcBinBuf_ST>> HobotMipiCapIml::gen_gdc_bin_stereo(int gdc_width, int gdc_height,int out_width, int out_height,
		std::vector<sensor_msgs::msg::CameraInfo> &cam_info, std::vector<sensor_msgs::msg::CameraInfo> &cal_cam_info) {
	std::vector<std::shared_ptr<GdcBinBuf_ST>> gdc_bin_buf;

#if 1
	if (gdc_width <= 0 || gdc_height<= 0 || out_width <= 0 || out_height <= 0 || cam_info.size() != 2) {
		return gdc_bin_buf;
	}
    // cam param
	cal_cam_info.clear();
    cv::Mat Rl, Rr, Pl, Pr, Q;
    cv::Mat Kl, Kr, Dl, Dr, R_rl, t_rl;
    cv::Mat undistmap1l, undistmap2l, undistmap1r, undistmap2r;
	float camera_cx, camera_cy, camera_fx, camera_fy, base_line;
	float gdc_width_scale, gdc_height_scale, out_width_scale, out_height_scale;

	gdc_width_scale = gdc_width / static_cast<float>(cam_info[0].width);
	gdc_height_scale = gdc_height / static_cast<float>(cam_info[0].height);
	out_width_scale = out_width / static_cast<float>(gdc_width);
	out_height_scale = out_height / static_cast<float>(gdc_height);

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
    cv::Mat tTr = RT(cv::Rect(3, 0, 1, 3));
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
    std::cout << "Kl:\n" << Kl << std::endl;
	std::cout << "Dl:\n" << Dl << std::endl;
	std::cout << "Kr:\n" << Kr << std::endl;
	std::cout << "Dr:\n" << Dr << std::endl;
	std::cout << "R_rl:\n" << R_rl << std::endl;
	std::cout << "t_rl:\n" << t_rl << std::endl;
	cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(gdc_width, gdc_height), R_rl, t_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 0 ,cv::Size(out_width, out_height));
	cv::initUndistortRectifyMap(Kl, Dl, Rl, Pl, cv::Size(out_width, out_height), CV_32FC1, undistmap1l, undistmap2l);
	cv::initUndistortRectifyMap(Kr, Dr, Rr, Pr, cv::Size(out_width, out_height), CV_32FC1, undistmap1r, undistmap2r);

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
	wnds.angle = 0;
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

	wnds.transform = CUSTOM;
	wnds.custom.full_tile_calc = 1;
	wnds.custom.tile_incr_x = 50;
	wnds.custom.tile_incr_y = 50;
	wnds.custom.w = out_width-1;
	wnds.custom.h = out_height-1;
	wnds.custom.centerx = out_width / 2 - 1;
	wnds.custom.centery = out_height / 2 - 1;

	std::vector<point_t> bin_map(out_width * out_height);
	std::transform(undistmap1l.ptr<float>(), undistmap1l.ptr<float>() + undistmap1l.total(),
			undistmap2l.ptr<float>(), bin_map.begin(),
			[](float x, float y) {
				point_t p;
				p.x = static_cast<double>(x);
				p.y = static_cast<double>(y);
				return p;
			});

	wnds.custom.points = bin_map.data();

	uint32_t *bin_buf_ptr = nullptr;
	uint64_t bin_buf_size;
	int64_t alloc_flags = 0;
	int offset = 0;
#if 0
	auto ret = hbn_gen_gdc_bin(&gdc_param, &wnds, 1, (uint32_t**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_gen_gdc_bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
#else
	auto ret = hbn_gen_gdc_cfg(&gdc_param, &wnds, 1, (void**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_gen_gdc_bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
#endif

    hb_mem_common_buf_t *bin_buf = new hb_mem_common_buf_t;
	memset(bin_buf, 0, sizeof(hb_mem_common_buf_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
	ret = hb_mem_alloc_com_buf(bin_buf_size, alloc_flags, bin_buf);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        //hbn_free_gdc_bin(bin_buf_ptr);
		hbn_free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        //hbn_free_gdc_bin(bin_buf_ptr);
		hbn_free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
	//hbn_free_gdc_bin(bin_buf_ptr);
	hbn_free_gdc_cfg(bin_buf_ptr);
	auto gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;
	gdc_bin_buf.push_back(gdc_bin_ptr);


	std::transform(undistmap1r.ptr<float>(), undistmap1r.ptr<float>() + undistmap1r.total(),
			undistmap2r.ptr<float>(), bin_map.begin(),
			[](float x, float y) {
				point_t p;
				p.x = static_cast<double>(x);
				p.y = static_cast<double>(y);
				return p;
			});

	wnds.custom.points = bin_map.data();

	bin_buf_ptr = nullptr;
#if 0
	ret = hbn_gen_gdc_bin(&gdc_param, &wnds, 1, (uint32_t**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_gen_gdc_bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
#else
	ret = hbn_gen_gdc_cfg(&gdc_param, &wnds, 1, (void**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_gen_gdc_bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
#endif

    bin_buf = new hb_mem_common_buf_t;
	memset(bin_buf, 0, sizeof(hb_mem_common_buf_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
	ret = hb_mem_alloc_com_buf(bin_buf_size, alloc_flags, bin_buf);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        //hbn_free_gdc_bin(bin_buf_ptr);
		hbn_free_gdc_cfg(bin_buf_ptr);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}

	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        //hbn_free_gdc_bin(bin_buf_ptr);
		hbn_free_gdc_cfg(bin_buf_ptr);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
	//hbn_free_gdc_bin(bin_buf_ptr);
	hbn_free_gdc_cfg(bin_buf_ptr);
	gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;
	gdc_bin_buf.push_back(gdc_bin_ptr);
	

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

	std::cout << "Kl:" << std::endl
			<< Kl << std::endl
			<< "Dl:" << std::endl
			<< Dl << std::endl
			<< "Kr: " << std::endl
			<< Kr << std::endl
			<< "Dr:" << std::endl
			<< Dr << std::endl
			<< "R, t: " << std::endl
			<< R_rl << std::endl
			<< t_rl << std::endl
			<< "calib file width, height: " << cam_info[0].width << ", " << cam_info[0].height << std::endl
			<< "gdc_width_scale, gdc_height_scale: " << gdc_width_scale << ", " << gdc_height_scale << std::endl
			<< "rectify [f, cx, cy, baseline]: " << "[" << camera_fx << ", " << camera_cx << ", " << camera_cy << ", " << base_line << "]" << std::endl
			<< std::endl;

	return gdc_bin_buf;
#else
	return gdc_bin_buf;
#endif

}

std::shared_ptr<GdcBinBuf_ST> HobotMipiCapIml::gen_gdc_bin(int gdc_width, int gdc_height,int out_width, int out_height,
       sensor_msgs::msg::CameraInfo *cam_info, sensor_msgs::msg::CameraInfo *cal_cam_info) {
	if (gdc_width <= 0 || gdc_height<= 0 || out_width <= 0 || out_height <= 0 ||  cam_info == nullptr || cal_cam_info == nullptr) {
		return nullptr;
	}
	float gdc_width_scale, gdc_height_scale, out_width_scale, out_height_scale;
	gdc_width_scale = gdc_width / static_cast<float>(cam_info->width);
	gdc_height_scale = gdc_height / static_cast<float>(cam_info->height);
	out_width_scale = out_width / static_cast<float>(gdc_width);
	out_height_scale = out_height /  static_cast<float>(gdc_height);
	cv::Mat K, D, R, T, P;
	D = cv::Mat(1, cam_info->d.size(), CV_64F, cam_info->d.data()).clone();
	K = cv::Mat(3, 3, CV_64F, cam_info->k.data()).clone();
	R = cv::Mat(3, 3, CV_64F, cam_info->r.data()).clone();
	P = cv::Mat(3, 4, CV_64F, cam_info->p.data()).clone();
	cv::Mat K_inv = K.inv();
	cv::Mat RT = K_inv * P;
	T = RT(cv::Rect(3, 0, 1, 3));
	K.at<double>(0, 0) *= gdc_width_scale;
	K.at<double>(0, 2) *= gdc_width_scale;
	K.at<double>(1, 1) *= gdc_height_scale;
	K.at<double>(1, 2) *= gdc_height_scale;	

    param_t gdc_param;
	memset(&gdc_param, 0, sizeof(param_t));
	gdc_param.format = FMT_SEMIPLANAR_420;
	gdc_param.in.w = gdc_width;
	gdc_param.in.h = gdc_height;
	gdc_param.out.w = gdc_width;
	gdc_param.out.h = gdc_height;
	gdc_param.x_offset = 0;
	gdc_param.y_offset = 0;
	gdc_param.diameter = gdc_height;
	gdc_param.fov = 180;

	window_t  wnds;
	memset(&wnds, 0, sizeof(window_t));
	wnds.strength = 1.0;
	wnds.strengthY = 1.0;
	wnds.angle = 0;
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
	wnds.out_r.w = gdc_width;
	wnds.out_r.h = gdc_height;
	wnds.input_roi_r.x = 0;
	wnds.input_roi_r.y = 0;
	wnds.input_roi_r.w = gdc_width;
	wnds.input_roi_r.h = gdc_height;
	wnds.pan = 0;
	wnds.tilt = 0;
	wnds.zoom = 1;
	wnds.transform = CUSTOM;
	wnds.custom.full_tile_calc = 1;
	wnds.custom.tile_incr_x = 50;
	wnds.custom.tile_incr_y = 50;
	wnds.custom.w = gdc_width - 1;
	wnds.custom.h = gdc_height - 1;
	wnds.custom.centerx = gdc_width / 2 - 1;
	wnds.custom.centery = gdc_height / 2 - 1;

	cv::Mat undistmap1l, undistmap2l;
	cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, D, cv::Size(gdc_width, gdc_height), 0, cv::Size(gdc_width, gdc_height));
	cv::initUndistortRectifyMap(K, D, cv::Mat(), new_K, cv::Size(gdc_width, gdc_height), CV_32FC1, undistmap1l, undistmap2l);

	std::vector<point_t> bin_map(gdc_width * gdc_height);
	std::transform(undistmap1l.ptr<float>(), undistmap1l.ptr<float>() + undistmap1l.total(),
			undistmap2l.ptr<float>(), bin_map.begin(),
			[](float x, float y) {
				point_t p;
				p.x = static_cast<double>(x);
				p.y = static_cast<double>(y);
				return p;
			});

	wnds.custom.points = bin_map.data();

#if 0
	std::ofstream outFile("./custom_config.txt");
    // 检查文件是否成功打开
    if (outFile) {
		std::ostringstream stream;
		stream << "1" << std::endl;
		stream << "50 50" << std::endl;
		stream << gdc_height << " " << gdc_width << std::endl;
		stream << wnds.custom.centery << " " << wnds.custom.centerx << std::endl;
		point_t *tmp_ptr = bin_map.data();
		for (int i = 0; i < gdc_height; i++) {
			for (int j = 0; j < gdc_width; j++) {
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

	auto ret = hbn_gen_gdc_cfg(&gdc_param, &wnds, 1, (void**)&bin_buf_ptr, &bin_buf_size);
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
        //hbn_free_gdc_bin(bin_buf_ptr);
		hbn_free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        //hbn_free_gdc_bin(bin_buf_ptr);
		hbn_free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	//hbn_free_gdc_bin(bin_buf_ptr);
	hbn_free_gdc_cfg(bin_buf_ptr);
	auto gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;

	cal_cam_info->width = out_width;
    cal_cam_info->height = out_height;
    cal_cam_info->d.resize(cam_info->d.size(),0.0);
	new_K.at<double>(0, 0) *= out_width_scale;
	new_K.at<double>(0, 2) *= out_width_scale;
	new_K.at<double>(1, 1) *= out_height_scale;
	new_K.at<double>(1, 2) *= out_height_scale;	
	std::copy(new_K.ptr<double>(0), new_K.ptr<double>(0) + new_K.total(), cal_cam_info->k.begin());
    cal_cam_info->r = cam_info->r;
	cv::Mat new_P = new_K * RT;
	std::copy(new_P.ptr<double>(0), new_P.ptr<double>(0) + new_P.total(), cal_cam_info->p.begin());

	return gdc_bin_ptr;
}

std::shared_ptr<GdcBinBuf_ST> HobotMipiCapIml::gen_gdc_bin_rotation(int gdc_width, int gdc_height,int out_width, int out_height,
       double rotation) {
	if (gdc_width <= 0 || gdc_height<= 0 || out_width <= 0 || out_height <= 0) {
		return nullptr;
	}
	std::cout << "gen_gdc_bin_rotation---gdc_width:"<<gdc_width<<",gdc_height:"<<gdc_height<<",out_width:"<<out_width<<",out_height:"<<out_height<<",rotation:"<<rotation<<std::endl;
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
	auto ret = hbn_gen_gdc_cfg(&gdc_param, &wnds, 1, (void **)&bin_buf_ptr, &bin_buf_size);
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
        //hbn_free_gdc_bin(bin_buf_ptr);
		hbn_free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        //hbn_free_gdc_bin(bin_buf_ptr);
		hbn_free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	//hbn_free_gdc_bin(bin_buf_ptr);
	hbn_free_gdc_cfg(bin_buf_ptr);
	auto gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;
	return gdc_bin_ptr;
}

std::shared_ptr<GdcBinBuf_ST> HobotMipiCapIml::gen_gdc_bin_json(std::string file) {
#if 0
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
        //hbn_free_gdc_bin(bin_buf_ptr);
		hbn_free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        //hbn_free_gdc_bin(bin_buf_ptr);
		hbn_free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	//hbn_free_gdc_bin(bin_buf_ptr);
	hbn_free_gdc_cfg(bin_buf_ptr);
	auto gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;
	return gdc_bin_ptr;
#else
	return nullptr;
#endif
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


int HobotMipiCapIml::detectEeprom(std::string &device, int &i2c_bus, uint16_t &i2c_addr) {

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
		std::cout << "----------------" << std::endl;
		std::cout << "check_0:" << std::to_string(check_0) << std::endl;
		std::cout << "chip_type:" << chip_type << std::endl;
		std::cout << "checksum:" << std::to_string(checksum) << std::endl;
		std::cout << "sum:" << std::to_string(sum%255) << std::endl;
		std::cout << "--------------" << std::endl;
		if (buf[0] == eeprom_id.check_value) {
			i2c_bus = num;
			i2c_addr = eeprom_id.i2c_dev_addr;
			device = eeprom_id.device_name;
			return 0;
		} else {
			std::cout << "eeprom check failure bus:" << num << ",addr:" << eeprom_id.i2c_dev_addr << ",device_name" << eeprom_id.device_name << std::endl;
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
  if (detectEeprom(device, i2c_bus, i2c_addr) == -1) {
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
