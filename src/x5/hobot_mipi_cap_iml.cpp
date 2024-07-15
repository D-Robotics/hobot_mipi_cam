// Copyright (c) 2022，Horizon Robotics.
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

#include <string>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <regex>
#include <cmath>

#include <sys/select.h>

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
  return 0;
}

int HobotMipiCapIml::init(MIPI_CAP_INFO_ST &info) {
  int ret = 0;
  cap_info_ = info;
  std::vector<int> sensor_v;
  std::vector<int> host_v;

  if (cap_info_.device_mode_.compare("dual") == 0) {
	sensor_v = {3,3};
	host_v = {0,2};
	
	pipe_contex.resize(2);
	pipe_contex[0].cap_info_ = &cap_info_;
	pipe_contex[1].cap_info_ = &cap_info_;

	//copy_config(&pipe_contex[0].sensor_config, vp_sensor_config_list[sensor_v[0]]);
	memcpy(&pipe_contex[0].sensor_config, vp_sensor_config_list[sensor_v[0]], sizeof(vp_sensor_config_t));
	vp_sensor_fixed_mipi_host_1(host_v[0], &pipe_contex[0].sensor_config);
	ret = create_and_run_vflow(&pipe_contex[0]);
	ERR_CON_EQ(ret, 0);
	//copy_config(&pipe_contex[1].sensor_config, vp_sensor_config_list[sensor_v[1]]);
	memcpy(&pipe_contex[1].sensor_config, vp_sensor_config_list[sensor_v[1]], sizeof(vp_sensor_config_t));
	ret = vp_sensor_fixed_mipi_host_1(host_v[1], &pipe_contex[1].sensor_config);
	ERR_CON_EQ(ret, 0);
	ret = create_and_run_vflow(&pipe_contex[1]);
	ERR_CON_EQ(ret, 0);

    //n2d_pipe_contex.cap_info_ = &cap_info_;
	//ret = create_and_run_n2d_vflow(&n2d_pipe_contex);
	//ERR_CON_EQ(ret, 0);

  } else {
	sensor_v = {3};
	host_v = {0};
	pipe_contex.resize(1);
	pipe_contex[0].cap_info_ = &cap_info_;
	memcpy(&pipe_contex[0].sensor_config, vp_sensor_config_list[sensor_v[0]], sizeof(vp_sensor_config_t));
	ret = vp_sensor_fixed_mipi_host_1(host_v[0], &pipe_contex[0].sensor_config);
	ERR_CON_EQ(ret, 0);
	ret = create_and_run_vflow(&pipe_contex[0]);
	ERR_CON_EQ(ret, 0);
  }

  hb_mem_module_open();

  return ret;
}

int HobotMipiCapIml::deInit() {
  int i = 0;
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "x5_cam_deinit start.\n");
	for(auto contex : pipe_contex){
		hbn_vflow_destroy(contex.vflow_fd);
	}

	hb_mem_module_close();
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
	printf("read_vse_data vse_node_handle[%d] = %ld\n",i, contex.vse_node_handle);
	if (contex.vse_node_handle <= 0) {
		printf("get vflow %d vse handle error\n", i);
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
     RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "x5 camera isn't started");
    return -1;
  }
  for(auto contex : pipe_contex){
    ret = hbn_vflow_stop(contex.vflow_fd);
    ERR_CON_EQ(ret, 0);
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x5_mipi_cam_stop end.\n");
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
  int loop = 6;
  if (dual_frame_task_) {
	do {
		std::shared_ptr<VideoBuffer_ST> buff_ptr = nullptr;
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
		usleep(5 * 1000);
	} while (loop-- > 0);

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
	
	ret = getVnodeFrame(pipe_contex[nChnID].vse_node_handle, 0, nVOutW, nVOutH, &stride,
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
	hbn_vnode_image_t out_img;
	int ret = hbn_vnode_getframe(handle, channel, 1000, &out_img);

	if (ret != 0) {
		printf("hbn_vnode_getframe VSE channel  = %d,ret = %d failed\n", channel,ret);
		return -1;
	}
	hb_mem_invalidate_buf_with_vaddr((uint64_t)out_img.buffer.virt_addr[0],out_img.buffer.size[0]);

	hb_mem_invalidate_buf_with_vaddr((uint64_t)out_img.buffer.virt_addr[1],out_img.buffer.size[1]);

	//*timestamp = out_img.info.trig_tv.tv_sec * 1e9 + out_img.info.trig_tv.tv_usec * 1e3;
	//*timestamp = out_img.info.tv.tv_sec * 1e9 + out_img.info.tv.tv_usec * 1e3;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
	
	uint64_t timestamp_1 = tv.tv_sec * 1e9 + tv.tv_usec * 1e3;
	uint64_t timestamp_2 = ts.tv_sec * 1e9 + ts.tv_nsec;
	*timestamp = out_img.info.timestamps + (timestamp_1 - timestamp_2);
	*frame_id = out_img.info.frame_id;
	
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
  q_v_buff_.resize(2);

  hbn_vnode_get_fd(pipe_contex[0].vse_node_handle, 0, &ochn_fd[0]);
  hbn_vnode_get_fd(pipe_contex[1].vse_node_handle, 0, &ochn_fd[1]);

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
		for (int i = 0; i < buff_ptr.size(); i++) {
			if (buff_ptr[i]) {
				q_v_buff_[i].push(buff_ptr[i]);
				buff_ptr[i] = nullptr;
			}
		}
		continue;
    } else {
		for (int i = 0; i < ochn_fd.size(); i++) {
			if (FD_ISSET(ochn_fd[i], &readfds)) {
				if (buff_ptr[i] == nullptr) {
					if (q_v_buff_[i].size() >= 3) {
						buff_ptr[i] = q_v_buff_[i].front();
						q_v_buff_[i].pop();
					} else if (q_buff_empty_.size() > 0) {
						buff_ptr[i] = q_buff_empty_.front();
						q_buff_empty_.pop();
					}
					if (buff_ptr[i]) {
						loss_cnt[i] = 0;
						ret = getVnodeFrame(pipe_contex[i].vse_node_handle, 0, &buff_ptr[i]->width, &buff_ptr[i]->height, &buff_ptr[i]->stride,
								buff_ptr[i]->buff, buff_ptr[i]->buff_size, &buff_ptr[i]->data_size, &buff_ptr[i]->timestamp, &buff_ptr[i]->frame_id);
						if (ret != 0) {
							printf("hbn_vnode_getframe VSE channel = %d failed ,ret = %d\n", i,ret);
							q_buff_empty_.push(buff_ptr[i]);
							buff_ptr[i] = nullptr;
						}
					}
				}
			}
		}

		if (buff_ptr[0] && buff_ptr[1]  && (buff_ptr[0]->width == buff_ptr[1]->width) &&
	   			(buff_ptr[0]->height == buff_ptr[1]->height)) {

			if (std::abs((int)(buff_ptr[0]->timestamp - buff_ptr[1]->timestamp)) < 15000000) {
				if (q_combine_buff_.size() >= 3) {
					combine_buff_ptr = q_combine_buff_.front();
					q_combine_buff_.pop();
				} else if (q_combine_buff_empty_.size() > 0) {
					combine_buff_ptr = q_combine_buff_empty_.front();
					q_combine_buff_empty_.pop();
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
						q_combine_buff_.push(combine_buff_ptr);

					} else {
						q_combine_buff_empty_.push(combine_buff_ptr);
					}
					combine_buff_ptr = nullptr;
				}
				q_v_buff_[0].push(buff_ptr[0]);
				buff_ptr[0] = nullptr;
				q_v_buff_[1].push(buff_ptr[1]);
				buff_ptr[1] = nullptr;
			} else {
				if (buff_ptr[0]->timestamp < buff_ptr[1]->timestamp) {
					q_v_buff_[0].push(buff_ptr[0]);
					buff_ptr[0] = nullptr;
				} else {
					q_v_buff_[1].push(buff_ptr[1]);
					buff_ptr[1] = nullptr;
				}
			} 
		}
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


	hw_id = sensor_config.vin_node_attr->cim_attr.mipi_rx;
	ret = hbn_vnode_open(HB_VIN, hw_id, AUTO_ALLOC_ID, &pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);
	// 设置基本属性
	ret = hbn_vnode_set_attr(pipe_contex->vin_node_handle, sensor_config.vin_node_attr);
	ERR_CON_EQ(ret, 0);
	// 设置输入通道的属性
	ret = hbn_vnode_set_ichn_attr(pipe_contex->vin_node_handle, chn_id, sensor_config.vin_ichn_attr);
	ERR_CON_EQ(ret, 0);
	// 设置输出通道的属性
	ret = hbn_vnode_set_ochn_attr(pipe_contex->vin_node_handle, chn_id, sensor_config.vin_ochn_attr);
	ERR_CON_EQ(ret, 0);
	// 设置额外属性，for mclk

	vin_attr_ex.vin_attr_ex_mask = 0x80;
	vin_attr_ex.mclk_ex_attr.mclk_freq = 24000000; // 24MHz


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
						| HB_MEM_USAGE_CACHED;
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
	isp_ichn_attr_t isp_ichn_attr;
	vse_attr_t vse_attr = {0};
	vse_ichn_attr_t vse_ichn_attr;
	vse_ochn_attr_t vse_ochn_attr;
	ret = hbn_vnode_get_ichn_attr(pipe_contex->isp_node_handle, chn_id, &isp_ichn_attr);
	ERR_CON_EQ(ret, 0);
	int input_width = isp_ichn_attr.width;
	int input_height = isp_ichn_attr.height;


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

	return 0;
}

int HobotMipiCapIml::create_and_run_vflow(pipe_contex_t *pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	int32_t ret = 0;
    if (pipe_contex->cap_info_->device_mode_.compare("dual") == 0) {
		pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
		pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
		int fps_rate = (1000000 / pipe_contex->cap_info_->fps);
		pipe_contex->sensor_config.camera_config->sensor_mode = 6;
		pipe_contex->sensor_config.vin_node_attr->lpwm_attr.enable = 1;
		for (auto& attr : pipe_contex->sensor_config.vin_node_attr->lpwm_attr.lpwm_chn_attr) {
			attr.period = fps_rate;
		}
	} else {
		pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
		pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
		//int fps_rate = (1000000 / pipe_contex->cap_info_->fps);
		//for (auto& attr : pipe_contex->sensor_config.vin_node_attr->lpwm_attr.lpwm_chn_attr) {
		//	attr.period = fps_rate;
		//}
	}
	// 创建pipeline中的每个node
	ret = creat_camera_node(pipe_contex->sensor_config.camera_config, &pipe_contex->cam_fd);
	ERR_CON_EQ(ret, 0);
	ret = creat_vin_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	ret = creat_isp_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	ret = creat_vse_node(pipe_contex);
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
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->vse_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->vin_node_handle,
							1,
							pipe_contex->isp_node_handle,
							0);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							0,
							pipe_contex->vse_node_handle,
							0);
	ERR_CON_EQ(ret, 0);

	ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd,
							pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);

	ret = hbn_camera_change_fps(pipe_contex->cam_fd, pipe_contex->sensor_config.camera_config->fps);
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


}  // namespace mipi_cam
