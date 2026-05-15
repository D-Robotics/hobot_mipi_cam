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
#include "sensor_msgs/distortion_models.hpp"
#include "opencv2/opencv.hpp"

#include "hobot_mipi_calibration.hpp"

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

#define ALIGN_16(x) (((x) + 15) & ~15)

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

  return 0;
}

int HobotMipiCapIml::init(MIPI_CAP_INFO_ST &info) {
  cap_info_ = info;
  if (cap_info_.link_type_ == 1) {
	return gsml_init(info);
  } else {
	return mipi_init(info);
  }return 0;
}

int HobotMipiCapIml::mipi_init(MIPI_CAP_INFO_ST &info) {
  int ret = 0;
  cap_info_ = info;
  cap_info_.sub_stream_enable_ = false;
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
	vin_online_isp = 0;
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
	auto contex_tmp = std::make_shared<pipe_contex_t>();
	pipe_contex.push_back(contex_tmp);
	contex_tmp = std::make_shared<pipe_contex_t>();
	pipe_contex.push_back(contex_tmp);
	pipe_contex[0]->cap_info_ = &cap_info_;
	pipe_contex[1]->cap_info_ = &cap_info_;
	copy_config(&pipe_contex[0]->sensor_config, vp_sensor_config_list[v_host_info[0].sensor_index]);
	//memcpy(&pipe_contex[0]->sensor_config, vp_sensor_config_list[v_host_info[0].sensor_index], sizeof(vp_sensor_config_t));
	ret = vp_sensor_fixed_mipi_host_1(v_host_info[0].host_num, &pipe_contex[0]->sensor_config, &pipe_contex[0]->csi_config);
	ERR_CON_EQ(ret, 0);
	gdc_bin_buf_.clear();
	mipi_calibration &calibration_instance = mipi_calibration::GetInstance();
	if (cap_info_.gdc_enable_) {
		vp_sensor_config_t *sensor_cfg = &pipe_contex[0]->sensor_config;
		if (cam_info_.size() != 2) {
			auto cal_params = mipi_calibration::GetInstance().getCalibrationParams();
			if (cal_params.size() >= 1)
			{
				cam_info_ = cal_params[0].cam_info_;
				cap_info_.cal_rotation_ = cal_params[0].cal_rotation_;
				awb_otp_data_ = cal_params[0].awb_otp_data_;
			}
		}
			auto gdc_bin = gen_gdc_bin_stereo(sensor_cfg->isp_cfg->isp_attr.size.width, sensor_cfg->isp_cfg->isp_attr.size.height,
					cap_info_.width, cap_info_.height, cap_info_.width, cap_info_.height, cam_info_, cal_cam_info_, cap_info_.rotation_, cap_info_.cal_rotation_, cap_info_.cal_alpha_);

			if (gdc_bin.size() == 2) {
				gdc_bin_buf_.push_back(gdc_bin[0]);
				gdc_bin_buf_.push_back(gdc_bin[1]);
				pipe_contex[0]->gdc_bin = gdc_bin[0];
				pipe_contex[1]->gdc_bin = gdc_bin[1];
			}			
	}
	if ((cap_info_.rotation_ != 0) && (gdc_bin_buf_.size() == 0)) {
		vp_sensor_config_t *sensor_conf = &pipe_contex[0]->sensor_config;
		int width;
		int height;
		if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
			width = cap_info_.height;
			height = cap_info_.width;
		} else {
			width = cap_info_.width;
			height = cap_info_.height;
		}

		auto gdc_bin = gen_gdc_bin_rotation(width, height, cap_info_.width, cap_info_.height, cap_info_.rotation_);
		if (gdc_bin) {
			gdc_bin_buf_.push_back(gdc_bin);
			pipe_contex[0]->gdc_bin_r = gdc_bin;
			pipe_contex[1]->gdc_bin_r = gdc_bin;
		}
	}

	pipeline_connect_param_init(pipe_contex[0]);

	ret = create_and_run_vflow(pipe_contex[0]);
	ERR_CON_EQ(ret, 0);
	copy_config(&pipe_contex[1]->sensor_config, vp_sensor_config_list[v_host_info[1].sensor_index]);
	//memcpy(&pipe_contex[1]->sensor_config, vp_sensor_config_list[v_host_info[1].sensor_index], sizeof(vp_sensor_config_t));
	ret = vp_sensor_fixed_mipi_host_1(v_host_info[1].host_num, &pipe_contex[1]->sensor_config, &pipe_contex[1]->csi_config);
	ERR_CON_EQ(ret, 0);
	pipeline_connect_param_init(pipe_contex[1]);
	ret = create_and_run_vflow(pipe_contex[1]);
	ERR_CON_EQ(ret, 0);
	if ((cap_info_.dual_combine_ == 1) || (cap_info_.dual_combine_ == 2)) {
		combine_flag_ = true;
	}
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

	auto contex_tmp = std::make_shared<pipe_contex_t>();
	pipe_contex.push_back(contex_tmp);
	pipe_contex[0]->cap_info_ = &cap_info_;
	memcpy(&pipe_contex[0]->sensor_config, vp_sensor_config_list[v_host_info[0].sensor_index], sizeof(vp_sensor_config_t));
	ret = vp_sensor_fixed_mipi_host_1(v_host_info[0].host_num, &pipe_contex[0]->sensor_config, &pipe_contex[0]->csi_config);
	ERR_CON_EQ(ret, 0);

    gdc_bin_buf_.clear();
	if (cap_info_.gdc_enable_) {
		vp_sensor_config_t *sensor_cfg = &pipe_contex[0]->sensor_config;
		if (cam_info_.size() > 0) {
			sensor_msgs::msg::CameraInfo cal_cam_info;
			if (cal_tpye_ == 0) {
				auto gdc_bin = gen_gdc_bin(sensor_cfg->isp_cfg->isp_attr.size.width, sensor_cfg->isp_cfg->isp_attr.size.height,
						cap_info_.width, cap_info_.height, &cam_info_[0], &cal_cam_info, cap_info_.rotation_, cap_info_.cal_rotation_);
				//auto gdc_bin = gen_gdc_bin_json("./gdc_bin_custom_config.json");
				if (gdc_bin) {
					gdc_bin_buf_.push_back(gdc_bin);
					pipe_contex[0]->gdc_bin = gdc_bin;
					cal_cam_info_.push_back(cal_cam_info);
				}
			}
		}
	}
	if ((cap_info_.rotation_ != 0) && (gdc_bin_buf_.size() == 0)) {
		int width;
		int height;
		if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
			width = cap_info_.height;
			height = cap_info_.width;
		} else {
			width = cap_info_.width;
			height = cap_info_.height;
		}
		auto gdc_bin = gen_gdc_bin_rotation(width, height, cap_info_.width, cap_info_.height, cap_info_.rotation_);
		if (gdc_bin) {
			gdc_bin_buf_.push_back(gdc_bin);
			pipe_contex[0]->gdc_bin_r = gdc_bin;
		}
	}
	pipeline_connect_param_init(pipe_contex[0]);
	ret = create_and_run_vflow(pipe_contex[0]);
	ERR_CON_EQ(ret, 0);
  }

  cap_info_.sensor_type = pipe_contex[0]->sensor_config.sensor_name;

  m_inited_ = true;

  return ret;
}

int HobotMipiCapIml::gsml_init(MIPI_CAP_INFO_ST &info) {
	int ret = 0;
	cap_info_ = info;
	cap_info_.sub_stream_enable_ = false;
	std::vector<int> sensor_v;
	std::vector<int> host_v;
	std::vector<mipi_host_info_t> v_host_info;
	std::vector<mipi_host_info_t> v_host_info_detect;
	int sensor_index = 0;
	bool sensor_flag = false;
	int sensor_index2 = 0;
	bool sensor_flag2 = false;
	mipi_host_info_t host_info;
	read_gsml_config(cap_info_.gsml_cfg_file_);
#if 0
	gsml_config_.resize(1);
	LINK_CONFIG_ST g_link;
	g_link.link_port = 0;
	g_link.sensor_type = "gsml_sc132gs";
	g_link.camera_mode = "dual";
	g_link.mipi_rx = 4;
	gsml_config_[0].link.push_back(g_link);
#endif
#if 0
	g_link.link_port = 1;
	g_link.sensor_type = "ov02b10-1300p25";
	g_link.camera_mode = "dual";
	gsml_config_[0].link.push_back(g_link);
#endif

#if 0
	g_link.link_port = 0;
	g_link.sensor_type = "ov02b10-1300p25";
	g_link.camera_mode = "dual";
	gsml_config_[0].link.push_back(g_link);

	g_link.link_port = 1;
	g_link.sensor_type = "ov02b10-1300p25";
	g_link.camera_mode = "dual";
	gsml_config_[0].link.push_back(g_link);
#endif
	int pipeline_num = 0, pipeline_count = 0;
	hb_mem_module_open();
	int text_flag = 0;
	if (!gsml_config_.empty()) {
		gdc_bin_buf_.clear();
		gdc_bin_buf_r_.clear();
		for (auto gsml_cfg : gsml_config_) {
			int des_num = vp_get_deserial_list_number();
			vp_deserial_config_t *deserial_cfg = nullptr;
			for (int i = 0; i < des_num; i++) {
				if (strcasecmp(vp_deserial_config_list[i]->sensor_name, gsml_cfg.deserial_name.c_str()) == 0) {
					deserial_cfg = vp_deserial_config_list[i];
					break;
				}
			}
			if (deserial_cfg == nullptr) {
				return -1;
			}
			auto des_contex = std::make_shared<DESERIAL_CONTEX_ST>();
			copy_deserial_config(&des_contex->deserial_attr, deserial_cfg->deserial_attr);
			deserial_contex.push_back(des_contex);

			for (auto link : gsml_cfg.link) {
				int num = 0;
				num = vp_get_gmsl_list_number();
				vp_sensor_config_t *sensor_cfg = nullptr;
				for (int i = 0; i < num; i++) {
					printf("index: %d  sensor_name: %-16s \tconfig_file:%s\n", i, vp_gmsl_config_list[i]->sensor_name, vp_gmsl_config_list[i]->config_file);
					if (strcasecmp(vp_gmsl_config_list[i]->sensor_name, link.sensor_type.c_str()) == 0) {
						sensor_cfg = vp_gmsl_config_list[i];
						break;
					}
				}
				if (sensor_cfg == nullptr) {
					return -1;
				}
				if (link.camera_mode == "dual") {
					vp_deserial_config_update(&des_contex->deserial_attr, sensor_cfg->camera_config, link.link_port);
					if (link.valid_port2) {
						vp_deserial_config_update(&des_contex->deserial_attr, sensor_cfg->camera_config, link.link_port2);
					}
				} else {
					vp_deserial_config_update(&des_contex->deserial_attr, sensor_cfg->camera_config, link.link_port);
				}
			}
			deserial_handle_t des_fd = 0;
			ret = create_deserial_node(&des_contex->deserial_attr, des_fd);
			ERR_CON_EQ(ret, 0);


			for (auto link : gsml_cfg.link) {
				int num = 0;
				num = vp_get_gmsl_list_number();
				vp_sensor_config_t *sensor_cfg = nullptr;
				for (int i = 0; i < num; i++) {
					printf("index: %d  sensor_name: %-16s \tconfig_file:%s\n", i, vp_gmsl_config_list[i]->sensor_name, vp_gmsl_config_list[i]->config_file);
					if (strcasecmp(vp_gmsl_config_list[i]->sensor_name, link.sensor_type.c_str()) == 0) {
						sensor_cfg = vp_gmsl_config_list[i];
						break;
					}
				}
				if (sensor_cfg == nullptr) {
					return -1;
				}
				if (link.camera_mode == "dual") {
					auto pipe_contex_tmp = std::make_shared<pipe_contex_t>();
					pipe_contex_tmp->cap_info_ = &cap_info_;
					copy_config(&pipe_contex_tmp->sensor_config, sensor_cfg);
					pipe_contex_tmp->des_fd = des_fd;
					pipe_contex_tmp->sensor_config.vin_attr->vin_node_attr.cim_attr.mipi_rx = link.mipi_rx;
					pipe_contex_tmp->sensor_config.vin_attr->vin_node_attr.cim_attr.vc_index = pipeline_num % 4;
					if (link.valid_phy && pipe_contex_tmp->sensor_config.camera_config->mipi_cfg) {
						pipe_contex_tmp->sensor_config.camera_config->mipi_cfg->rx_attr.phy = link.phy;
					}
					pipe_contex_tmp->sensor_config.camera_config->addr += pipeline_num;
					pipe_contex_tmp->sensor_config.camera_config->eeprom_addr += pipeline_num;
					pipe_contex_tmp->sensor_config.camera_config->serial_addr += pipeline_num;

					pipe_contex_tmp->gsml_link_port_ = link.link_port;
					pipe_contex_tmp->camera_bind_ = true;
					pipeline_connect_param_init(pipe_contex_tmp);
					ret = create_and_run_vflow_step1(pipe_contex_tmp);
					ERR_CON_EQ(ret, 0);
					#if 1
					create_gsml_gdc_bin(pipe_contex_tmp);
					#endif
					ret = create_and_run_vflow_step2(pipe_contex_tmp);
					ERR_CON_EQ(ret, 0);
					pipeline_num++;
					
					auto pipe_contex_tmp_2 = std::make_shared<pipe_contex_t>();
					pipe_contex_tmp_2->cap_info_ = &cap_info_;
					copy_config(&pipe_contex_tmp_2->sensor_config, sensor_cfg);
					pipe_contex_tmp_2->des_fd = des_fd;
					//pipe_contex_tmp_2->sensor_config.camera_config = pipe_contex_tmp_2->sensor_config.camera_slave_config;
					pipe_contex_tmp_2->sensor_config.vin_attr->vin_node_attr.cim_attr.mipi_rx = link.mipi_rx2;
					pipe_contex_tmp_2->sensor_config.vin_attr->vin_node_attr.cim_attr.vc_index = pipeline_num % 4;
					if (link.valid_phy2 && pipe_contex_tmp_2->sensor_config.camera_config->mipi_cfg) {
						pipe_contex_tmp_2->sensor_config.camera_config->mipi_cfg->rx_attr.phy = link.phy2;
					}
					pipe_contex_tmp_2->gsml_link_port_ = -1;
					pipe_contex_tmp_2->camera_bind_ = false;
					ret = create_and_run_vflow_step1(pipe_contex_tmp_2);
					ERR_CON_EQ(ret, 0);

					if (pipe_contex_tmp->gdc_bin) {
						pipe_contex_tmp_2->gdc_bin = pipe_contex_tmp->gdc_bin;
					} else if (pipe_contex_tmp->gdc_bin_r) {
						pipe_contex_tmp_2->gdc_bin_r = pipe_contex_tmp->gdc_bin_r;
					}
					ret = create_and_run_vflow_step2(pipe_contex_tmp_2);
					ERR_CON_EQ(ret, 0);
					if (link.dual_seq == 1) {
						pipe_contex.push_back(pipe_contex_tmp_2);
						des_contex->pipe.push_back(pipe_contex_tmp_2);
						pipe_contex.push_back(pipe_contex_tmp);
						des_contex->pipe.push_back(pipe_contex_tmp);
					} else {
						pipe_contex.push_back(pipe_contex_tmp);
						des_contex->pipe.push_back(pipe_contex_tmp);
						pipe_contex.push_back(pipe_contex_tmp_2);
						des_contex->pipe.push_back(pipe_contex_tmp_2);												
					}
					pipeline_num++;
				} else {
					auto pipe_contex_tmp = std::make_shared<pipe_contex_t>();
					pipe_contex_tmp->cap_info_ = &cap_info_;
					copy_config(&pipe_contex_tmp->sensor_config, sensor_cfg);
					pipe_contex_tmp->sensor_config.vin_attr->vin_node_attr.cim_attr.mipi_rx = link.mipi_rx;
					pipe_contex_tmp->sensor_config.vin_attr->vin_node_attr.cim_attr.vc_index = pipeline_num % 4;
					if (link.valid_phy && pipe_contex_tmp->sensor_config.camera_config->mipi_cfg) {
						pipe_contex_tmp->sensor_config.camera_config->mipi_cfg->rx_attr.phy = link.phy;
					}
					pipe_contex_tmp->sensor_config.camera_config->addr += pipeline_num;
					pipe_contex_tmp->sensor_config.camera_config->eeprom_addr += pipeline_num;
					pipe_contex_tmp->sensor_config.camera_config->serial_addr += pipeline_num;

					pipe_contex_tmp->gsml_link_port_ = pipeline_num % 4;
					pipe_contex_tmp->camera_bind_ = true;
					pipeline_connect_param_init(pipe_contex_tmp);
					ret = create_and_run_vflow_step1(pipe_contex_tmp);
					ERR_CON_EQ(ret, 0);
					#if 1
					create_gsml_gdc_bin(pipe_contex_tmp);
					#endif
					ret = create_and_run_vflow_step2(pipe_contex_tmp);
					ERR_CON_EQ(ret, 0);
					pipe_contex.push_back(pipe_contex_tmp);
					des_contex->pipe.push_back(pipe_contex_tmp);
					pipeline_num++;					
				}
			}
		}
		
		cap_info_.device_mode_ = "multi";
		combine_flag_ = true;

	} else {
		deserial_handle_t des_fd;
		auto contex_tmp = std::make_shared<pipe_contex_t>();
		pipe_contex.push_back(contex_tmp);
		pipe_contex[0]->cap_info_ = &cap_info_;

		int num = 0;
		num = vp_get_gmsl_list_number();
		vp_sensor_config_t *sensor_cfg = nullptr;
		for (int i = 0; i < num; i++) {
			printf("index: %d  sensor_name: %-16s \tconfig_file:%s\n", i, vp_gmsl_config_list[i]->sensor_name, vp_gmsl_config_list[i]->config_file);
			if (strcasecmp(vp_gmsl_config_list[i]->sensor_name, cap_info_.sensor_type.c_str()) == 0) {
				sensor_cfg = vp_gmsl_config_list[i];
				break;
			}
		}
		if (sensor_cfg == nullptr) {
			return -1;
		}

		pipe_contex[0]->camera_bind_ = true;
		copy_config(&pipe_contex[0]->sensor_config, sensor_cfg);

		ret = create_deserial_node(pipe_contex[0]->sensor_config.deserial_attr, des_fd);
		ERR_CON_EQ(ret, 0);

		pipe_contex[0]->des_fd = des_fd;
		pipe_contex[0]->gsml_link_port_ = cap_info_.link_port_;
		pipe_contex[0]->sensor_config.camera_config->addr += (uint8_t)(1 + cap_info_.link_port_);
		pipe_contex[0]->sensor_config.camera_config->serial_addr += (uint8_t)(1  + cap_info_.link_port_);
		pipe_contex[0]->sensor_config.camera_config->eeprom_addr += (uint8_t)(1  + cap_info_.link_port_);
		pipe_contex[0]->sensor_config.vin_attr->vin_node_attr.cim_attr.vc_index = cap_info_.link_port_;

		pipeline_connect_param_init(pipe_contex[0]);
		ret = create_and_run_vflow_step1(pipe_contex[0]);
		ERR_CON_EQ(ret, 0);
		create_gsml_gdc_bin(pipe_contex[0]);
		ret = create_and_run_vflow_step2(pipe_contex[0]);
		ERR_CON_EQ(ret, 0);
	}
	if (!pipe_contex.empty()) {
		cap_info_.sensor_type = pipe_contex[0]->sensor_config.sensor_name;
	} else {
		return -1;
	}
	m_inited_ = true;
  
	return ret;
  }
  


int HobotMipiCapIml::deInit() {
  int i = 0;
  if (m_inited_) {
	m_inited_ = false;
	
	for(auto contex : pipe_contex) {
		hbn_camera_destroy(contex->cam_fd);
		hbn_vflow_destroy(contex->vflow_fd);
	}
    for (auto deserial : deserial_contex) {
		hbn_deserial_destroy(deserial->des_fd);
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
    ret = hbn_vflow_start(contex->vflow_fd);
    ERR_CON_EQ(ret, 0);
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
	task_.emplace_back(std::make_shared<std::thread>(std::bind(&HobotMipiCapIml::multiFrameTask, this)));
	if (combine_flag_) {
		for(auto contex : pipe_contex) {
			v_frame_que_.push_back(std::make_shared<FrameQueue>());
		}
		task_.emplace_back(std::make_shared<std::thread>(std::bind(&HobotMipiCapIml::sync_task, this)));
	}
#if 0
	if (cap_info_.sub_stream_enable_ == true) {
		for(auto contex : pipe_contex) {
			auto que_manger = std::make_shared<BuffQueueManage>();
			que_manger->creat_buff(5);
			v_sub_buff_que_manger_.push_back(que_manger);
		}
		sub_combine_buff_que_manger_ = std::make_shared<BuffQueueManage>();
		sub_combine_buff_que_manger_->creat_buff(5);
		task_.emplace_back(std::make_shared<std::thread>(std::bind(&HobotMipiCapIml::subMultiFrameTask, this)));
		if (combine_flag_) {
			for(auto contex : pipe_contex) {
				v_sub_frame_que_.push_back(std::make_shared<FrameQueue>());
			}
			task_.emplace_back(std::make_shared<std::thread>(std::bind(&HobotMipiCapIml::sub_sync_task, this)));
		}
	}
#endif
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
  for (auto task : task_) {
    task->join();
  }
  task_.clear();
  started_ = false;
  for(auto contex : pipe_contex){
    ret = hbn_vflow_stop(contex->vflow_fd);
    ERR_CON_EQ(ret, 0);
    if(contex->sensor_config.sensor_type != SENSOR_TYPE_NORMAL) {
		if (contex->camera_bind_) {
			hbn_deserial_detach_from_vin(contex->des_fd, (camera_des_link_t)contex->gsml_link_port_);
			hbn_camera_detach_from_deserial(contex->cam_fd);
		} else {
			hbn_camera_detach_from_vin(contex->cam_fd);
		}
	}
	if (contex->gdc_node_handle != 0) {
		hbn_vnode_close(contex->gdc_node_handle);
	}
	if (contex->gdc_node_handle_r != 0) {
		hbn_vnode_close(contex->gdc_node_handle_r);
	}
	if (contex->pym_node_handle != 0) {
		hbn_vnode_close(contex->pym_node_handle);
	}
	if (contex->ynr_node_handle != 0) {
		hbn_vnode_close(contex->ynr_node_handle);
	}
	if (contex->isp_node_handle != 0) {
		hbn_vnode_close(contex->isp_node_handle);
	}
	if (contex->vin_node_handle != 0) {
		hbn_vnode_close(contex->vin_node_handle);
	}
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
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_vnode_getframe handle = %p, channel  = %d,ret = %d failed\n", handle, channel,ret);
		return -1;
	}
	hb_mem_invalidate_buf_with_vaddr((uint64_t)out_img.buffer.virt_addr[0],out_img.buffer.size[0]);

	hb_mem_invalidate_buf_with_vaddr((uint64_t)out_img.buffer.virt_addr[1],out_img.buffer.size[1]);

	//*timestamp = out_img.info.trig_tv.tv_sec * 1e9 + out_img.info.trig_tv.tv_usec * 1e3;
	//*timestamp = out_img.info.tv.tv_sec * 1e9 + out_img.info.tv.tv_usec * 1e3;
	struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
	double sys_timestamps = ts.tv_sec * 1e9 + ts.tv_nsec;
 
    int32_t exposure_time = (out_img.info.tv.tv_sec - out_img.info.trig_tv.tv_sec) * 1e9 + 
                          (out_img.info.tv.tv_usec - out_img.info.trig_tv.tv_usec) * 1e3;  

//   if (out_img.info.trig_tv.tv_sec != 0 && 
//       out_img.info.trig_tv.tv_usec != 0) {
//       out_img.info.sys_timestamps -= exposure_time;
//   }
  
  //  timestamps means kernel timestamp when the frame is obtained
  //  sys_timestamps means kernel system timestamp when the frame is obtained
  //  tv means hardware timestamp when the frame is obtained
  //  trig_tv means hardware timestamp when the frame is triggered by the external trigger
  double timestamps = out_img.info.timestamps * 1e-9;
  // double sys_timestamps = out_img.info.sys_timestamps * 1e-9;
  double hw_timestamp = out_img.info.tv.tv_sec + (double)out_img.info.tv.tv_usec * 1e-6;
  double tri_timestamp = out_img.info.trig_tv.tv_sec + (double)out_img.info.trig_tv.tv_usec * 1e-6;
  double current_ts =  ts.tv_sec + (double)ts.tv_nsec * 1e-9;
  
  buff_ptr->frame_id = out_img.info.frame_id;
  if ("realtime" == cap_info_.frame_ts_type_) {
	buff_ptr->timestamp = sys_timestamps;
  } else {
	buff_ptr->timestamp = out_img.info.timestamps;
  }                       
                          
  RCLCPP_DEBUG(rclcpp::get_logger("mipi_cap"),
            "capture a frame, handle: %llu, id: %d, timestamps: %f, sys_timestamps: %f, HW timestamp: %f, trig timestamp: %f,"
            "current timestamp: %f, laps ms: %fms, exposure_time: %fms.", 
			                        handle, buff_ptr->frame_id, timestamps, sys_timestamps, hw_timestamp, tri_timestamp,
                              current_ts, (current_ts - sys_timestamps) * 1e3, exposure_time * 1e-6);

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

int HobotMipiCapIml::getVnodeFrameGroup(hbn_vnode_handle_t handle, int channel, std::shared_ptr<VideoBuffer> buff_ptr) {
	
	if (buff_ptr == nullptr) {

		return -1;
	}
	// hbn_vnode_image_t out_img;
	// int ret = hbn_vnode_getframe(handle, channel, 1000, &out_img);

	hbn_vnode_image_group_t out_img;
	int ret = hbn_vnode_getframe_group(handle, channel, 1000, &out_img);

	if (ret != 0) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_vnode_getframe_group handle = %p, channel  = %d,ret = %d failed\n", handle, channel,ret);
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
		buff_ptr->timestamp = out_img.info.timestamps + (timestamp_1 - timestamp_2);
	} else {
		buff_ptr->timestamp = out_img.info.timestamps;
	} 
	//*timestamp = out_img.info.sys_timestamps;
	//*timestamp = out_img.info.timestamps;
	buff_ptr->frame_id = out_img.info.frame_id;

	RCLCPP_DEBUG(rclcpp::get_logger("mipi_cap"),
		"capture laps ms= %d", ((timestamp_2 - out_img.info.timestamps)/1000000));
	
	//std::cout << "getVnodeFrame--system time sec:" << tv.tv_sec << ", image time sec:" << out_img.info.tv.tv_sec
	//          << ", trig time sec:" << out_img.info.trig_tv.tv_sec 
	//		  << ", image timestamps(/1e9) sec:" << out_img.info.timestamps / 1e9 <<  std::endl;

	//std::cout << "getVnodeFrame--system time sec:" << tv.tv_sec << ", timestamp time sec:" << (int)(*timestamp / 1e9) <<  std::endl;
	
	buff_ptr->stride = out_img.buf_group.graph_group[0].stride;
	buff_ptr->width = out_img.buf_group.graph_group[0].width;
	buff_ptr->height = out_img.buf_group.graph_group[0].height;
	buff_ptr->buff_size = out_img.buf_group.graph_group[0].size[0] + out_img.buf_group.graph_group[0].size[1];
	buff_ptr->buff.resize(buff_ptr->buff_size);
	buff_ptr->encode = "nv12";

	memcpy(buff_ptr->buff.data(), out_img.buf_group.graph_group[0].virt_addr[0], out_img.buf_group.graph_group[0].size[0]);
	memcpy(buff_ptr->buff.data() + out_img.buf_group.graph_group[0].size[0], out_img.buf_group.graph_group[0].virt_addr[1], out_img.buf_group.graph_group[0].size[1]);
	hbn_vnode_releaseframe_group(handle, channel, &out_img);
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
		hbn_vnode_get_fd(pipe_contex[i]->stream_handle, 0, &ochn_fd[i]);
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
					if (pipe_contex[i]->stream_group) {
						ret = getVnodeFrameGroup(pipe_contex[i]->stream_handle, 0, buff_ptr);
					} else {
						ret = getVnodeFrame(pipe_contex[i]->stream_handle, 0, buff_ptr);
					}
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

int HobotMipiCapIml::getCapInfo(MIPI_CAP_INFO_ST &info) {
  info = cap_info_;
  return 0;
}

int HobotMipiCapIml::create_camera_node(std::shared_ptr<pipe_contex_t> pipe_contex, int link_port) {
	int32_t ret = 0;
#if ngy
	if(pipe_contex->sensor_config.sensor_type != SENSOR_TYPE_NORMAL){
		pipe_contex->sensor_config.camera_config->addr += (uint8_t)(1 + link_port);
		pipe_contex->sensor_config.camera_config->serial_addr += (uint8_t)(1  + link_port);
		pipe_contex->sensor_config.camera_config->eeprom_addr += (uint8_t)(1  + link_port);
	}
#endif
	ret = hbn_camera_create(pipe_contex->sensor_config.camera_config, &pipe_contex->cam_fd);
	ERR_CON_EQ(ret, 0);
	return 0;
}

int HobotMipiCapIml::create_deserial_node(std::shared_ptr<pipe_contex_t> pipe_contex) {
	int32_t ret = 0;
	vp_sensor_config_t& sensor_config = pipe_contex->sensor_config;
	ret = hbn_deserial_create(sensor_config.deserial_attr, &pipe_contex->des_fd);
	ERR_CON_EQ(ret, 0);
	RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),"deserial_config:,%02x,%s, des_fd:%ld \n\r" ,sensor_config.deserial_attr->addr, sensor_config.deserial_attr->name, pipe_contex->des_fd);
	return 0;
}

int HobotMipiCapIml::create_deserial_node(deserial_config_t *deserial_attr, deserial_handle_t &des_fd) {
	int32_t ret = 0;
	if (deserial_attr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"deserial_config is nullptr");
	}
	ret = hbn_deserial_create(deserial_attr, &des_fd);
	ERR_CON_EQ(ret, 0);
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"deserial_config:,%02x,%s, des_fd:%ld \n\r" ,deserial_attr->addr, deserial_attr->name, des_fd);
	return 0;
}

int HobotMipiCapIml::create_vin_node(std::shared_ptr<pipe_contex_t> pipe_contex, int is_online, int link_port) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	uint32_t hw_id = 0;
	int32_t ret = 0;
	uint32_t chn_id = 0;
	uint64_t vin_attr_ex_mask = 0;
	vin_attr_ex_t vin_attr_ex;
	vp_sensor_config_t& sensor_config = pipe_contex->sensor_config;

	hw_id = sensor_config.vin_attr->vin_node_attr.cim_attr.mipi_rx;

	if(is_online){
		sensor_config.vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].ddr_en = 0;
		sensor_config.vin_attr->vin_node_attr.cim_attr.cim_isp_flyby = 1;
	}else{
		sensor_config.vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].ddr_en = 1;
		sensor_config.vin_attr->vin_node_attr.cim_attr.cim_isp_flyby = 0;
	}
#if ngy
	if(pipe_contex->sensor_config.sensor_type != SENSOR_TYPE_NORMAL){
		sensor_config.vin_attr->vin_node_attr.cim_attr.vc_index = link_port;
		printf("vc_index:%d\n", sensor_config.vin_attr->vin_node_attr.cim_attr.vc_index);
	}
#endif

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
		memset(&alloc_attr_raw, 0, sizeof(hbn_buf_alloc_attr_t));
		alloc_attr_raw.buffers_num = 6;
		alloc_attr_raw.is_contig = 1;
		alloc_attr_raw.flags = HB_MEM_USAGE_CPU_READ_OFTEN
							| HB_MEM_USAGE_CPU_WRITE_OFTEN
							| HB_MEM_USAGE_CACHED;
		ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->vin_node_handle, chn_id, &alloc_attr_raw);
		ERR_CON_EQ(ret, 0);
	}

	return 0;
}


int HobotMipiCapIml::create_isp_node(std::shared_ptr<pipe_contex_t> pipe_contex, int hw_id, int slot_id, int mode, int is_online) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	hbn_buf_alloc_attr_t alloc_attr = {0};
	uint32_t chn_id = 0;
	int ret = 0;
	vp_sensor_config_t& sensor_config = pipe_contex->sensor_config;
	sensor_config.isp_cfg->isp_attr.channel.hw_id = hw_id;
	sensor_config.isp_cfg->isp_attr.channel.slot_id = slot_id;
	sensor_config.isp_cfg->isp_attr.sched_mode = (sched_mode_e)mode;

	if(is_online){
		sensor_config.isp_cfg->ochn_attr.stream_output_mode = STREAM_OUTPUT_MODE_ENABLE;
		sensor_config.isp_cfg->ochn_attr.axi_output_mode = AXI_OUTPUT_MODE_DISABLE;
	}else{
		sensor_config.isp_cfg->ochn_attr.stream_output_mode = STREAM_OUTPUT_MODE_DISABLE;
		sensor_config.isp_cfg->ochn_attr.axi_output_mode = AXI_OUTPUT_MODE_YUV420;
	}

	ret = hbn_vnode_open(HB_ISP, hw_id, AUTO_ALLOC_ID, &pipe_contex->isp_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_attr(pipe_contex->isp_node_handle, sensor_config.isp_cfg);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ochn_attr(pipe_contex->isp_node_handle, chn_id, &sensor_config.isp_cfg->ochn_attr);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ichn_attr(pipe_contex->isp_node_handle, chn_id, &sensor_config.isp_cfg->ichn_attr);
	ERR_CON_EQ(ret, 0);
	if (!is_online) {
		alloc_attr.buffers_num = 3;
		alloc_attr.is_contig = 1;
		alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
							| HB_MEM_USAGE_CPU_WRITE_OFTEN
							| HB_MEM_USAGE_CACHED;
		ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->isp_node_handle, chn_id, &alloc_attr);
		ERR_CON_EQ(ret, 0);
	}

	return 0;
}

int HobotMipiCapIml::create_ynr_node(std::shared_ptr<pipe_contex_t> pipe_contex, int slot_id, int work_mode) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	hbn_buf_alloc_attr_t alloc_attr = {0};
	int hw_id = 1; //固定为1
	uint32_t chn_id = 0;
	int ret = 0;
	vp_sensor_config_t& sensor_config = pipe_contex->sensor_config;
	sensor_config.ynr_attr->work_mode = work_mode;
	sensor_config.ynr_attr->slot_id = slot_id;

	ret = hbn_vnode_open(HB_YNR, hw_id, AUTO_ALLOC_ID, &pipe_contex->ynr_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_attr(pipe_contex->ynr_node_handle, sensor_config.ynr_attr);
	ERR_CON_EQ(ret, 0);

	struct hobot_ynr_channel_input_config channel_input_cfg = {0};
	ret = hbn_vnode_set_ichn_attr(pipe_contex->ynr_node_handle, 0, &channel_input_cfg);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ichn_attr(pipe_contex->ynr_node_handle, 1, &channel_input_cfg);
	ERR_CON_EQ(ret, 0);

	struct hobot_ynr_channel_output_config channel_output_cfg = {0};
	ret = hbn_vnode_set_ochn_attr(pipe_contex->ynr_node_handle, 0, &channel_output_cfg);
	ERR_CON_EQ(ret, 0);

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

const char* get_link_mode_string(int is_online){
	return (is_online ? "online" : "offline");
}


/**
	PIPELINE_SCENE_ISP_BYPASS：
		cim_0: online pym0
		cim_1: online pym1
		cim_4: offline PYM4
	PIPELINE_SCENE_ISP_ONLY:
		cim_0: online isp0 online pym0
		cim_1: offline isp0 online pym0
		cim_4: offline isp0 online pym0
	PIPELINE_SCENE_ISP_YNR
		cim_0: offline isp1 online ynr1 online pym1
		cim_1: online isp1 online ynr1 online pym1
		cim_4: offline isp1 online ynr1 online pym1
 */
void HobotMipiCapIml::pipeline_connect_param_init(std::shared_ptr<pipe_contex_t> pipe_contex){
	vp_sensor_config_t *sensor_config = &pipe_contex->sensor_config;
	if(sensor_config->sensor_type == SENSOR_TYPE_GMSL_YUV){
		pipe_contex->sensor_type_ = PIPELINE_SCENE_ISP_BYPASS;
	}else{
		if(sensor_config->ynr_attr == NULL){
			pipe_contex->sensor_type_ = PIPELINE_SCENE_ISP_ONLY;
		}else if((sensor_config->camera_config->width > 2048) ||
			(sensor_config->camera_config->height > 2048)){
				pipe_contex->sensor_type_ = PIPELINE_SCENE_ISP_ONLY;
		}else{
			pipe_contex->sensor_type_ = PIPELINE_SCENE_ISP_YNR;
		}
	}

	pipeline_channel_info_t* ch_info = &pipe_contex->pipe_info_;
	// 情景1(不需要ISP): 尽量online
	if(pipe_contex->sensor_type_ == PIPELINE_SCENE_ISP_BYPASS){
		ch_info->pym_hw_id = sensor_config->vin_attr->vin_node_attr.cim_attr.mipi_rx;
		if(sensor_config->vin_attr->vin_node_attr.cim_attr.mipi_rx == 4){
			ch_info->pym_mode = PYM_M2M_MODE; //offline
			ch_info->pym_slot_id = 0; 		  //offline的情况可以不用设置
			ch_info->is_online_vin_pym = 0;
		}else{
			ch_info->pym_mode = PYM_MANUAL_MODE; //online
			ch_info->pym_slot_id = 0;
			if (sensor_config->pym_cfg == nullptr) {
				ch_info->is_online_vin_pym = 0;
			} else {
				ch_info->is_online_vin_pym = 1;
			}	
		}

		//printf("	[%d] [not use isp].\n", pipeline_index);
		printf("		vin [hw:%d]\n", sensor_config->vin_attr->vin_node_attr.cim_attr.mipi_rx);
		printf("		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->pym_slot_id, ch_info->pym_hw_id, ch_info->pym_mode);
		printf("		vin ->%s-> pym \n",
			get_link_mode_string(ch_info->is_online_vin_pym));
	// 情景2(需要ISP， 不需要YNR): 固定使用ISP0
	}else if(pipe_contex->sensor_type_ == PIPELINE_SCENE_ISP_ONLY){
		ch_info->isp_mode = SCHED_MODE_MANUAL;
		ch_info->isp_hw_id = 0;
		ch_info->isp_slot_id = isp0_next_slot_id++;

		ch_info->pym_slot_id = ch_info->isp_slot_id;
		ch_info->pym_hw_id = 0;  		  			//固定设置为0
		ch_info->pym_mode = PYM_MANUAL_MODE; 		//offline

			ch_info->is_online_vin_isp = 0;
		if (sensor_config->pym_cfg == nullptr) {
			ch_info->is_online_isp_pym = 0;
		} else {
		    ch_info->is_online_isp_pym = 1;
		}

		//printf("	[%d] only use [isp only].\n", pipeline_index);
		printf("		vin [hw:%d]\n", sensor_config->vin_attr->vin_node_attr.cim_attr.mipi_rx);
		printf("		isp [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode);
		printf("		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->pym_hw_id, ch_info->pym_slot_id, ch_info->pym_mode);
		printf("		vin ->%s-> isp ->%s-> pym \n",
			get_link_mode_string(ch_info->is_online_vin_isp),
			get_link_mode_string(ch_info->is_online_isp_pym));

	// 情景3(ISP + YNR): 固定使用ISP1
	}else if(pipe_contex->sensor_type_ == PIPELINE_SCENE_ISP_YNR){
		ch_info->isp_mode = SCHED_MODE_MANUAL;
		ch_info->isp_hw_id = 1;
		ch_info->isp_slot_id = isp0_next_slot_id++;

		ch_info->ynr_mode = 1; //1:Manaul 模式	2:全online模式
		ch_info->ynr_slot_id = ch_info->isp_slot_id;

		ch_info->pym_slot_id = ch_info->isp_slot_id;
		ch_info->pym_hw_id = 1;
		ch_info->pym_mode = PYM_MANUAL_MODE;
		ch_info->is_online_vin_isp = 0;
		ch_info->is_online_isp_ynr = 1;
		ch_info->is_online_ynr_pym = 1;

		//printf("	[%d] use [isp + ynr].\n", pipeline_index);
		printf("		vin [hw:%d]\n", sensor_config->vin_attr->vin_node_attr.cim_attr.mipi_rx);
		printf("		isp [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode);
		printf("		ynr [hw:%d] [slot_id:%d] [mode:%d]\n",
			1, ch_info->ynr_slot_id, ch_info->ynr_mode);
		printf("		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->pym_hw_id, ch_info->pym_slot_id, ch_info->pym_mode);

		printf("		vin ->%s-> isp ->%s-> ynr ->%s-> pym\n",
			get_link_mode_string(ch_info->is_online_vin_isp),
			get_link_mode_string(ch_info->is_online_isp_ynr),
			get_link_mode_string(ch_info->is_online_ynr_pym));
	}else{
		//error
	}
}


static int check_pym_config(int src_width, int src_height, int width, int height, 
					int &bl_width, int &bl_height,int &bl_stride, int &roi_sel, int &roi_layer) {
	if ((src_width & 1) || (src_height & 1) || (width & 1) || (height & 1)) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"), "width and height isn't charmonium, width:%d, height:%d", width, height);
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
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"), "width and height over rang, width:%d, height:%d", width, height);
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
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"), "width and height over rang, width:%d, height:%d", width, height);
		return -1;
	}
	return 0;
}


int HobotMipiCapIml::create_pym_node(std::shared_ptr<pipe_contex_t> pipe_contex, int hw_id, int slot_id, int pym_mode) {
	if (pipe_contex == nullptr) {
		return -1;
	}

	// auto vi_hw_id = pipe_contex->sensor_config.vin_attr->vin_node_attr.cim_attr.mipi_rx;
	// if(vi_hw_id == 4){
	// 	pipe_contex->sensor_config.pym_cfg->slot_id = pipe_contex->sensor_config.isp_cfg->isp_attr.channel.slot_id;
	// 	pipe_contex->sensor_config.pym_cfg->hw_id = 1;
	// 	pipe_contex->sensor_config.pym_cfg->pym_mode = 1;
	// }
	pipe_contex->sensor_config.pym_cfg->slot_id = slot_id;
	pipe_contex->sensor_config.pym_cfg->hw_id = hw_id;
	pipe_contex->sensor_config.pym_cfg->pym_mode = pym_mode;
	int src_width = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_width;
	int src_height = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_height;
    int out_width;
	int out_height;
	if (pipe_contex->gdc_init_valid == 1) {
		out_width = src_width;
		out_height = src_height;
	} else {
		if ((pipe_contex->cap_info_->rotation_ == 90.0) || (pipe_contex->cap_info_->rotation_ == 270.0)) {
			out_width = pipe_contex->cap_info_->height;
			out_height = pipe_contex->cap_info_->width;
		} else {
			out_width = pipe_contex->cap_info_->width;
			out_height = pipe_contex->cap_info_->height;
		}
	}

	int roi_sel = 0;
	int roi_layer = 0;
	int bl_width = src_width;
	int bl_height = src_height;
	int bl_stride = src_width;
	if (check_pym_config(src_width, src_height, out_width,
			out_height, bl_width, bl_height, bl_stride, roi_sel, roi_layer) == -1) {
		return -1;
	}
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"), "creat_pym_node--roi_sel: %d, roi_layer: %d, bl_width: %d, bl_height: %d", roi_sel, roi_layer, bl_width, bl_height);
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_sel[0] = roi_sel;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_layer[0] = roi_layer;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].region_width = bl_width;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].region_height = bl_height;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].wstride_uv = out_width;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].wstride_y = out_width;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].out_width = out_width;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].out_height = out_height;
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[0].vstride = out_height;

	int ret = 0;
	uint32_t chn_id = 0;
	// uint32_t hw_id = pipe_contex->sensor_config.pym_cfg->hw_id;
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

int HobotMipiCapIml::create_gdc_node_r(std::shared_ptr<pipe_contex_t> pipe_contex) {
	if ((pipe_contex == nullptr) || (pipe_contex->gdc_bin_r == nullptr)) {
		return -1;
	}
	int ret = 0;
	uint32_t chn_id = 0;
	isp_cfg_t isp_attr;
	pipe_contex->gdc_init_valid_r = 0;
	int input_width, input_height, out_width, out_height;

	if ((pipe_contex->cap_info_->rotation_ == 90.0) || (pipe_contex->cap_info_->rotation_ == 270.0)) {
		out_height = input_width = pipe_contex->cap_info_->height;
		out_width = input_height = pipe_contex->cap_info_->width;
	} else {
		out_width = input_width = pipe_contex->cap_info_->width;
		out_height = input_height = pipe_contex->cap_info_->height;
	}

	gdc_settings_t gdc_setting = {0};
	uint32_t hw_id = 0;
	ret = hbn_vnode_open(HB_GDC, hw_id, AUTO_ALLOC_ID, &pipe_contex->gdc_node_handle_r);
	ERR_CON_EQ(ret, 0);


	gdc_setting.gdc_config.config_addr = pipe_contex->gdc_bin_r->bin_buf->phys_addr;
	gdc_setting.gdc_config.config_size = pipe_contex->gdc_bin_r->bin_buf->size;
	gdc_setting.gdc_config.input_width = input_width;
	gdc_setting.gdc_config.input_height = input_height;
	gdc_setting.gdc_config.input_stride = ALIGN_16(input_width);//16字节对齐
	gdc_setting.gdc_config.output_width = out_width;
	gdc_setting.gdc_config.output_height =out_height;
	gdc_setting.gdc_config.output_stride = ALIGN_16(out_width);//16字节对齐
	
	gdc_setting.gdc_config.div_width = 0;
	gdc_setting.gdc_config.div_height = 0;
	gdc_setting.gdc_config.total_planes = 2;
	gdc_setting.binary_ion_id = pipe_contex->gdc_bin_r->bin_buf->share_id;
	gdc_setting.binary_offset = pipe_contex->gdc_bin_r->bin_buf->offset;
	gdc_setting.magicNumber = MAGIC_NUMBER;

	ret = hbn_vnode_set_attr(pipe_contex->gdc_node_handle_r, &gdc_setting);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ichn_attr(pipe_contex->gdc_node_handle_r, chn_id, &gdc_setting);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ochn_attr(pipe_contex->gdc_node_handle_r, chn_id, &gdc_setting);
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

int HobotMipiCapIml::create_gdc_node(std::shared_ptr<pipe_contex_t> pipe_contex) {
	if ((pipe_contex == nullptr) || (pipe_contex->gdc_bin == nullptr)) {
		return -1;
	}
	int ret = 0;
	uint32_t chn_id = 0;
	pipe_contex->gdc_init_valid = 0;

	auto input_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
	auto input_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
	auto out_width = pipe_contex->cap_info_->width;
	auto out_height = pipe_contex->cap_info_->height;

    gdc_settings_t gdc_setting = {0};
	uint32_t hw_id = 0;
	ret = hbn_vnode_open(HB_GDC, hw_id, AUTO_ALLOC_ID, &pipe_contex->gdc_node_handle);
	ERR_CON_EQ(ret, 0);
	

	gdc_setting.gdc_config.config_addr = pipe_contex->gdc_bin->bin_buf->phys_addr;
	gdc_setting.gdc_config.config_size = pipe_contex->gdc_bin->bin_buf->size;
	gdc_setting.gdc_config.input_width = input_width;
	gdc_setting.gdc_config.input_height = input_height;
	gdc_setting.gdc_config.input_stride = ALIGN_16(input_width);//16字节对齐
	gdc_setting.gdc_config.output_width = out_width;
	gdc_setting.gdc_config.output_height =out_height;
	gdc_setting.gdc_config.output_stride = ALIGN_16(out_width);//16字节对齐
	gdc_setting.gdc_config.div_width = 0;
	gdc_setting.gdc_config.div_height = 0;
	gdc_setting.gdc_config.total_planes = 2;
	gdc_setting.binary_ion_id = pipe_contex->gdc_bin->bin_buf->share_id;
	gdc_setting.binary_offset = pipe_contex->gdc_bin->bin_buf->offset;
	gdc_setting.magicNumber = MAGIC_NUMBER;
	ret = hbn_vnode_set_attr(pipe_contex->gdc_node_handle, &gdc_setting);
	ERR_CON_EQ(ret, 0);
    ret = hbn_vnode_set_ichn_attr(pipe_contex->gdc_node_handle, chn_id, &gdc_setting);
	ERR_CON_EQ(ret, 0);
    ret = hbn_vnode_set_ochn_attr(pipe_contex->gdc_node_handle, chn_id, &gdc_setting);

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

int HobotMipiCapIml::create_and_run_vflow(std::shared_ptr<pipe_contex_t> pipe_contex) {
		if (pipe_contex == nullptr) {
		return -1;
	}
	pipe_contex->stream_group = 0;
	int32_t ret = 0;

    if (pipe_contex->sensor_config.sensor_type == SENSOR_TYPE_NORMAL) {
		//pipe_contex->sensor_config.isp_attr->input_mode = 2;
		if (pipe_contex->cap_info_->lpwm_enable_) {
			pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
			pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
			int fps_rate = (1000000 / pipe_contex->cap_info_->fps);
			pipe_contex->sensor_config.camera_config->sensor_mode = 6;
			//pipe_contex->sensor_config.vin_attr->vin_node_attr.lpwm_attr.enable = 1;
			for (auto& attr : pipe_contex->sensor_config.vin_attr->vin_node_attr.lpwm_attr.lpwm_chn_attr) {
				attr.period = fps_rate;
				attr.enable = 1;
			}
		} else {
			pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
			pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
			pipe_contex->sensor_config.camera_config->sensor_mode = 1;
			for (auto& attr : pipe_contex->sensor_config.vin_attr->vin_node_attr.lpwm_attr.lpwm_chn_attr) {
				attr.enable = 0;
			}
		}
	} else {
		//	pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
		//	pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
	}
	// 创建pipeline中的每个node
	ret = create_camera_node(pipe_contex, pipe_contex->gsml_link_port_);
	ERR_CON_EQ(ret, 0);

	int scene_type = pipe_contex->sensor_type_;
	pipeline_channel_info_t *ch_info = &pipe_contex->pipe_info_;

	if(scene_type == PIPELINE_SCENE_ISP_ONLY){
		ret = create_vin_node(pipe_contex, ch_info->is_online_vin_isp, pipe_contex->gsml_link_port_);
		ERR_CON_EQ(ret, 0);

		ret = create_isp_node(pipe_contex, ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode, ch_info->is_online_isp_pym);
		ERR_CON_EQ(ret, 0);
	}else if(scene_type == PIPELINE_SCENE_ISP_YNR){
		ret = create_vin_node(pipe_contex, ch_info->is_online_vin_isp, pipe_contex->gsml_link_port_);
		ERR_CON_EQ(ret, 0);

		ret = create_isp_node(pipe_contex, ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode, ch_info->is_online_isp_ynr);
		ERR_CON_EQ(ret, 0);

		ret = create_ynr_node(pipe_contex, ch_info->ynr_slot_id, ch_info->ynr_mode); //1: Manaul 模式  2: 全 online模式
		ERR_CON_EQ(ret, 0);
	}else{
		ret = create_vin_node(pipe_contex, ch_info->is_online_vin_pym, pipe_contex->gsml_link_port_);
		ERR_CON_EQ(ret, 0);
	}

	if (pipe_contex->sensor_config.pym_cfg) {
		if (cap_info_.gdc_enable_) {
			create_gdc_node(pipe_contex);
		}
		create_gdc_node_r(pipe_contex);
		ret = create_pym_node(pipe_contex, ch_info->pym_hw_id, ch_info->pym_slot_id, ch_info->pym_mode);
		ERR_CON_EQ(ret, 0);
	}
	// 2. 添加node 到 flow
	ret = hbn_vflow_create(&pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);

	if(scene_type == PIPELINE_SCENE_ISP_ONLY){
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle);
		ERR_CON_EQ(ret, 0);
	}else if(scene_type == PIPELINE_SCENE_ISP_YNR){
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle);
		ERR_CON_EQ(ret, 0);
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
								pipe_contex->ynr_node_handle);
		ERR_CON_EQ(ret, 0);
	}else{
		//do nothing
	}

	if (pipe_contex->sensor_config.pym_cfg) {
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
		// 3. 绑定 Flow 中的Node
		if(scene_type == PIPELINE_SCENE_ISP_BYPASS){
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
									pipe_contex->vin_node_handle,
									ch_info->is_online_vin_pym,
									pipe_contex->pym_node_handle,
									0);
			ERR_CON_EQ(ret, 0);

		}else if(scene_type == PIPELINE_SCENE_ISP_ONLY){
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
									pipe_contex->vin_node_handle,
									ch_info->is_online_vin_isp,
									pipe_contex->isp_node_handle,
									0);
			ERR_CON_EQ(ret, 0);

			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							ch_info->is_online_isp_pym,
							pipe_contex->pym_node_handle,
							0);
			ERR_CON_EQ(ret, 0);

		}else if(scene_type == PIPELINE_SCENE_ISP_YNR){
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
									pipe_contex->vin_node_handle,
									ch_info->is_online_vin_isp,
									pipe_contex->isp_node_handle,
									0);
			ERR_CON_EQ(ret, 0);

			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							ch_info->is_online_isp_ynr,
							pipe_contex->ynr_node_handle,
							0);
			ERR_CON_EQ(ret, 0);

			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->ynr_node_handle,
							ch_info->is_online_ynr_pym,
							pipe_contex->pym_node_handle,
							0);
			ERR_CON_EQ(ret, 0);

		}else{
			//error
		}
		pipe_contex->stream_handle = pipe_contex->pym_node_handle;
		pipe_contex->stream_group = 1;


		if (pipe_contex->gdc_init_valid_r == 1) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc rotation.\n");
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->pym_node_handle,
								0,
								pipe_contex->gdc_node_handle_r,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->gdc_node_handle_r;
			pipe_contex->stream_group = 0;
		} else if (pipe_contex->gdc_init_valid == 1) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc cal.\n");
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->pym_node_handle,
								0,
								pipe_contex->gdc_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->gdc_node_handle;
			pipe_contex->stream_group = 0;
		} else {
		
		}	
	} else {
		// 3. 绑定 Flow 中的Node
		if(scene_type == PIPELINE_SCENE_ISP_BYPASS){
			pipe_contex->stream_handle = pipe_contex->vin_node_handle;
			pipe_contex->stream_group = 0;

		}else if(scene_type == PIPELINE_SCENE_ISP_ONLY){
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
									pipe_contex->vin_node_handle,
									ch_info->is_online_vin_isp,
									pipe_contex->isp_node_handle,
									0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->isp_node_handle;
			pipe_contex->stream_group = 1;
		}else if(scene_type == PIPELINE_SCENE_ISP_YNR){
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
									pipe_contex->vin_node_handle,
									ch_info->is_online_vin_isp,
									pipe_contex->isp_node_handle,
									0);
			ERR_CON_EQ(ret, 0);

			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							ch_info->is_online_isp_ynr,
							pipe_contex->ynr_node_handle,
							0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->ynr_node_handle;
			pipe_contex->stream_group = 0;
		}else{
			//error
		}		
	}	

	if(pipe_contex->sensor_config.sensor_type != SENSOR_TYPE_NORMAL) {
		//ret = create_deserial_node(pipe_contex);
		//ERR_CON_EQ(ret, 0);
		if (pipe_contex->camera_bind_) {
			ret = hbn_camera_attach_to_deserial(pipe_contex->cam_fd, pipe_contex->des_fd, (camera_des_link_t)pipe_contex->gsml_link_port_);
			ERR_CON_EQ(ret, 0);
			ret = hbn_deserial_attach_to_vin(pipe_contex->des_fd, (camera_des_link_t)pipe_contex->gsml_link_port_, pipe_contex->vin_node_handle);
			ERR_CON_EQ(ret, 0);
		} else {
			//ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd, pipe_contex->vin_node_handle);
			//ERR_CON_EQ(ret, 0);
		}
	}else {
		ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd,
							pipe_contex->vin_node_handle);
		ERR_CON_EQ(ret, 0);
	}

	return 0;
}

int HobotMipiCapIml::create_and_run_vflow_step1(std::shared_ptr<pipe_contex_t> pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	pipe_contex->stream_group = 0;
	int32_t ret = 0;

    if (pipe_contex->sensor_config.sensor_type == SENSOR_TYPE_NORMAL) {
		//pipe_contex->sensor_config.isp_attr->input_mode = 2;
		if (pipe_contex->cap_info_->lpwm_enable_) {
			pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
			pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
			int fps_rate = (1000000 / pipe_contex->cap_info_->fps);
			pipe_contex->sensor_config.camera_config->sensor_mode = 6;
			//pipe_contex->sensor_config.vin_attr->vin_node_attr.lpwm_attr.enable = 1;
			for (auto& attr : pipe_contex->sensor_config.vin_attr->vin_node_attr.lpwm_attr.lpwm_chn_attr) {
				attr.period = fps_rate;
				attr.enable = 1;
			}
		} else {
			pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
			pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
			pipe_contex->sensor_config.camera_config->sensor_mode = 1;
			for (auto& attr : pipe_contex->sensor_config.vin_attr->vin_node_attr.lpwm_attr.lpwm_chn_attr) {
				attr.enable = 0;
			}
		}
	} else {
		//	pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
		//	pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
	}
	// 创建pipeline中的每个node
	ret = create_camera_node(pipe_contex, pipe_contex->gsml_link_port_);
	ERR_CON_EQ(ret, 0);

	int scene_type = pipe_contex->sensor_type_;
	pipeline_channel_info_t *ch_info = &pipe_contex->pipe_info_;

	if(scene_type == PIPELINE_SCENE_ISP_ONLY){
		ret = create_vin_node(pipe_contex, ch_info->is_online_vin_isp, pipe_contex->gsml_link_port_);
		ERR_CON_EQ(ret, 0);
	}else if(scene_type == PIPELINE_SCENE_ISP_YNR){
		ret = create_vin_node(pipe_contex, ch_info->is_online_vin_isp, pipe_contex->gsml_link_port_);
		ERR_CON_EQ(ret, 0);
	}else{
		ret = create_vin_node(pipe_contex, ch_info->is_online_vin_pym, pipe_contex->gsml_link_port_);
		ERR_CON_EQ(ret, 0);
	}

	// 2. 添加node 到 flow
	ret = hbn_vflow_create(&pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);

	if(pipe_contex->sensor_config.sensor_type != SENSOR_TYPE_NORMAL) {
		//ret = create_deserial_node(pipe_contex);
		//ERR_CON_EQ(ret, 0);
		if (pipe_contex->camera_bind_) {
			ret = hbn_camera_attach_to_deserial(pipe_contex->cam_fd, pipe_contex->des_fd, (camera_des_link_t)pipe_contex->gsml_link_port_);
			ERR_CON_EQ(ret, 0);
			ret = hbn_deserial_attach_to_vin(pipe_contex->des_fd, (camera_des_link_t)pipe_contex->gsml_link_port_, pipe_contex->vin_node_handle);
			ERR_CON_EQ(ret, 0);
		} else {
			//ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd, pipe_contex->vin_node_handle);
			//ERR_CON_EQ(ret, 0);
		}
	}else {
		ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd,
							pipe_contex->vin_node_handle);
		ERR_CON_EQ(ret, 0);
	}

	return 0;
}


int HobotMipiCapIml::create_and_run_vflow_step2(std::shared_ptr<pipe_contex_t> pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	pipe_contex->stream_group = 0;
	int32_t ret = 0;

	int scene_type = pipe_contex->sensor_type_;
	pipeline_channel_info_t *ch_info = &pipe_contex->pipe_info_;

	if(scene_type == PIPELINE_SCENE_ISP_ONLY){
		ret = create_isp_node(pipe_contex, ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode, ch_info->is_online_isp_pym);
		ERR_CON_EQ(ret, 0);
	}else if(scene_type == PIPELINE_SCENE_ISP_YNR){
		ret = create_isp_node(pipe_contex, ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode, ch_info->is_online_isp_ynr);
		ERR_CON_EQ(ret, 0);

		ret = create_ynr_node(pipe_contex, ch_info->ynr_slot_id, ch_info->ynr_mode); //1: Manaul 模式  2: 全 online模式
		ERR_CON_EQ(ret, 0);
	}else{
		//do nothing
	}

	if (pipe_contex->sensor_config.pym_cfg) {
		if (cap_info_.gdc_enable_) {
			create_gdc_node(pipe_contex);
		}
		create_gdc_node_r(pipe_contex);
		ret = create_pym_node(pipe_contex, ch_info->pym_hw_id, ch_info->pym_slot_id, ch_info->pym_mode);
		ERR_CON_EQ(ret, 0);
	}
	if(scene_type == PIPELINE_SCENE_ISP_ONLY){
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle);
		ERR_CON_EQ(ret, 0);
	}else if(scene_type == PIPELINE_SCENE_ISP_YNR){
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle);
		ERR_CON_EQ(ret, 0);
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
								pipe_contex->ynr_node_handle);
		ERR_CON_EQ(ret, 0);
	}else{
		//do nothing
	}

	if (pipe_contex->sensor_config.pym_cfg) {
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
		// 3. 绑定 Flow 中的Node
		if(scene_type == PIPELINE_SCENE_ISP_BYPASS){
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
									pipe_contex->vin_node_handle,
									ch_info->is_online_vin_pym,
									pipe_contex->pym_node_handle,
									0);
			ERR_CON_EQ(ret, 0);

		}else if(scene_type == PIPELINE_SCENE_ISP_ONLY){
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
									pipe_contex->vin_node_handle,
									ch_info->is_online_vin_isp,
									pipe_contex->isp_node_handle,
									0);
			ERR_CON_EQ(ret, 0);

			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							ch_info->is_online_isp_pym,
							pipe_contex->pym_node_handle,
							0);
			ERR_CON_EQ(ret, 0);

		}else if(scene_type == PIPELINE_SCENE_ISP_YNR){
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
									pipe_contex->vin_node_handle,
									ch_info->is_online_vin_isp,
									pipe_contex->isp_node_handle,
									0);
			ERR_CON_EQ(ret, 0);

			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							ch_info->is_online_isp_ynr,
							pipe_contex->ynr_node_handle,
							0);
			ERR_CON_EQ(ret, 0);

			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->ynr_node_handle,
							ch_info->is_online_ynr_pym,
							pipe_contex->pym_node_handle,
							0);
			ERR_CON_EQ(ret, 0);

		}else{
			//error
		}
		pipe_contex->stream_handle = pipe_contex->pym_node_handle;
		pipe_contex->stream_group = 1;


		if (pipe_contex->gdc_init_valid_r == 1) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc rotation.\n");
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->pym_node_handle,
								0,
								pipe_contex->gdc_node_handle_r,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->gdc_node_handle_r;
			pipe_contex->stream_group = 0;
		} else if (pipe_contex->gdc_init_valid == 1) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc cal.\n");
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->pym_node_handle,
								0,
								pipe_contex->gdc_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->gdc_node_handle;
			pipe_contex->stream_group = 0;
		} else {
		
		}	
	} else {
		// 3. 绑定 Flow 中的Node
		if(scene_type == PIPELINE_SCENE_ISP_BYPASS){
			pipe_contex->stream_handle = pipe_contex->vin_node_handle;
			pipe_contex->stream_group = 0;

		}else if(scene_type == PIPELINE_SCENE_ISP_ONLY){
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
									pipe_contex->vin_node_handle,
									ch_info->is_online_vin_isp,
									pipe_contex->isp_node_handle,
									0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->isp_node_handle;
			pipe_contex->stream_group = 1;
		}else if(scene_type == PIPELINE_SCENE_ISP_YNR){
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
									pipe_contex->vin_node_handle,
									ch_info->is_online_vin_isp,
									pipe_contex->isp_node_handle,
									0);
			ERR_CON_EQ(ret, 0);

			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							ch_info->is_online_isp_ynr,
							pipe_contex->ynr_node_handle,
							0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->ynr_node_handle;
			pipe_contex->stream_group = 0;
		}else{
			//error
		}		
	}	

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

bool HobotMipiCapIml::read_gsml_config(std::string gsml_cfg_file) {
  std::ifstream gsml_config(gsml_cfg_file);
  if (!gsml_config.is_open()) {
    return false;
  }
  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string errs;

  if (!Json::parseFromStream(builder, gsml_config, &root, &errs)) {
	  std::cout << "解析失败: " << errs << std::endl;
	  return false;
  }
  try {
	// 获取 deserial 数组
	const Json::Value deserials = root["deserial"];

	for (unsigned int i = 0; i < deserials.size(); i++) {
		const Json::Value& des = deserials[i];
		GSML_CONFIG_ST gsml_config;
		gsml_config.deserial_name = des["name"].asString();
		// 获取 link 数组
		const Json::Value links = des["link"];
		for (unsigned int j = 0; j < links.size(); j++) {
			const Json::Value& link = links[j];
			LINK_CONFIG_ST link_config;
			link_config.sensor_type = link["sensor"].asString();
			link_config.camera_mode = link["camera_mode"].asString();
			link_config.dual_mode = link["dual_mode"].asInt();
			link_config.link_port = link["link_port"].asInt();
			link_config.mipi_rx = link["mipi_rx"].asInt();
			if (link.isMember("dual_seq")) {
				link_config.dual_seq = link["dual_seq"].asInt();
			} else {
				link_config.dual_seq = 0;
			}
			if (link.isMember("link_port2")) {
				link_config.link_port2 = link["link_port2"].asInt();
				link_config.valid_port2 = true;
			} else {
				link_config.valid_port2 = false;
			}
			if (link.isMember("mipi_rx2")) {
				link_config.mipi_rx2 = link["mipi_rx2"].asInt();
			} else {
				link_config.mipi_rx2 = 0;
			}
			if (link.isMember("phy")) {
			link_config.phy = link["phy"].asInt();
				link_config.valid_phy = true;
			} else {
				link_config.valid_phy = false;
			}
			if (link.isMember("phy2")) {
				link_config.phy2 = link["phy2"].asInt();
				link_config.valid_phy2 = true;
			} else {
				link_config.valid_phy2 = false;
			}
			gsml_config.link.push_back(link_config);
		}
		gsml_config_.push_back(gsml_config);
	}
  }catch (std::runtime_error& e) {
    return false;
  }
  return true;
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


int HobotMipiCapIml::create_gsml_gdc_bin(std::shared_ptr<pipe_contex_t> pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	if (cap_info_.gdc_enable_) {
		if (cam_info_.size() > 0 && gdc_bin_buf_.empty()) {
			sensor_msgs::msg::CameraInfo cal_cam_info;
			if (cal_tpye_ == 0) {
				auto gdc_bin = gen_gdc_bin(pipe_contex->sensor_config.isp_cfg->isp_attr.size.width, pipe_contex->sensor_config.isp_cfg->isp_attr.size.height,
						cap_info_.width, cap_info_.height, &cam_info_[0], &cal_cam_info, cap_info_.rotation_, cap_info_.cal_rotation_);
				//auto gdc_bin = gen_gdc_bin_json("./gdc_bin_custom_config.json");
				if (gdc_bin) {
					gdc_bin_buf_.push_back(gdc_bin);
					pipe_contex->gdc_bin = gdc_bin;
					cal_cam_info_.push_back(cal_cam_info);
				}
			}
		} else if (!gdc_bin_buf_.empty()) {
			pipe_contex->gdc_bin = gdc_bin_buf_[0];
		}
	}
	if ((cap_info_.rotation_ != 0) && (gdc_bin_buf_.size() == 0) && (gdc_bin_buf_r_.empty())) {
		int width;
		int height;
		if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
			width = cap_info_.height;
			height = cap_info_.width;
		} else {
			width = cap_info_.width;
			height = cap_info_.height;
		}
		auto gdc_bin = gen_gdc_bin_rotation(width, height, cap_info_.width, cap_info_.height, cap_info_.rotation_);
		if (gdc_bin) {
			gdc_bin_buf_r_.push_back(gdc_bin);
			pipe_contex->gdc_bin_r = gdc_bin;
		}
	} else if (!gdc_bin_buf_r_.empty()) {
		pipe_contex->gdc_bin_r = gdc_bin_buf_r_[0];
	}

	return 0;
}



}  // namespace mipi_cam
