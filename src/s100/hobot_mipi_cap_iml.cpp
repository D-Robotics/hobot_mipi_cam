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
		// stream_mode_==1且需旋转(场景5)：先在源分辨率生成纯旋转bin(GDC_r)，矫正bin以pre_rotation
		// 作用于已旋转图像(与X5 mode-1一致)；mode-0的旋转折入矫正bin(不生成GDC_r)；矫正bin失败时
		// 旋转bin保留入队，该链退化为仅旋转(X5同款入队时序)
		std::shared_ptr<GdcBinBuf_ST> rot_bin = nullptr;
		if ((cap_info_.stream_mode_ == 1) && (cap_info_.rotation_ != 0)) {
			rot_bin = gen_gdc_bin_rotation(sensor_cfg->isp_cfg->isp_attr.size.width,
					sensor_cfg->isp_cfg->isp_attr.size.height,
					sensor_cfg->isp_cfg->isp_attr.size.width,
					sensor_cfg->isp_cfg->isp_attr.size.height, cap_info_.rotation_);
		}
		int cal_in_width = sensor_cfg->isp_cfg->isp_attr.size.width;
		int cal_in_height = sensor_cfg->isp_cfg->isp_attr.size.height;
		if ((rot_bin != nullptr) && ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0))) {
			cal_in_width = sensor_cfg->isp_cfg->isp_attr.size.height;
			cal_in_height = sensor_cfg->isp_cfg->isp_attr.size.width;
		}
		// mode-1：末端GDC矫正+缩放，out=cap；mode-0(与X5 mode-0一致)：GDC 1:1(旋转折入矫正bin)，
		// out=源分辨率(90/270交换)，主码流缩放由GDC后PYM2 group0完成(硬件契约：喂PYM的GDC不缩放)；
		// sub码流未启用时无PYM2，GDC即链路末端，保持缩放到cap(契约允许末端GDC缩放)
		int cal_out_width = cap_info_.width;
		int cal_out_height = cap_info_.height;
		if ((cap_info_.stream_mode_ != 1) && cap_info_.sub_stream_enable_) {
			cal_out_width = sensor_cfg->isp_cfg->isp_attr.size.width;
			cal_out_height = sensor_cfg->isp_cfg->isp_attr.size.height;
			if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
				cal_out_width = sensor_cfg->isp_cfg->isp_attr.size.height;
				cal_out_height = sensor_cfg->isp_cfg->isp_attr.size.width;
			}
		}
		auto gdc_bin = gen_gdc_bin_stereo(cal_in_width, cal_in_height,
				cal_out_width, cal_out_height, cam_info_, cal_cam_info_, cap_info_.rotation_, cap_info_.cal_rotation_, cap_info_.cal_alpha_,
				rot_bin != nullptr);

		if (gdc_bin.size() == 2) {
			if (rot_bin != nullptr) {
				gdc_bin_buf_.push_back(rot_bin);
				pipe_contex[0]->gdc_bin_r = rot_bin;
				pipe_contex[1]->gdc_bin_r = rot_bin;
			}
			gdc_bin_buf_.push_back(gdc_bin[0]);
			gdc_bin_buf_.push_back(gdc_bin[1]);
			pipe_contex[0]->gdc_bin = gdc_bin[0];
			pipe_contex[1]->gdc_bin = gdc_bin[1];
			}
	}
	// 无矫正bin的纯旋转回退：以源分辨率1:1旋转(PYM1直通层喂GDC_r，gen_gdc_bin_rotation内部
	// 90/270自动把输出交换为旋转后尺寸，out参数被覆盖)；mode-1+rotation且gdc_enable=false、
	// 或矫正bin生成失败(任意mode)时走到这里
	if ((cap_info_.rotation_ != 0) && (gdc_bin_buf_.size() == 0)) {
		vp_sensor_config_t *sensor_conf = &pipe_contex[0]->sensor_config;
		auto gdc_bin = gen_gdc_bin_rotation(sensor_conf->isp_cfg->isp_attr.size.width,
			sensor_conf->isp_cfg->isp_attr.size.height, cap_info_.width, cap_info_.height, cap_info_.rotation_);
		if (gdc_bin) {
			gdc_bin_buf_.push_back(gdc_bin);
			pipe_contex[0]->gdc_bin_r = gdc_bin;
			pipe_contex[1]->gdc_bin_r = gdc_bin;
		}
	}

	// ---- stream_mode_==0(双码流均GDC矫正, 采用PYM+GDC+PYM流程)：子码流内参=主码流矫正内参 ----
	// 子码流图像=PYM2 group1从矫正后源分辨率图像缩放到sub尺寸，内参(交换后源尺度)由
	// scaleSubStreamCamInfo等比缩放到sub(与X5 mode-0语义一致)
	if ((cap_info_.stream_mode_ == 0) && cap_info_.sub_stream_enable_ && (cal_cam_info_.size() == 2)) {
		cal_cam_info_sub_ = cal_cam_info_;
	} else if ((cap_info_.stream_mode_ == 1) && cap_info_.sub_stream_enable_ && (cam_info_.size() == 2)) {
		// stream_mode_==1(主码流矫正, 子码流仅旋转不矫正)：子码流内参=原始内参(与X5 mode-1语义一致)
		cal_cam_info_sub_ = cam_info_;
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
			// stream_mode_==1且需旋转(场景5)：先在源分辨率生成纯旋转bin(GDC_r)，矫正bin以pre_rotation
			// 作用于已旋转图像(与X5 mode-1一致)；mode-0的旋转折入矫正bin(不生成GDC_r)；矫正bin失败时
			// 旋转bin保留入队，该链退化为仅旋转(X5同款入队时序)
			std::shared_ptr<GdcBinBuf_ST> rot_bin = nullptr;
			if ((cap_info_.stream_mode_ == 1) && (cap_info_.rotation_ != 0)) {
				rot_bin = gen_gdc_bin_rotation(sensor_cfg->isp_cfg->isp_attr.size.width,
						sensor_cfg->isp_cfg->isp_attr.size.height,
						sensor_cfg->isp_cfg->isp_attr.size.width,
						sensor_cfg->isp_cfg->isp_attr.size.height, cap_info_.rotation_);
			}
			int cal_in_width = sensor_cfg->isp_cfg->isp_attr.size.width;
			int cal_in_height = sensor_cfg->isp_cfg->isp_attr.size.height;
			if ((rot_bin != nullptr) && ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0))) {
				cal_in_width = sensor_cfg->isp_cfg->isp_attr.size.height;
				cal_in_height = sensor_cfg->isp_cfg->isp_attr.size.width;
			}
			// mode-1：末端GDC矫正+缩放，out=cap；mode-0(与X5 mode-0一致)：GDC 1:1(旋转折入矫正bin)，
			// out=源分辨率(90/270交换)，主码流缩放由GDC后PYM2 group0完成(硬件契约：喂PYM的GDC不缩放)；
			// sub码流未启用时无PYM2，GDC即链路末端，保持缩放到cap(契约允许末端GDC缩放)
			int cal_out_width = cap_info_.width;
			int cal_out_height = cap_info_.height;
			if ((cap_info_.stream_mode_ != 1) && cap_info_.sub_stream_enable_) {
				cal_out_width = sensor_cfg->isp_cfg->isp_attr.size.width;
				cal_out_height = sensor_cfg->isp_cfg->isp_attr.size.height;
				if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
					cal_out_width = sensor_cfg->isp_cfg->isp_attr.size.height;
					cal_out_height = sensor_cfg->isp_cfg->isp_attr.size.width;
				}
			}
			auto gdc_bin = gen_gdc_bin(cal_in_width, cal_in_height,
					cal_out_width, cal_out_height, &cam_info_[0], &cal_cam_info, cap_info_.rotation_, cap_info_.cal_rotation_,
					0.0, rot_bin != nullptr);
			//auto gdc_bin = gen_gdc_bin_json("./gdc_bin_custom_config.json");
			if (gdc_bin) {
				if (rot_bin != nullptr) {
					gdc_bin_buf_.push_back(rot_bin);
					pipe_contex[0]->gdc_bin_r = rot_bin;
				}
				gdc_bin_buf_.push_back(gdc_bin);
				pipe_contex[0]->gdc_bin = gdc_bin;
				cal_cam_info_.push_back(cal_cam_info);
			}
		}
	}
	// 无矫正bin的纯旋转回退：以源分辨率1:1旋转(PYM1直通层喂GDC_r，gen_gdc_bin_rotation内部
	// 90/270自动把输出交换为旋转后尺寸，out参数被覆盖)；mode-1+rotation且gdc_enable=false、
	// 或矫正bin生成失败(任意mode)时走到这里
	if ((cap_info_.rotation_ != 0) && (gdc_bin_buf_.size() == 0)) {
		vp_sensor_config_t *sensor_conf = &pipe_contex[0]->sensor_config;
		auto gdc_bin = gen_gdc_bin_rotation(sensor_conf->isp_cfg->isp_attr.size.width,
			sensor_conf->isp_cfg->isp_attr.size.height, cap_info_.width, cap_info_.height, cap_info_.rotation_);
		if (gdc_bin) {
			gdc_bin_buf_.push_back(gdc_bin);
			pipe_contex[0]->gdc_bin_r = gdc_bin;
		}
	}
	// ---- stream_mode_==0(双码流均GDC矫正, 采用PYM+GDC+PYM流程)：子码流内参=主码流矫正内参 ----
	if ((cap_info_.stream_mode_ == 0) && cap_info_.sub_stream_enable_ && (cal_cam_info_.size() > 0)) {
		cal_cam_info_sub_ = cal_cam_info_;
	} else if ((cap_info_.stream_mode_ == 1) && cap_info_.sub_stream_enable_ && (cam_info_.size() > 0)) {
		// stream_mode_==1(主码流矫正, 子码流仅旋转不矫正)：子码流内参=原始内参(与X5 mode-1语义一致)
		cal_cam_info_sub_ = cam_info_;
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
					//printf("index: %d  sensor_name: %-16s \tconfig_file:%s\n", i, vp_gmsl_config_list[i]->sensor_name, vp_gmsl_config_list[i]->config_file);
					if (strcasecmp(vp_gmsl_config_list[i]->sensor_name, link.sensor_type.c_str()) == 0) {
						sensor_cfg = vp_gmsl_config_list[i];
						break;
					}
				}
				if (sensor_cfg == nullptr) {
					return -1;
				}
				if (link.camera_mode == "dual") {
					deserial_config_update(&des_contex->deserial_attr, sensor_cfg->camera_config, link.link_port);
					if (link.valid_port2) {
						deserial_config_update(&des_contex->deserial_attr, sensor_cfg->camera_config, link.link_port2);
					}
				} else {
					deserial_config_update(&des_contex->deserial_attr, sensor_cfg->camera_config, link.link_port);
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
					//printf("index: %d  sensor_name: %-16s \tconfig_file:%s\n", i, vp_gmsl_config_list[i]->sensor_name, vp_gmsl_config_list[i]->config_file);
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
					std::vector<sensor_msgs::msg::CameraInfo> local_cam_info;
					std::vector<std::shared_ptr<GdcBinBuf_ST>> gdc_bins;
					if (!link.calibration_file.empty())
					{
						std::string cal_file_path;
						if (link.calibration_file[0] == '/') {
							// 绝对路径直接用
							cal_file_path = link.calibration_file;
						} else {
							cal_file_path = cap_info_.config_path + link.calibration_file;
						}
						RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
										"Dual cal_file_path: %s",  cal_file_path.c_str());
						sensor_msgs::msg::CameraInfo cam_l, cam_r;
						bool cal_ok = mipi_calibration::GetInstance().getDualCamCalibrationIml(cam_l, cam_r, cal_file_path);
						if(cal_ok) {
							local_cam_info = {cam_l, cam_r};
							RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
											"Dual calibration loaded from file: %s", cal_file_path.c_str());
						} else {
							RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
											"Calibration file failed: %s, will try EEPROM", cal_file_path.c_str());
						}
					}
					if (local_cam_info.size() != 2) {
						if (strcasecmp(pipe_contex_tmp->sensor_config.sensor_name, "ov02b10-1300p25") == 0) {
							mipi_calibration::GetInstance().getDualCamCalibrationFromEeprom_baolong(local_cam_info);
						}
					}
					if (local_cam_info.size() == 2)
					{
						gdc_bins = create_gsml_gdc_bin_stereo(pipe_contex_tmp, &local_cam_info);
						cam_info_.push_back(local_cam_info[0]);
						cam_info_.push_back(local_cam_info[1]);
					}

					if (gdc_bins.size() == 2)
					{
						gdc_bin_buf_.push_back(gdc_bins[0]);
						gdc_bin_buf_.push_back(gdc_bins[1]);
						if (link.dual_seq == 1) {
							pipe_contex_tmp->gdc_bin = gdc_bins[1];
						} else {
							pipe_contex_tmp->gdc_bin = gdc_bins[0];
						}
					}
					if ((cap_info_.rotation_ != 0) && gdc_bins.empty())
					{
						if (gdc_bin_buf_r_.empty())
						{
							// 无矫正bin的纯旋转回退：以源分辨率1:1旋转(gen_gdc_bin_rotation内部90/270
							// 自动交换输出尺寸，out参数被覆盖)，PYM1直通层喂GDC_r
							int src_w = 0, src_h = 0;
							if (pipe_contex_tmp->sensor_config.pym_cfg != nullptr) {
								src_w = pipe_contex_tmp->sensor_config.pym_cfg->chn_ctrl.src_in_width;
								src_h = pipe_contex_tmp->sensor_config.pym_cfg->chn_ctrl.src_in_height;
							} else if (pipe_contex_tmp->sensor_config.isp_cfg != nullptr) {
								src_w = pipe_contex_tmp->sensor_config.isp_cfg->isp_attr.size.width;
								src_h = pipe_contex_tmp->sensor_config.isp_cfg->isp_attr.size.height;
							}
							auto rot_bin = (src_w > 0 && src_h > 0) ? gen_gdc_bin_rotation(
								 src_w, src_h, cap_info_.width, cap_info_.height, cap_info_.rotation_) : nullptr;
							if (rot_bin)
							{
								gdc_bin_buf_r_.push_back(rot_bin);
								pipe_contex_tmp->gdc_bin_r = rot_bin;
							}
						}
						else
						{
							pipe_contex_tmp->gdc_bin_r = gdc_bin_buf_r_[0];
						}
					}
					#endif
					ret = create_and_run_vflow_step2(pipe_contex_tmp);
					ERR_CON_EQ(ret, 0);
					pipeline_num++;
					
					auto pipe_contex_tmp_2 = std::make_shared<pipe_contex_t>();
					pipe_contex_tmp_2->cap_info_ = &cap_info_;
					copy_config(&pipe_contex_tmp_2->sensor_config, sensor_cfg);
					pipe_contex_tmp_2->des_fd = des_fd;
					if (pipe_contex_tmp_2->sensor_config.camera_slave_config != NULL) {
						RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "Copy camera slave config");
						pipe_contex_tmp_2->sensor_config.camera_config = pipe_contex_tmp_2->sensor_config.camera_slave_config;
					}
					pipe_contex_tmp_2->sensor_config.vin_attr->vin_node_attr.cim_attr.mipi_rx = link.mipi_rx2;
					pipe_contex_tmp_2->sensor_config.vin_attr->vin_node_attr.cim_attr.vc_index = pipeline_num % 4;
					if (link.valid_phy2 && pipe_contex_tmp_2->sensor_config.camera_config->mipi_cfg) {
						pipe_contex_tmp_2->sensor_config.camera_config->mipi_cfg->rx_attr.phy = link.phy2;
					}
					if (link.dual_mode == 1) {
						pipe_contex_tmp_2->gsml_link_port_ = link.link_port2;
						pipe_contex_tmp_2->camera_bind_ = true;
					} else {
						pipe_contex_tmp_2->gsml_link_port_ = -1;
						pipe_contex_tmp_2->camera_bind_ = false;
					}
					pipeline_connect_param_init(pipe_contex_tmp_2);
					ret = create_and_run_vflow_step1(pipe_contex_tmp_2);
					ERR_CON_EQ(ret, 0);

					if (gdc_bins.size() == 2) {
						if (link.dual_seq == 1) {
							pipe_contex_tmp_2->gdc_bin = gdc_bins[0];
						} else {
							pipe_contex_tmp_2->gdc_bin = gdc_bins[1];
						}
						// stream_mode_==1双GDC链(场景5)：旋转bin同步传播到同link的第二路pipe
						if (pipe_contex_tmp->gdc_bin_r) {
							pipe_contex_tmp_2->gdc_bin_r = pipe_contex_tmp->gdc_bin_r;
						}
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
					pipe_contex.push_back(pipe_contex_tmp);
					des_contex->pipe.push_back(pipe_contex_tmp);
					pipeline_num++;					
				}
			}
		}
		if (pipe_contex.size() == 1) {
			cap_info_.device_mode_ = "single";
			combine_flag_ = false;
		} else if (pipe_contex.size() == 2) {
			cap_info_.device_mode_ = "dual";
			combine_flag_ = true;
		} else {
			cap_info_.device_mode_ = "multi";
			combine_flag_ = true;
		}
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
	// ---- stream_mode_==0(双码流均GDC矫正)：子码流内参=主码流矫正内参 ----
	// ---- stream_mode_==1(主码流矫正, 子码流仅旋转不矫正)：子码流内参=原始内参(与X5 mode-1语义一致) ----
	if ((cap_info_.stream_mode_ == 0) && cap_info_.sub_stream_enable_ && (cal_cam_info_.size() >= 2)) {
		cal_cam_info_sub_ = cal_cam_info_;
	} else if ((cap_info_.stream_mode_ == 1) && cap_info_.sub_stream_enable_ && (cam_info_.size() >= 2)) {
		cal_cam_info_sub_ = cam_info_;
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
		// 销毁PYM2独立flow与释放GDC后PYM的独立配置副本
		if (contex->vflow_post_fd != 0) {
			hbn_vflow_destroy(contex->vflow_post_fd);
			contex->vflow_post_fd = 0;
		}
		if (contex->pym_cfg_post != nullptr) {
			free(contex->pym_cfg_post);
			contex->pym_cfg_post = nullptr;
		}
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
    // PYM2所在独立flow(M2M节点,由桥接线程sendframe喂帧)
    if (contex->pym_post_valid && (contex->vflow_post_fd != 0)) {
      ret = hbn_vflow_start(contex->vflow_post_fd);
      ERR_CON_EQ(ret, 0);
    }
    RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
      "vflow start ok, vflow:%ld, vin:%p, isp:%p, ynr:%p, pym:%p, gdc:%p valid:%d, gdc_r:%p valid_r:%d, stream:%p, stream_group:%d",
      contex->vflow_fd, contex->vin_node_handle, contex->isp_node_handle, contex->ynr_node_handle,
      contex->pym_node_handle, contex->gdc_node_handle, contex->gdc_init_valid,
      contex->gdc_node_handle_r, contex->gdc_init_valid_r, contex->stream_handle, contex->stream_group);
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
	// GDC→PYM2(M2M)桥接线程(每pipe独立,避免sendframe阻塞串行化)：任一pipe启用PYM+GDC+PYM流程时启动
	for (int i = 0; i < (int)pipe_contex.size(); i++) {
		auto contex = pipe_contex[i];
		if (contex->pym_post_valid && contex->pym_post_src_handle) {
			task_.emplace_back(std::make_shared<std::thread>(
				std::bind(&HobotMipiCapIml::gdcToPymBridgeTask, this, i)));
		}
	}
	if (combine_flag_) {
		for(auto contex : pipe_contex) {
			v_frame_que_.push_back(std::make_shared<FrameQueue>());
		}
		task_.emplace_back(std::make_shared<std::thread>(std::bind(&HobotMipiCapIml::sync_task, this)));
	}
	if (cap_info_.sub_stream_enable_ == true) {
		for(auto contex : pipe_contex) {
			auto que_manger = std::make_shared<BuffQueueManage>();
			que_manger->creat_buff(5);
			v_sub_buff_que_manger_.push_back(que_manger);
		}
		sub_combine_buff_que_manger_ = std::make_shared<BuffQueueManage>();
		sub_combine_buff_que_manger_->creat_buff(5);
		if (combine_flag_) {
			for(auto contex : pipe_contex) {
				v_sub_frame_que_.push_back(std::make_shared<FrameQueue>());
			}
			task_.emplace_back(std::make_shared<std::thread>(std::bind(&HobotMipiCapIml::sub_sync_task, this)));
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
  for (auto task : task_) {
    task->join();
  }
  task_.clear();
  started_ = false;
  for(auto contex : pipe_contex){
    ret = hbn_vflow_stop(contex->vflow_fd);
    ERR_CON_EQ(ret, 0);
    // PYM2独立flow先于节点关闭停止
    if (contex->vflow_post_fd != 0) {
      ret = hbn_vflow_stop(contex->vflow_post_fd);
      ERR_CON_EQ(ret, 0);
    }
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
	if (contex->pym_node_handle_post != 0) {
		hbn_vnode_close(contex->pym_node_handle_post);
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

int HobotMipiCapIml::copyGroupFrameToBuffer(const hbn_vnode_image_group_t &out_img, int group_idx, std::shared_ptr<VideoBuffer> buff_ptr) {
	if ((buff_ptr == nullptr) || (group_idx < 0) || (group_idx >= 6)) {
		return -1;
	}
	const auto &graph = out_img.buf_group.graph_group[group_idx];
	if ((graph.virt_addr[0] == nullptr) || (graph.virt_addr[1] == nullptr) ||
		(graph.width == 0) || (graph.height == 0) || (graph.size[0] == 0)) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
			"invalid pym group frame, group_idx:%d, width:%d, height:%d, size:%lu,%lu",
			group_idx, graph.width, graph.height, graph.size[0], graph.size[1]);
		return -1;
	}
	hb_mem_invalidate_buf_with_vaddr((uint64_t)graph.virt_addr[0], graph.size[0]);
	hb_mem_invalidate_buf_with_vaddr((uint64_t)graph.virt_addr[1], graph.size[1]);
	struct timeval tv;
	gettimeofday(&tv, NULL);
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	uint64_t timestamp_1 = tv.tv_sec * 1e9 + tv.tv_usec * 1e3;
	uint64_t timestamp_2 = ts.tv_sec * 1e9 + ts.tv_nsec;
	if ("realtime" == cap_info_.frame_ts_type_) {
		buff_ptr->timestamp = out_img.info.timestamps + (timestamp_1 - timestamp_2);
	} else {
		buff_ptr->timestamp = out_img.info.timestamps;
	}
	buff_ptr->frame_id = out_img.info.frame_id;
	buff_ptr->stride = graph.stride;
	buff_ptr->width = graph.width;
	buff_ptr->height = graph.height;
	buff_ptr->buff_size = graph.size[0] + graph.size[1];
	buff_ptr->buff.resize(buff_ptr->buff_size);
	buff_ptr->encode = "nv12";
	memcpy(buff_ptr->buff.data(), graph.virt_addr[0], graph.size[0]);
	memcpy(buff_ptr->buff.data() + graph.size[0], graph.virt_addr[1], graph.size[1]);
	RCLCPP_DEBUG(rclcpp::get_logger("mipi_cap"),
		"capture pym group[%d], id:%d, width:%d, height:%d, stride:%d, ts:%lu",
		group_idx, buff_ptr->frame_id, buff_ptr->width, buff_ptr->height, buff_ptr->stride, buff_ptr->timestamp);
	return 0;
}

int HobotMipiCapIml::getVnodeFrameGroup(hbn_vnode_handle_t handle, int group_idx, std::shared_ptr<VideoBuffer> buff_ptr) {
	if (buff_ptr == nullptr) {
		return -1;
	}
	hbn_vnode_image_group_t out_img;
	int ret = hbn_vnode_getframe_group(handle, 0, 1000, &out_img);
	if (ret != 0) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_vnode_getframe_group handle = %p, group_idx = %d, ret = %d failed\n", handle, group_idx, ret);
		return -1;
	}
	ret = copyGroupFrameToBuffer(out_img, group_idx, buff_ptr);
	hbn_vnode_releaseframe_group(handle, 0, &out_img);
	return ret;
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
		ochn_fd[i] = -1;
		int fd_ret = hbn_vnode_get_fd(pipe_contex[i]->stream_handle, 0, &ochn_fd[i]);
		RCLCPP_DEBUG(rclcpp::get_logger("mipi_cap"),
			"multiFrameTask fd init, pipe:%d, stream_handle:%p, stream_group:%d, get_fd_ret:%d, fd:%d",
			i, pipe_contex[i]->stream_handle, pipe_contex[i]->stream_group, fd_ret, ochn_fd[i]);
	});

	int select_timeout_count = 0;
	while (started_) {
	  max_handle = 0;
	  FD_ZERO(&readfds);
	  std::for_each(indices.begin(), indices.end(), [&](int i) {
		if (ochn_fd[i] >= 0) {
			FD_SET(ochn_fd[i], &readfds);
			max_handle = max_handle > ochn_fd[i]?max_handle : ochn_fd[i];
		}
	  });

	  timeout.tv_sec = 2;
	  timeout.tv_usec = 0;
	  result = select(max_handle + 1, &readfds, nullptr, nullptr, &timeout);
	  if (result == -1) {
		  std::cerr << "Select error" << std::endl;
		  break;
	  } else if (result == 0) {
		  // 超时
		  select_timeout_count++;
		  RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
			"select timeout, count:%d, max_handle:%d, pipe_num:%d, fd[0]:%d, fd[1]:%d",
			select_timeout_count, max_handle, pipe_num,
			pipe_num > 0 ? ochn_fd[0] : -1, pipe_num > 1 ? ochn_fd[1] : -1);
		  continue;
	  } else {
			  RCLCPP_DEBUG(rclcpp::get_logger("mipi_cap"),
				"select ready, result:%d, max_handle:%d", result, max_handle);
			  for (int i = 0; i < ochn_fd.size(); i++) {
				  //if (!rclcpp::ok()) break;
				  if (FD_ISSET(ochn_fd[i], &readfds)) {
					std::shared_ptr<VideoBuffer> buff_ptr = v_buff_que_manger_[i]->get_empty_buff();
					if (!buff_ptr) {
						continue;
					}
					if (pipe_contex[i]->stream_group) {
						hbn_vnode_image_group_t out_img;
						ret = hbn_vnode_getframe_group(pipe_contex[i]->stream_handle, 0, 1000, &out_img);
						if (ret == 0) {
							ret = copyGroupFrameToBuffer(out_img, pipe_contex[i]->main_stream_group_idx, buff_ptr);
							if (ret == 0) {
								if (combine_flag_) {
									auto buff_tmp = std::make_shared<VideoBuffer>(*buff_ptr);
									v_frame_que_[i]->push(buff_tmp);
								}
								buff_ptr->return_data_que();
							} else {
								buff_ptr->return_empty_que();
							}
							if (cap_info_.sub_stream_enable_ && pipe_contex[i]->sub_stream_valid &&
								(i < v_sub_buff_que_manger_.size())) {
								auto sub_buff_ptr = v_sub_buff_que_manger_[i]->get_empty_buff();
								if (sub_buff_ptr) {
									int sub_ret = copyGroupFrameToBuffer(out_img, pipe_contex[i]->sub_stream_group_idx, sub_buff_ptr);
									if (sub_ret == 0) {
										if (combine_flag_ && (i < v_sub_frame_que_.size())) {
											auto buff_tmp = std::make_shared<VideoBuffer>(*sub_buff_ptr);
											v_sub_frame_que_[i]->push(buff_tmp);
										}
										sub_buff_ptr->return_data_que();
									} else {
										sub_buff_ptr->return_empty_que();
									}
								}
							}
							hbn_vnode_releaseframe_group(pipe_contex[i]->stream_handle, 0, &out_img);
						} else {
							RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_vnode_getframe_group PYM pipe = %d failed, ret = %d\n", i, ret);
							buff_ptr->return_empty_que();
						}
					} else {
						ret = getVnodeFrame(pipe_contex[i]->stream_handle, 0, buff_ptr);
						if (ret == 0) {
							if (combine_flag_) {
								auto buff_tmp = std::make_shared<VideoBuffer>(*buff_ptr);
								v_frame_que_[i]->push(buff_tmp);
							}
							buff_ptr->return_data_que();
							// stream_group==0(主码流为GDC输出)的子码流来源：
							// stream_mode_==1双GDC(场景5)时子码流来自GDC_r后PYM2的group1(已旋转未矫正)；
							// stream_mode_==1仅标定或PYM2创建失败的回退时，子码流来自GDC前PYM1的group1(未矫正)
							if (cap_info_.sub_stream_enable_ &&
								pipe_contex[i]->sub_stream_valid && pipe_contex[i]->pym_node_handle &&
								(i < v_sub_buff_que_manger_.size())) {
								hbn_vnode_handle_t sub_pym_handle =
									pipe_contex[i]->pym_post_valid ? pipe_contex[i]->pym_node_handle_post
																	: pipe_contex[i]->pym_node_handle;
								auto sub_buff_ptr = v_sub_buff_que_manger_[i]->get_empty_buff();
								if (sub_buff_ptr) {
									hbn_vnode_image_group_t out_img;
									int sub_ret = hbn_vnode_getframe_group(sub_pym_handle, 0, 100, &out_img);
									if (sub_ret == 0) {
										sub_ret = copyGroupFrameToBuffer(out_img, pipe_contex[i]->sub_stream_group_idx, sub_buff_ptr);
										hbn_vnode_releaseframe_group(sub_pym_handle, 0, &out_img);
										if (sub_ret == 0) {
											if (combine_flag_ && (i < v_sub_frame_que_.size())) {
												auto buff_tmp = std::make_shared<VideoBuffer>(*sub_buff_ptr);
												v_sub_frame_que_[i]->push(buff_tmp);
											}
											sub_buff_ptr->return_data_que();
										} else {
											sub_buff_ptr->return_empty_que();
										}
									} else {
										RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_vnode_getframe_group PYM sub stream pipe = %d failed, ret = %d\n", i, sub_ret);
										sub_buff_ptr->return_empty_que();
									}
								}
							}
						} else {
							RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hbn_vnode_getframe channel = %d failed ,ret = %d\n", i,ret);
							buff_ptr->return_empty_que();
						}
					}
				  }
			  }
	  }
	}
	return;
  }

// GDC输出→PYM2(M2M输入)的CPU桥接线程(每pipe一个)：本驱动上GDC ochn→PYM ichn的bind拉流不生效
// (实测GDC出帧后PYM2从不请求,Request恒为0)，M2M输入须由CPU经sendframe喂帧
// (与hobot_cv喂VSE同款模式)；sendframe为阻塞语义(约1帧周期)，返回后即可释放GDC帧，
// 缓冲为句柄传递零拷贝，桥接开销仅为每帧两次syscall
void HobotMipiCapIml::gdcToPymBridgeTask(int pipe_idx) {
	if (!started_) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "gdc bridge: camera isn't started");
		return;
	}
	if ((pipe_idx < 0) || (pipe_idx >= (int)pipe_contex.size()) ||
		(!pipe_contex[pipe_idx]->pym_post_valid) || (!pipe_contex[pipe_idx]->pym_post_src_handle)) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "gdc bridge: invalid pipe:%d", pipe_idx);
		return;
	}
	auto contex = pipe_contex[pipe_idx];

	int ochn_fd = -1;
	int fd_ret = hbn_vnode_get_fd(contex->pym_post_src_handle, 0, &ochn_fd);
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
		"gdc bridge fd init, pipe:%d, src gdc:%p, pym2:%p, get_fd_ret:%d, fd:%d",
		pipe_idx, contex->pym_post_src_handle, contex->pym_node_handle_post, fd_ret, ochn_fd);
	if ((fd_ret != 0) || (ochn_fd < 0)) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"), "gdc bridge: get fd failed, pipe:%d", pipe_idx);
		return;
	}

	fd_set readfds;
	struct timeval timeout;
	int result;
	int select_timeout_count = 0;
	while (started_) {
		FD_ZERO(&readfds);
		FD_SET(ochn_fd, &readfds);

		timeout.tv_sec = 2;
		timeout.tv_usec = 0;
		result = select(ochn_fd + 1, &readfds, nullptr, nullptr, &timeout);
		if (result == -1) {
			std::cerr << "gdc bridge select error" << std::endl;
			break;
		} else if (result == 0) {
			select_timeout_count++;
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
				"gdc bridge select timeout, pipe:%d, count:%d", pipe_idx, select_timeout_count);
			continue;
		} else if (FD_ISSET(ochn_fd, &readfds)) {
			hbn_vnode_image_t img;
			int ret = hbn_vnode_getframe(contex->pym_post_src_handle, 0, 1000, &img);
			if (ret == 0) {
				ret = hbn_vnode_sendframe(contex->pym_node_handle_post, 0, &img);
				hbn_vnode_releaseframe(contex->pym_post_src_handle, 0, &img);
				if (ret != 0) {
					RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
						"gdc bridge sendframe pipe = %d failed, ret = %d", pipe_idx, ret);
				}
			} else {
				RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
					"gdc bridge getframe pipe = %d failed, ret = %d", pipe_idx, ret);
			}
		}
	}
	return;
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
		ch_info->pym_mode = PYM_M2M_MODE;
		ch_info->pym_slot_id = isp0_next_slot_id++; 
		ch_info->is_online_vin_pym = 0;

		//printf("	[%d] [not use isp].\n", pipeline_index);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		vin [hw:%d]\n", sensor_config->vin_attr->vin_node_attr.cim_attr.mipi_rx);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->pym_hw_id, ch_info->pym_slot_id, ch_info->pym_mode);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		vin ->%s-> pym \n",
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
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		vin [hw:%d]\n", sensor_config->vin_attr->vin_node_attr.cim_attr.mipi_rx);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		isp [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->pym_hw_id, ch_info->pym_slot_id, ch_info->pym_mode);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		vin ->%s-> isp ->%s-> pym \n",
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
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		vin [hw:%d]\n", sensor_config->vin_attr->vin_node_attr.cim_attr.mipi_rx);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		isp [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		ynr [hw:%d] [slot_id:%d] [mode:%d]\n",
			1, ch_info->ynr_slot_id, ch_info->ynr_mode);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->pym_hw_id, ch_info->pym_slot_id, ch_info->pym_mode);

		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),"		vin ->%s-> isp ->%s-> ynr ->%s-> pym\n",
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
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"), "width and height over rang, width:%d, height:%d, src_width:%d, src_height:%d, bl_width_16:%d, bl_height_16:%d", width, height, src_width, src_height, bl_width_16, bl_height_16);
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
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"), "width and height over rang,src_width:%d, src_height:%d, width:%d, height:%d",src_width, src_height,  width, height);
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
	// 链上任一GDC生效时group0=源分辨率1:1直通(硬件契约：第一个PYM不缩放，缩放交给GDC后PYM2/末端GDC)
	if ((pipe_contex->gdc_init_valid == 1) || (pipe_contex->gdc_init_valid_r == 1)) {
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
	auto setup_pym_roi = [&](int group_idx, int target_width, int target_height) -> int {
		int local_roi_sel = 0;
		int local_roi_layer = 0;
		int local_bl_width = src_width;
		int local_bl_height = src_height;
		int local_bl_stride = src_width;
		if (check_pym_config(src_width, src_height, target_width, target_height,
				local_bl_width, local_bl_height, local_bl_stride, local_roi_sel, local_roi_layer) == -1) {
			return -1;
		}
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
			"creat_pym_node group[%d]--roi_sel: %d, roi_layer: %d, bl_width: %d, bl_height: %d, out_width: %d, out_height: %d",
			group_idx, local_roi_sel, local_roi_layer, local_bl_width, local_bl_height, target_width, target_height);
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_sel[group_idx] = local_roi_sel;
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_layer[group_idx] = local_roi_layer;
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[group_idx].region_width = local_bl_width;
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[group_idx].region_height = local_bl_height;
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[group_idx].wstride_uv = ALIGN_16(target_width);
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[group_idx].wstride_y = ALIGN_16(target_width);
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[group_idx].out_width = target_width;
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[group_idx].out_height = target_height;
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[group_idx].vstride = target_height;
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_en |= (1 << group_idx);
		return 0;
	};
	pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_en = 0;
	if (check_pym_config(src_width, src_height, out_width,
			out_height, bl_width, bl_height, bl_stride, roi_sel, roi_layer) == -1) {
		return -1;
	}
	if (setup_pym_roi(pipe_contex->main_stream_group_idx, out_width, out_height) == -1) {
		return -1;
	}
	// PYM1的group1子码流配置：pym_post_valid时子码流实际取GDC后PYM2的group1，
	// 但在线链(ISP/YNR/PYM同slot)的PYM单group配置(ds_roi_en仅bit0)会被判ILLEGAL_ATTR，
	// 故group1始终配置(PYM2生效时仅闲置输出，不改变子码流来源)
	if (pipe_contex->cap_info_->sub_stream_enable_) {
		pipe_contex->sub_stream_valid = false;
		int sub_out_width;
		int sub_out_height;
		if ((pipe_contex->cap_info_->rotation_ == 90.0) || (pipe_contex->cap_info_->rotation_ == 270.0)) {
			sub_out_width = pipe_contex->cap_info_->sub_height;
			sub_out_height = pipe_contex->cap_info_->sub_width;
		} else {
			sub_out_width = pipe_contex->cap_info_->sub_width;
			sub_out_height = pipe_contex->cap_info_->sub_height;
		}
		if (setup_pym_roi(pipe_contex->sub_stream_group_idx, sub_out_width, sub_out_height) == -1) {
			RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
				"create sub stream pym roi failed, width:%d, height:%d", sub_out_width, sub_out_height);
			return -1;
		}
		pipe_contex->sub_stream_valid = true;
	}

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

	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
		"pym final cfg, handle:%p, hw:%d, slot:%d, mode:%d, output_buf_num:%d, fb_buf_num:%d, ds_roi_en:0x%x, src:%dx%d stride_y:%d stride_uv:%d, ochn_buf_num:%d",
		pipe_contex->pym_node_handle,
		pipe_contex->sensor_config.pym_cfg->hw_id,
		pipe_contex->sensor_config.pym_cfg->slot_id,
		pipe_contex->sensor_config.pym_cfg->pym_mode,
		pipe_contex->sensor_config.pym_cfg->output_buf_num,
		pipe_contex->sensor_config.pym_cfg->fb_buf_num,
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_en,
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_width,
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_height,
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_stride_y,
		pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_stride_uv,
		alloc_attr.buffers_num);
	for (int group_idx : {pipe_contex->main_stream_group_idx, pipe_contex->sub_stream_group_idx}) {
		const auto &roi = pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_info[group_idx];
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
			"pym roi cfg, group:%d, enable:%d, roi_sel:%d, roi_layer:%d, region:%dx%d, out:%dx%d, stride_y:%d, stride_uv:%d, vstride:%d",
			group_idx,
			(pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_en & (1 << group_idx)) ? 1 : 0,
			pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_sel[group_idx],
			pipe_contex->sensor_config.pym_cfg->chn_ctrl.ds_roi_layer[group_idx],
			roi.region_width, roi.region_height, roi.out_width, roi.out_height,
			roi.wstride_y, roi.wstride_uv, roi.vstride);
	}

	return 0;
}

// 创建GDC后的第二个PYM节点做主/子码流分流(PYM+GDC+PYM流程，PYM2等价于X5的VSE)：
// stream_mode_==0：输入为主GDC输出(交换后源尺寸,已完成矫正与旋转1:1)，group0缩放到cap输出主码流、group1输出子码流；
// stream_mode_==1双GDC链(场景5)：输入为GDC_r纯旋转输出(源分辨率旋转后)，group0直通后送GDC矫正缩放、group1输出子码流；
// mode-1仅旋转：输入为GDC_r输出，group0直通(主码流=旋转后源分辨率，X5语义)。
// 输入已旋转，子尺寸无需再交换宽高
int HobotMipiCapIml::create_pym_node_post(std::shared_ptr<pipe_contex_t> pipe_contex, int hw_id, int slot_id, int pym_mode,
		int in_width, int in_height) {
	if ((pipe_contex == nullptr) || (pipe_contex->sensor_config.pym_cfg == nullptr)) {
		return -1;
	}
	int ret = 0;
	uint32_t chn_id = 0;
	pipe_contex->pym_post_valid = false;

	// 从sensor的pym配置拷贝独立副本(纯POD结构)，不影响PYM1的配置
	if (pipe_contex->pym_cfg_post == nullptr) {
		pipe_contex->pym_cfg_post = (pym_cfg_t *)malloc(sizeof(pym_cfg_t));
		if (pipe_contex->pym_cfg_post == nullptr) {
			RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"), "malloc pym_cfg_post failed");
			return -1;
		}
		memcpy(pipe_contex->pym_cfg_post, pipe_contex->sensor_config.pym_cfg, sizeof(pym_cfg_t));
	}
	pym_cfg_t *pym_cfg = pipe_contex->pym_cfg_post;
	pym_cfg->hw_id = hw_id;
	pym_cfg->pym_mode = pym_mode;
	pym_cfg->slot_id = slot_id;

	// 输入=上游GDC输出：stream_mode_==0为主GDC(矫正+旋转)输出cap尺寸；双GDC链为GDC_r纯旋转输出(源分辨率旋转后)
	int src_width = in_width;
	int src_height = in_height;
	pym_cfg->chn_ctrl.src_in_width = src_width;
	pym_cfg->chn_ctrl.src_in_height = src_height;
	pym_cfg->chn_ctrl.src_in_stride_y = ALIGN_16(src_width);
	pym_cfg->chn_ctrl.src_in_stride_uv = ALIGN_16(src_width);

	// group0: 主码流目标(X5 VSE chn0规则)：mode-1=直通(=输入尺寸，链路末端还有缩放GDC或主码流即源分辨率)；
	// mode-0=缩放到cap(硬件契约：喂PYM的GDC不缩放，主码流缩放由PYM完成)
	int out_width = src_width;
	int out_height = src_height;
	if (pipe_contex->cap_info_->stream_mode_ != 1) {
		out_width = pipe_contex->cap_info_->width;
		out_height = pipe_contex->cap_info_->height;
	}
	int roi_sel = 0;
	int roi_layer = 0;
	int bl_width = src_width;
	int bl_height = src_height;
	int bl_stride = src_width;
	pym_cfg->chn_ctrl.ds_roi_en = 0;
	if (check_pym_config(src_width, src_height, out_width, out_height,
			bl_width, bl_height, bl_stride, roi_sel, roi_layer) == -1) {
		return -1;
	}
	pym_cfg->chn_ctrl.ds_roi_sel[pipe_contex->main_stream_group_idx] = roi_sel;
	pym_cfg->chn_ctrl.ds_roi_layer[pipe_contex->main_stream_group_idx] = roi_layer;
	pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->main_stream_group_idx].region_width = bl_width;
	pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->main_stream_group_idx].region_height = bl_height;
	pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->main_stream_group_idx].wstride_y = ALIGN_16(out_width);
	pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->main_stream_group_idx].wstride_uv = ALIGN_16(out_width);
	pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->main_stream_group_idx].out_width = out_width;
	pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->main_stream_group_idx].out_height = out_height;
	pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->main_stream_group_idx].vstride = out_height;
	pym_cfg->chn_ctrl.ds_roi_en |= (1 << pipe_contex->main_stream_group_idx);

	// group1: 子码流(输入已完成矫正与旋转，直接使用sub尺寸)
	pipe_contex->sub_stream_valid = false;
	int sub_out_width = pipe_contex->cap_info_->sub_width;
	int sub_out_height = pipe_contex->cap_info_->sub_height;
	if ((sub_out_width > 0) && (sub_out_height > 0)) {
		int sub_roi_sel = 0;
		int sub_roi_layer = 0;
		int sub_bl_width = src_width;
		int sub_bl_height = src_height;
		int sub_bl_stride = src_width;
		if (check_pym_config(src_width, src_height, sub_out_width, sub_out_height,
				sub_bl_width, sub_bl_height, sub_bl_stride, sub_roi_sel, sub_roi_layer) == -1) {
			RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
				"create post pym sub group failed, width:%d, height:%d", sub_out_width, sub_out_height);
			return -1;
		}
		pym_cfg->chn_ctrl.ds_roi_sel[pipe_contex->sub_stream_group_idx] = sub_roi_sel;
		pym_cfg->chn_ctrl.ds_roi_layer[pipe_contex->sub_stream_group_idx] = sub_roi_layer;
		pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->sub_stream_group_idx].region_width = sub_bl_width;
		pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->sub_stream_group_idx].region_height = sub_bl_height;
		pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->sub_stream_group_idx].wstride_y = ALIGN_16(sub_out_width);
		pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->sub_stream_group_idx].wstride_uv = ALIGN_16(sub_out_width);
		pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->sub_stream_group_idx].out_width = sub_out_width;
		pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->sub_stream_group_idx].out_height = sub_out_height;
		pym_cfg->chn_ctrl.ds_roi_info[pipe_contex->sub_stream_group_idx].vstride = sub_out_height;
		pym_cfg->chn_ctrl.ds_roi_en |= (1 << pipe_contex->sub_stream_group_idx);
		pipe_contex->sub_stream_valid = true;
	}

	hbn_buf_alloc_attr_t alloc_attr = {0};
	ret = hbn_vnode_open(HB_PYM, hw_id, AUTO_ALLOC_ID, &pipe_contex->pym_node_handle_post);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_attr(pipe_contex->pym_node_handle_post, pym_cfg);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ichn_attr(pipe_contex->pym_node_handle_post, chn_id, pym_cfg);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ochn_attr(pipe_contex->pym_node_handle_post, chn_id, pym_cfg);
	ERR_CON_EQ(ret, 0);
	alloc_attr.buffers_num = 3;
	alloc_attr.is_contig = 1;
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
						| HB_MEM_USAGE_CPU_WRITE_OFTEN
						| HB_MEM_USAGE_CACHED
						| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->pym_node_handle_post, chn_id, &alloc_attr);
	ERR_CON_EQ(ret, 0);
	// PYM2(M2M)独立成flow(不加入相机主flow、无bind)：本驱动上GDC→PYM的bind拉流不生效且
	// 与sendframe互斥("src node already bind, can't send again")，M2M节点须独立flow+
	// CPU sendframe喂帧(hobot_cv喂VSE同款模式)，由gdcToPymBridgeTask跨flow搬运
	if (pipe_contex->vflow_post_fd == 0) {
		ret = hbn_vflow_create(&pipe_contex->vflow_post_fd);
		ERR_CON_EQ(ret, 0);
	}
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_post_fd, pipe_contex->pym_node_handle_post);
	ERR_CON_EQ(ret, 0);
	pipe_contex->pym_post_valid = true;
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
		"create post pym node ok, handle:%p, hw:%d, slot:%d, mode:%d, src:%dx%d, group0(main):%dx%d, group1(sub):%dx%d, vflow_post:%ld",
		pipe_contex->pym_node_handle_post, hw_id, slot_id, pym_mode, src_width, src_height,
		out_width, out_height, sub_out_width, sub_out_height, pipe_contex->vflow_post_fd);

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

	if (pipe_contex->gdc_init_valid == 1) {
		// 双GDC链(场景5)：PYM1 group0直通src，GDC_r在源分辨率纯旋转，输出为旋转后(交换)尺寸
		if (pipe_contex->sensor_config.pym_cfg != nullptr) {
			input_width = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_width;
			input_height = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_height;
		} else {
			input_width = pipe_contex->cap_info_->width;
			input_height = pipe_contex->cap_info_->height;
		}
		if ((pipe_contex->cap_info_->rotation_ == 90.0) || (pipe_contex->cap_info_->rotation_ == 270.0)) {
			out_width = input_height;
			out_height = input_width;
		} else {
			out_width = input_width;
			out_height = input_height;
		}
	} else {
		// 无矫正bin的纯旋转链：PYM1直通层输出源分辨率喂GDC_r，1:1旋转输出=交换后源尺寸
		// (缩放由GDC后PYM2 group0完成，硬件契约：喂PYM的GDC不缩放)
		if (pipe_contex->sensor_config.pym_cfg != nullptr) {
			input_width = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_width;
			input_height = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_height;
		} else if (pipe_contex->sensor_config.isp_cfg != nullptr) {
			input_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
			input_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
		} else {
			input_width = pipe_contex->cap_info_->width;
			input_height = pipe_contex->cap_info_->height;
		}
		if ((pipe_contex->cap_info_->rotation_ == 90.0) || (pipe_contex->cap_info_->rotation_ == 270.0)) {
			out_width = input_height;
			out_height = input_width;
		} else {
			out_width = input_width;
			out_height = input_height;
		}
	}

	gdc_settings_t gdc_setting = {0};
	uint32_t hw_id = 0;
	// 双GDC链(场景5)：cal GDC已随PYM2放入post flow(见bind_gdc_pym_stream)，GDC_r留在主flow，
	// 每flow一个GDC实例(与D2每pipe一个GDC同款,同flow双GDC实例驱动不分配slot,实测不出帧)
	int gdc_ctx_id = AUTO_ALLOC_ID;
	ret = hbn_vnode_open(HB_GDC, hw_id, gdc_ctx_id, &pipe_contex->gdc_node_handle_r);
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
	// 同create_gdc_node：输出可能被PYM2等硬件节点DMA消费，须物理连续
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN |
					HB_MEM_USAGE_CPU_WRITE_OFTEN |
					HB_MEM_USAGE_CACHED |
					HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
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

	// auto input_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
	// auto input_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
	int input_width = 0, input_height = 0;
	if (pipe_contex->sensor_config.pym_cfg != nullptr)
	{
		input_width = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_width;
		input_height = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_height;
	}
	else if (pipe_contex->sensor_config.isp_cfg != nullptr)
	{
		input_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
		input_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
	}
	else if (pipe_contex->sensor_config.camera_config != nullptr)
	{
		input_width = pipe_contex->sensor_config.camera_config->width;
		input_height = pipe_contex->sensor_config.camera_config->height;
	}
	else if (pipe_contex->sensor_config.vin_attr != nullptr)
	{
		input_width = pipe_contex->sensor_config.vin_attr->vin_ichn_attr.width;
		input_height = pipe_contex->sensor_config.vin_attr->vin_ichn_attr.height;
	}
	// 双GDC链(场景5)：矫正bin以pre_rotation作用于已旋转图像，GDC输入为交换后的源尺寸
	if ((pipe_contex->gdc_bin_r != nullptr) &&
		((pipe_contex->cap_info_->rotation_ == 90.0) || (pipe_contex->cap_info_->rotation_ == 270.0))) {
		int input_tmp = input_width;
		input_width = input_height;
		input_height = input_tmp;
	}
	// mode-1：末端GDC矫正+缩放到cap；mode-0(与X5 mode-0一致)：GDC 1:1(旋转已折入矫正bin)，
	// 输出=源分辨率(90/270交换)，主码流缩放由GDC后PYM2 group0完成(硬件契约：喂PYM的GDC不缩放)；
	// sub码流未启用时无PYM2，GDC即链路末端，保持缩放到cap(契约允许末端GDC缩放)
	int out_width = pipe_contex->cap_info_->width;
	int out_height = pipe_contex->cap_info_->height;
	if ((pipe_contex->cap_info_->stream_mode_ != 1) && pipe_contex->cap_info_->sub_stream_enable_) {
		out_width = input_width;
		out_height = input_height;
		if ((pipe_contex->cap_info_->rotation_ == 90.0) || (pipe_contex->cap_info_->rotation_ == 270.0)) {
			int out_tmp = out_width;
			out_width = out_height;
			out_height = out_tmp;
		}
	}
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
	// GDC输出缓冲须为物理连续(GRAPHIC_CONTIGUOUS)：GDC被PYM2/下游硬件节点DMA消费时，
	// 非连续缓冲会导致下游M2M拉流静默失败(实测GDC出帧但PYM2从不请求,POLLHUP)
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN |
					HB_MEM_USAGE_CPU_WRITE_OFTEN |
					HB_MEM_USAGE_CACHED |
					HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	ret = hbn_vnode_set_ochn_buf_attr(pipe_contex->gdc_node_handle, chn_id, &alloc_attr);
	ERR_CON_EQ(ret, 0);
	pipe_contex->gdc_init_valid = 1;

	return 0;
}

// 创建GDC(矫正)/GDC_r(旋转)/GDC后PYM/PYM节点(pym_cfg存在时，由create_and_run_vflow与create_and_run_vflow_step2共用)。
// GDC后PYM(PYM2)做主/子码流分流：任一GDC生效，且非"stream_mode_==1仅标定"
// (该场景子码流取GDC前PYM1的group1，即当前mode-2行为)
int HobotMipiCapIml::create_gdc_pym_nodes(std::shared_ptr<pipe_contex_t> pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	int ret = 0;
	pipeline_channel_info_t *ch_info = &pipe_contex->pipe_info_;

	if (cap_info_.gdc_enable_) {
		create_gdc_node(pipe_contex);
	}
	create_gdc_node_r(pipe_contex);

	// GDC后PYM(PYM2)做主/子码流分流(stream_mode_==0双码流矫正,及旋转相关链路)：
	// PYM2输入=上游GDC输出，主码流=group0(基底层直通)、子码流=group1(缩放)
	bool need_post_pym = cap_info_.sub_stream_enable_ &&
		((pipe_contex->gdc_init_valid == 1) || (pipe_contex->gdc_init_valid_r == 1)) &&
		!((cap_info_.stream_mode_ == 1) && (pipe_contex->gdc_init_valid == 1) && (pipe_contex->gdc_init_valid_r == 0));
	// 启动校验(先于PYM节点创建，避免被PYM金字塔ROI表的模糊报错掩盖)：
	// PYM(金字塔ROI)只能降采样；mode-0主码流取PYM2 group0(缩放到cap)，
	// cap朝向须与GDC输出朝向一致(X5以gdc_resize_enable特例处理朝向不匹配，S100明确报错)
	if (need_post_pym) {
		// PYM2输入=上游GDC输出(硬件契约)=源分辨率±旋转交换(与create_gdc_node/create_gdc_node_r
		// 输出同基准)：mode-0为主GDC 1:1矫正输出、旋转链(双GDC或仅旋转)为GDC_r纯旋转输出
		int post_src_width = pipe_contex->cap_info_->width;
		int post_src_height = pipe_contex->cap_info_->height;
		if (pipe_contex->sensor_config.pym_cfg != nullptr) {
			post_src_width = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_width;
			post_src_height = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_height;
		} else if (pipe_contex->sensor_config.isp_cfg != nullptr) {
			post_src_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
			post_src_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
		}
		if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
			int post_src_tmp = post_src_width;
			post_src_width = post_src_height;
			post_src_height = post_src_tmp;
		}
		if (cap_info_.stream_mode_ != 1) {
			if ((cap_info_.width > post_src_width) || (cap_info_.height > post_src_height)) {
				RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
					"main stream cap %dx%d exceeds GDC output %dx%d, PYM cannot upscale; "
					"use stream_mode 1 (terminal gdc scaling) or reduce cap size",
					cap_info_.width, cap_info_.height, post_src_width, post_src_height);
				return -1;
			}
			
			// if (((cap_info_.width > cap_info_.height) != (post_src_width > post_src_height)) &&
			// 	(cap_info_.width != cap_info_.height) && (post_src_width != post_src_height)) {
			// 	RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
			// 		"cap %dx%d orientation mismatch with rotated source %dx%d "
			// 		"(X5 handles this via gdc_resize, S100 does not), check image size vs rotation",
			// 		cap_info_.width, cap_info_.height, post_src_width, post_src_height);
			// 	return -1;
			// }
		}
		if ((cap_info_.sub_width > post_src_width) || (cap_info_.sub_height > post_src_height)) {
			RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
				"sub stream %dx%d exceeds GDC output %dx%d, PYM cannot upscale, check sub image size",
				cap_info_.sub_width, cap_info_.sub_height, post_src_width, post_src_height);
			return -1;
		}
	}

	// PYM1先于GDC后PYM2创建：在线链PYM(ISP/YNR/PYM同slot,MANUAL模式)需先创建，
	// 否则其set_attr报ILLEGAL_ATTR(-10)；PYM1总是配置group1，
	// PYM2失败时子码流自动回退PYM1 group1(未矫正)，成功时子码流取PYM2 group1
	ret = create_pym_node(pipe_contex, ch_info->pym_hw_id, ch_info->pym_slot_id, ch_info->pym_mode);
	ERR_CON_EQ(ret, 0);

	if (need_post_pym) {
		int post_src_width = pipe_contex->cap_info_->width;
		int post_src_height = pipe_contex->cap_info_->height;
		if (pipe_contex->sensor_config.pym_cfg != nullptr) {
			post_src_width = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_width;
			post_src_height = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_height;
		} else if (pipe_contex->sensor_config.isp_cfg != nullptr) {
			post_src_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
			post_src_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
		}
		if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
			int post_src_tmp = post_src_width;
			post_src_width = post_src_height;
			post_src_height = post_src_tmp;
		}
		// PYM2用独立PYM硬件实例(与PYM1不同hw)：同一PYM hw上混用MANUAL(在线链)与
		// M2M(DDR输入)两种pym_mode，后创建者set_attr报ILLEGAL_ATTR(-10)(S100实测)
		int post_pym_hw = (ch_info->pym_hw_id == 0) ? 1 : 0;
		ret = create_pym_node_post(pipe_contex, post_pym_hw, isp0_next_slot_id++, PYM_M2M_MODE,
			post_src_width, post_src_height);
		if (ret != 0) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
				"create post pym node failed, ret:%d, main stream falls back to GDC output %dx%d "
				"(!= cap %dx%d, image dims will mismatch camera_info), sub stream falls back to pre-gdc PYM group output",
				ret, post_src_width, post_src_height, cap_info_.width, cap_info_.height);
			// PYM2失败时子码流回退PYM1 group1(PYM1已先建且总是配置group1)，恢复其有效标志
			pipe_contex->sub_stream_valid = true;
		}
	}

	return 0;
}

// 将GDC/PYM节点加入flow并按场景绑定链路、选择应用码流(pym_cfg存在时，由create_and_run_vflow与create_and_run_vflow_step2共用)。链路规则：
//   stream_mode_==1双GDC(场景5)：PYM1→GDC_r(主flow,纯旋转)→[PYM2主/子分流(post flow)]→GDC(同post flow,矫正+缩放)，
//     PYM2由GDC_r桥接喂帧、其ochn流内bind到末端GDC；主码流=GDC输出(已矫正)，子码流=PYM2 group1(已旋转未矫正)
//   仅旋转：PYM1→GDC_r(1:1旋转)→[PYM2分流]，PYM2生效时主码流=PYM2 group0(已旋转,源分辨率,X5语义)
//   矫正(stream_mode_==0)：PYM1→GDC(1:1矫正+旋转,源分辨率)→PYM2分流，
//     主码流=PYM2 group0(已矫正+缩放到cap)，子码流=PYM2 group1(已矫正+缩放)；
//     PYM2失效时主码流回退GDC输出(源分辨率,与cap不符仅WARN)、子码流回退PYM1 group1(未矫正)
//   无GDC：PYM1即应用码流(group0主/group1子)
int HobotMipiCapIml::bind_gdc_pym_stream(std::shared_ptr<pipe_contex_t> pipe_contex) {
	if (pipe_contex == nullptr) {
		return -1;
	}
	int ret = 0;
	int scene_type = pipe_contex->sensor_type_;
	pipeline_channel_info_t *ch_info = &pipe_contex->pipe_info_;

	// 1. 添加node到flow(PYM2在独立vflow_post_fd中，见create_pym_node_post，此处不添加)
	if (pipe_contex->gdc_init_valid_r == 1) {
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->gdc_node_handle_r);
		ERR_CON_EQ(ret, 0);
	}
	if (pipe_contex->gdc_init_valid == 1) {
		// 场景5(PYM2分流生效)：cal GDC不入主flow，随PYM2放入post flow——同flow两个GDC
		// 实例驱动不为第二个分配slot(实测S65535伪slot、bin不map、不出帧)，
		// 每flow一个GDC(与D2每pipe一个GDC同款)；见下方场景5bind
		bool cal_gdc_in_post = (cap_info_.stream_mode_ == 1) &&
			(pipe_contex->gdc_init_valid_r == 1) && pipe_contex->pym_post_valid;
		if (!cal_gdc_in_post) {
			ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
								pipe_contex->gdc_node_handle);
			ERR_CON_EQ(ret, 0);
		}
	}

	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->pym_node_handle);
	ERR_CON_EQ(ret, 0);
	// 2. 场景链路绑定(至PYM1)
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
	// 3. 默认应用码流=PYM1 group输出
	pipe_contex->stream_handle = pipe_contex->pym_node_handle;
	pipe_contex->stream_group = 1;

	// 4. GDC链路绑定与码流选择
	if ((cap_info_.stream_mode_ == 1) && (pipe_contex->gdc_init_valid_r == 1) && (pipe_contex->gdc_init_valid == 1)) {
		// 场景5(stream_mode_==1双GDC)：主码流=GDC矫正输出，子码流=PYM2 group1(已旋转未矫正)
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "start gdc rotation then gdc cal (dual gdc chain).\n");
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->pym_node_handle,
							0,
							pipe_contex->gdc_node_handle_r,
							0);
		ERR_CON_EQ(ret, 0);
		if (pipe_contex->pym_post_valid) {
			// 契约链路：PYM1→GDC_r(主flow,1:1纯旋转)→PYM2(post flow,桥接sendframe喂帧,
			// group0直通+group1子码流)→GDC(post flow,末端矫正+缩放到cap)。
			// cal GDC与PYM2同post flow(同flow双GDC实例驱动不分配slot,见上方add注释)，
			// PYM ochn→GDC ichn的流内bind可拉流(D2的PYM1→GDC同款；GDC→PYM方向不拉流故桥接保留)
			ret = hbn_vflow_add_vnode(pipe_contex->vflow_post_fd,
								pipe_contex->gdc_node_handle);
			ERR_CON_EQ(ret, 0);
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_post_fd,
								pipe_contex->pym_node_handle_post,
								0,
								pipe_contex->gdc_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			// GDC_r→PYM(M2M)：PYM2由CPU桥接sendframe喂帧，此处仅记录源节点
			pipe_contex->pym_post_src_handle = pipe_contex->gdc_node_handle_r;
		} else {
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->gdc_node_handle_r,
								0,
								pipe_contex->gdc_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
		}
		pipe_contex->stream_handle = pipe_contex->gdc_node_handle;
		pipe_contex->stream_group = 0;
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
			"PYM->GDC_r->PYM->GDC dual gdc streams active: main %dx%d rectified, sub %dx%d rotated",
			pipe_contex->cap_info_->width, pipe_contex->cap_info_->height,
			pipe_contex->cap_info_->sub_width, pipe_contex->cap_info_->sub_height);
	} else if (pipe_contex->gdc_init_valid_r == 1) {
		// 仅旋转：PYM1→GDC_r(纯旋转)→[PYM2分流(主=group0,子=group1)]
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc rotation.\n");
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->pym_node_handle,
							0,
							pipe_contex->gdc_node_handle_r,
							0);
		ERR_CON_EQ(ret, 0);
		if (pipe_contex->pym_post_valid) {
			// GDC_r→PYM(M2M)：PYM2在独立flow中由CPU桥接sendframe喂帧，此处仅记录源节点
			pipe_contex->pym_post_src_handle = pipe_contex->gdc_node_handle_r;
			pipe_contex->stream_handle = pipe_contex->pym_node_handle_post;
			pipe_contex->stream_group = 1;
		} else {
			pipe_contex->stream_handle = pipe_contex->gdc_node_handle_r;
			pipe_contex->stream_group = 0;
		}
	} else if (pipe_contex->gdc_init_valid == 1) {
		// 矫正(stream_mode_==0)：PYM1→GDC(矫正+旋转+缩放)→PYM2分流(主=group0,子=group1)
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc cal.\n");
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->pym_node_handle,
							0,
							pipe_contex->gdc_node_handle,
							0);
		ERR_CON_EQ(ret, 0);
		if (pipe_contex->pym_post_valid) {
			// GDC→PYM(M2M)：PYM2在独立flow中由CPU桥接线程sendframe喂帧
			// (本驱动上GDC→PYM的bind拉流不生效且与sendframe互斥，故不bind，见create_pym_node_post)
			pipe_contex->pym_post_src_handle = pipe_contex->gdc_node_handle;
			pipe_contex->stream_handle = pipe_contex->pym_node_handle_post;
			pipe_contex->stream_group = 1;
			RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
				"PYM+GDC+PYM streams active: pym -> gdc -> post pym (main %dx%d, sub %dx%d), gdc->pym by cpu bridge",
				pipe_contex->cap_info_->width, pipe_contex->cap_info_->height,
				pipe_contex->cap_info_->sub_width, pipe_contex->cap_info_->sub_height);
		} else {
			pipe_contex->stream_handle = pipe_contex->gdc_node_handle;
			pipe_contex->stream_group = 0;
		}
	} else {
		RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
			"PYM selected as app stream, no GDC bind, vflow:%ld, pym:%p, stream:%p, stream_group:%d",
			pipe_contex->vflow_fd, pipe_contex->pym_node_handle, pipe_contex->stream_handle, pipe_contex->stream_group);
	}

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
			pipe_contex->sensor_config.vin_attr->vin_node_attr.cim_attr.func.skip_frame = 0;
		} else {
			pipe_contex->sensor_config.camera_config->fps = pipe_contex->cap_info_->fps;
			pipe_contex->sensor_config.camera_config->mipi_cfg->rx_attr.fps = pipe_contex->cap_info_->fps;
			pipe_contex->sensor_config.camera_config->sensor_mode = 1;
			for (auto& attr : pipe_contex->sensor_config.vin_attr->vin_node_attr.lpwm_attr.lpwm_chn_attr) {
				attr.enable = 0;
			}
			pipe_contex->sensor_config.vin_attr->vin_node_attr.cim_attr.func.skip_frame = 1;
			pipe_contex->sensor_config.vin_attr->vin_node_attr.cim_attr.func.output_fps = pipe_contex->cap_info_->fps;
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
		ret = create_gdc_pym_nodes(pipe_contex);
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
		ret = bind_gdc_pym_stream(pipe_contex);
		ERR_CON_EQ(ret, 0);
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
		// pym链路：创建GDC(矫正)/GDC_r(旋转)/GDC后PYM/PYM节点(与MIPI直连路径共用，含PYM2分流)
		ret = create_gdc_pym_nodes(pipe_contex);
		ERR_CON_EQ(ret, 0);
	} else {
		// 无pym传感器：仅创建GDC节点并直接绑定到流上(保持GMSL既有行为)
		if (cap_info_.gdc_enable_) {
			create_gdc_node(pipe_contex);
		}
		create_gdc_node_r(pipe_contex);
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
	}

	if (pipe_contex->sensor_config.pym_cfg) {
		// 添加GDC/PYM节点到flow并按场景绑定链路、选择应用码流(与MIPI直连路径共用)
		ret = bind_gdc_pym_stream(pipe_contex);
		ERR_CON_EQ(ret, 0);
	} else {
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
		pipe_contex->stream_handle = pipe_contex->vin_node_handle;
		pipe_contex->stream_group = 0;
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
		}
		if (pipe_contex->gdc_init_valid_r == 1) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc rotation.\n");
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->stream_handle,
								0,
								pipe_contex->gdc_node_handle_r,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->gdc_node_handle_r;
			pipe_contex->stream_group = 0;
		} else if (pipe_contex->gdc_init_valid == 1) {
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "X5 start gdc cal.\n");
			ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->stream_handle,
								0,
								pipe_contex->gdc_node_handle,
								0);
			ERR_CON_EQ(ret, 0);
			pipe_contex->stream_handle = pipe_contex->gdc_node_handle;
			pipe_contex->stream_group = 0;
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
			if (link.isMember("calibration_file")) {
          	link_config.calibration_file = link["calibration_file"].asString();
        	} else {
          	link_config.calibration_file = "";
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
			// stream_mode_==1且需旋转(场景5)：先在源分辨率生成纯旋转bin(GDC_r)，矫正bin以pre_rotation
			// 作用于已旋转图像；mode-0的旋转折入矫正bin(不生成GDC_r)；矫正bin失败时旋转bin不入队
			// (gsml路径)，走下方源分辨率回退重新生成
			std::shared_ptr<GdcBinBuf_ST> rot_bin = nullptr;
			if ((cap_info_.stream_mode_ == 1) && (cap_info_.rotation_ != 0)) {
				rot_bin = gen_gdc_bin_rotation(pipe_contex->sensor_config.isp_cfg->isp_attr.size.width,
						pipe_contex->sensor_config.isp_cfg->isp_attr.size.height,
						pipe_contex->sensor_config.isp_cfg->isp_attr.size.width,
						pipe_contex->sensor_config.isp_cfg->isp_attr.size.height, cap_info_.rotation_);
			}
			int cal_in_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
			int cal_in_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
			if ((rot_bin != nullptr) && ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0))) {
				cal_in_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
				cal_in_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
			}
			// mode-1：末端GDC矫正+缩放，out=cap；mode-0(与X5 mode-0一致)：GDC 1:1(旋转折入矫正bin)，
			// out=源分辨率(90/270交换)，主码流缩放由GDC后PYM2 group0完成(硬件契约：喂PYM的GDC不缩放)；
			// sub码流未启用时无PYM2，GDC即链路末端，保持缩放到cap(契约允许末端GDC缩放)
			int cal_out_width = cap_info_.width;
			int cal_out_height = cap_info_.height;
			if ((cap_info_.stream_mode_ != 1) && cap_info_.sub_stream_enable_) {
				cal_out_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
				cal_out_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
				if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
					cal_out_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
					cal_out_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
				}
			}
			auto gdc_bin = gen_gdc_bin(cal_in_width, cal_in_height,
					cal_out_width, cal_out_height, &cam_info_[0], &cal_cam_info, cap_info_.rotation_, cap_info_.cal_rotation_,
					0.0, rot_bin != nullptr);
			//auto gdc_bin = gen_gdc_bin_json("./gdc_bin_custom_config.json");
			if (gdc_bin) {
				if (rot_bin != nullptr) {
					// 旋转bin入缓存，供后续link的pipe复用(与矫正bin的复用逻辑一致)
					gdc_bin_buf_r_.push_back(rot_bin);
					pipe_contex->gdc_bin_r = rot_bin;
				}
				gdc_bin_buf_.push_back(gdc_bin);
				pipe_contex->gdc_bin = gdc_bin;
				cal_cam_info_.push_back(cal_cam_info);
			}
		} else if (!gdc_bin_buf_.empty()) {
			pipe_contex->gdc_bin = gdc_bin_buf_[0];
		}
	}
	// 无矫正bin的纯旋转回退：以源分辨率1:1旋转(PYM1直通层喂GDC_r，gen_gdc_bin_rotation内部
	// 90/270自动把输出交换为旋转后尺寸，out参数被覆盖)
	if ((cap_info_.rotation_ != 0) && (gdc_bin_buf_.size() == 0) && (gdc_bin_buf_r_.empty())) {
		int src_w = 0, src_h = 0;
		if (pipe_contex->sensor_config.pym_cfg != nullptr) {
			src_w = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_width;
			src_h = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_height;
		} else if (pipe_contex->sensor_config.isp_cfg != nullptr) {
			src_w = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
			src_h = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
		}
		std::shared_ptr<GdcBinBuf_ST> gdc_bin = (src_w > 0 && src_h > 0) ?
			gen_gdc_bin_rotation(src_w, src_h, cap_info_.width, cap_info_.height, cap_info_.rotation_) : nullptr;
		if (gdc_bin) {
			gdc_bin_buf_r_.push_back(gdc_bin);
			pipe_contex->gdc_bin_r = gdc_bin;
		}
	} else if (!gdc_bin_buf_r_.empty()) {
		pipe_contex->gdc_bin_r = gdc_bin_buf_r_[0];
	}

	return 0;
}


std::vector<std::shared_ptr<GdcBinBuf_ST>> HobotMipiCapIml::create_gsml_gdc_bin_stereo(
	 std::shared_ptr<pipe_contex_t> pipe_contex,
	 std::vector<sensor_msgs::msg::CameraInfo> *cam_pair)
{
	std::vector<std::shared_ptr<GdcBinBuf_ST>> result;

	if (pipe_contex == nullptr || cam_pair == nullptr || cam_pair->size() < 2)
	{
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
						 ">>> create_gsml_gdc_bin_stereo: invalid input, cam_pair=%p, size=%zu",
						 (void *)cam_pair, cam_pair ? cam_pair->size() : 0);
		return result;
	}

	if (!cap_info_.gdc_enable_)
	{
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
						">>> create_gsml_gdc_bin_stereo: gdc_enable=%d, skip",
						cap_info_.gdc_enable_);
		return result;
	}

	int src_width = 0, src_height = 0;

	if (pipe_contex->sensor_config.pym_cfg != nullptr)
	{
		src_width = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_width;
		src_height = pipe_contex->sensor_config.pym_cfg->chn_ctrl.src_in_height;
	}
	else if (pipe_contex->sensor_config.isp_cfg != nullptr)
	{
		src_width = pipe_contex->sensor_config.isp_cfg->isp_attr.size.width;
		src_height = pipe_contex->sensor_config.isp_cfg->isp_attr.size.height;
	}
	else if (pipe_contex->sensor_config.camera_config != nullptr)
	{
		src_width = pipe_contex->sensor_config.camera_config->width;
		src_height = pipe_contex->sensor_config.camera_config->height;
	}
	else if (pipe_contex->sensor_config.vin_attr != nullptr)
	{
		src_width = pipe_contex->sensor_config.vin_attr->vin_ichn_attr.width;
		src_height = pipe_contex->sensor_config.vin_attr->vin_ichn_attr.height;
	}
	if (src_width <= 0 || src_height <= 0)
	{
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
						 ">>> create_gsml_gdc_bin_stereo: cannot determine src resolution!");
		return result;
	}
	RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
					">>> create_gsml_gdc_bin_stereo: src=%dx%d, dst=%dx%d, rotation=%.1f, cal_rotation=%.1f",
					src_width, src_height, cap_info_.width, cap_info_.height,
					cap_info_.rotation_, cap_info_.cal_rotation_);
	// stream_mode_==1且需旋转(场景5)：先在源分辨率生成纯旋转bin(GDC_r)，矫正bin以pre_rotation
	// 作用于已旋转图像；mode-0的旋转折入矫正bin(不生成GDC_r)；矫正bin失败时旋转bin不生效，
	// 由调用方走源分辨率回退重新生成
	std::shared_ptr<GdcBinBuf_ST> rot_bin = nullptr;
	if ((cap_info_.stream_mode_ == 1) && (cap_info_.rotation_ != 0)) {
		rot_bin = gen_gdc_bin_rotation(src_width, src_height, src_width, src_height, cap_info_.rotation_);
	}
	int cal_in_width = src_width;
	int cal_in_height = src_height;
	if ((rot_bin != nullptr) && ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0))) {
		cal_in_width = src_height;
		cal_in_height = src_width;
	}
	// mode-1：末端GDC矫正+缩放，out=cap；mode-0(与X5 mode-0一致)：GDC 1:1(旋转折入矫正bin)，
	// out=源分辨率(90/270交换)，主码流缩放由GDC后PYM2 group0完成(硬件契约：喂PYM的GDC不缩放)；
	// sub码流未启用时无PYM2，GDC即链路末端，保持缩放到cap(契约允许末端GDC缩放)
	int cal_out_width = cap_info_.width;
	int cal_out_height = cap_info_.height;
	if ((cap_info_.stream_mode_ != 1) && cap_info_.sub_stream_enable_) {
		cal_out_width = src_width;
		cal_out_height = src_height;
		if ((cap_info_.rotation_ == 90.0) || (cap_info_.rotation_ == 270.0)) {
			cal_out_width = src_height;
			cal_out_height = src_width;
		}
	}
	std::vector<sensor_msgs::msg::CameraInfo> cal_pair;
	auto gdc_bins = gen_gdc_bin_stereo(
		 cal_in_width, cal_in_height,
		 cal_out_width, cal_out_height,
		 *cam_pair,
		 cal_pair,
		 cap_info_.rotation_,
		 cap_info_.cal_rotation_,
		 0.0,
		 rot_bin != nullptr);
	if (gdc_bins.size() == 2)
	{
		for (auto &ci : cal_pair)
		{
			cal_cam_info_.push_back(ci);
		}
		if (rot_bin != nullptr) {
			// 场景5：双GDC链的旋转bin(源分辨率纯旋转)，由调用方传播到同link的第二路pipe
			pipe_contex->gdc_bin_r = rot_bin;
		}
		result = gdc_bins;
	}
	else
	{
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
						">>> create_gsml_gdc_bin_stereo: gen returned %zu bins (expected 2)",
						gdc_bins.size());
	}

	return result;
}

void HobotMipiCapIml::deserial_config_update(deserial_config_t *deserial, const camera_config_t *camera_config, int link_port) {
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
	return;
}

}  // namespace mipi_cam
