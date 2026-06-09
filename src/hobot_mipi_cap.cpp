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
#include "hobot_mipi_cap.hpp"

namespace mipi_cam {

void HobotMipiCap::free_gdc_cfg(void *buf_ptr) {
    #if defined(PLATFORM_S100) || defined(PLATFORM_S600)
    hbn_free_gdc_cfg((uint32_t *)buf_ptr);
    #else
    hbn_free_gdc_bin((uint32_t *)buf_ptr);
    #endif  
}

int32_t HobotMipiCap::gen_gdc_cfg(param_t *param, window_t *wnds, uint32_t wnd_num, void **cfg_buf, uint64_t *cfg_size) {
    #if defined(PLATFORM_S100) || defined(PLATFORM_S600)
    return hbn_gen_gdc_cfg(param, wnds, wnd_num, (void **)cfg_buf, cfg_size);
    #else
    return hbn_gen_gdc_bin(param, wnds, wnd_num, (uint32_t**)cfg_buf, cfg_size);
    #endif  
}

long long TOLERANCE = 52*1000*1000; // ms
void HobotMipiCap::sync_task() {
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
  
void HobotMipiCap::sub_sync_task() {
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

bool HobotMipiCap::isSynced(const std::vector<std::shared_ptr<VideoBuffer>> &frames, long long tolerance) {
    long long min_ts = (long long)frames[0]->timestamp;
    long long max_ts = (long long)frames[0]->timestamp;
    std::for_each(frames.begin(), frames.end(), [&](auto &frame) {
    min_ts = std::min(min_ts, (long long)frame->timestamp);
    max_ts = std::max(max_ts, (long long)frame->timestamp);
    });
    return (max_ts - min_ts) <= tolerance;
}

std::shared_ptr<GdcBinBuf_ST> HobotMipiCap::get_gdc_bin(std::string gdc_bin_file) {
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

std::vector<std::shared_ptr<GdcBinBuf_ST>> HobotMipiCap::gen_gdc_bin_stereo(int in_width, int in_height,int out_width, int out_height, 
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

	cv::Mat rvec;

	cv::Rodrigues(R_rl, rvec);

	RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_cap"),"===stetreo rvec==="  << rvec);

	double target_hfov = cal_alpha;
	double balance = 0.0;
	double fov_scale = 1.0;
	double hfov_l = 0.0, hfov_r = 0.0;
	//double alpha = 0;
	// TODO: Set alpha from config
	// cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height), R_rl, t_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 0.391 ,cv::Size(out_gdc_width, out_gdc_height));
	if (cam_info[0].distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
		if (target_hfov <= 0) {
			fov_scale = find_best_fov_scale(Kl, Dl, Kr, Dr, R_rl, t_rl, cv::Size(in_gdc_width, in_gdc_height), cv::Size(out_gdc_width, out_gdc_height));
		} else {
			// target_hfov > 0: 只关心逼近目标 FOV，不关心越界
			bool ok_ = computeFisheyeStereoParamsFromFOV(
				 target_hfov,
				 Kl, Dl, Kr, Dr, R_rl, t_rl,
				 cv::Size(in_gdc_width, in_gdc_height),
				 cv::Size(out_gdc_width, out_gdc_height),
				 balance, fov_scale, hfov_l, hfov_r);
		}
		RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_cap"), "best fov scale: " << fov_scale);
		RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_cap"), "final_balance: " << balance);
		RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_cap"), "hfov_l: " << hfov_l);
		RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_cap"), "target_hfov: " << target_hfov);
		cv::fisheye::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height), R_rl, t_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, cv::Size(out_gdc_width, out_gdc_height), 0.0, fov_scale);
		cv::fisheye::initUndistortRectifyMap(Kl, Dl, Rl, Pl, cv::Size(out_gdc_width, out_gdc_height), CV_32FC1, undistmap1l, undistmap2l);
		cv::fisheye::initUndistortRectifyMap(Kr, Dr, Rr, Pr, cv::Size(out_gdc_width, out_gdc_height), CV_32FC1, undistmap1r, undistmap2r);
	} else {
		double alpha = computeStereoAlphaFromFOV(
    		target_hfov,
    		Kl, Dl, Kr, Dr, R_rl, t_rl,
    		in_gdc_width, in_gdc_height,
    		out_gdc_width, out_gdc_height,
    		hfov_l, hfov_r);
		if (alpha <= 0) {
			alpha = 0;
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "Use default alpha=0.0 (target FOV invalid)");
		} else {
			// RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),
        	// 	"Auto compute alpha: %f\n , Left actual FOV: %f\n, Right actual FOV: %f\n ", alpha, actual_hfov_l, actual_hfov_r);
			RCLCPP_WARN(rclcpp::get_logger("mipi_cap"), "Auto compute alpha: %f", alpha);
			RCLCPP_WARN_STREAM(rclcpp::get_logger("mipi_cap"), "hfov_l: " << hfov_l);
		}
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
	void *bin_buf_ptr = nullptr;
	uint64_t bin_buf_size;
	int64_t alloc_flags = 0;
	int offset = 0;
	auto ret = gen_gdc_cfg(&gdc_param, &wnds, 1, (void**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"gen_gdc_cfg failed, ret = %d\n", ret);
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
        free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
	free_gdc_cfg(bin_buf_ptr);
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
	ret = gen_gdc_cfg(&gdc_param, &wnds, 1, (void**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"gen_gdc_cfg failed, ret = %d\n", ret);
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
        free_gdc_cfg(bin_buf_ptr);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}

	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        free_gdc_cfg(bin_buf_ptr);
		RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return gdc_bin_buf;
	}
	free_gdc_cfg(bin_buf_ptr);
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

std::pair<double, double> HobotMipiCap::calculatePinholeFOV(const cv::Mat& K_rect, int width, int hight) {
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

double HobotMipiCap::computeInitAlpha(double target_hfov, double fov_min, double fov_max) {
	if (std::abs(fov_max - fov_min) < 1e-6) {
        return 0.0;  // 如果FOV范围非常小，直接返回0
    }
	if (target_hfov <= fov_min - 1e-3) return 0.0;
    if (target_hfov >= fov_max + 1e-3) return 1.0;
    return (target_hfov - fov_min) / (fov_max - fov_min);
}

double HobotMipiCap::computeStereoAlphaFromFOV(
    double target_hfov,
    const cv::Mat& Kl, const cv::Mat& Dl,
    const cv::Mat& Kr, const cv::Mat& Dr,
    const cv::Mat& R_rl, const cv::Mat& t_rl,
    int in_gdc_width, int in_gdc_height,
    int out_gdc_width, int out_gdc_height,
    double& actual_hfov_l, double& actual_hfov_r) {

	// ------------ 步骤1：计算alpha=0时的双目FOV（FOV_min） ------------
    cv::Mat Rl0, Rr0, Pl0, Pr0, Q0;
    cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                      R_rl, t_rl, Rl0, Rr0, Pl0, Pr0, Q0,
                      cv::CALIB_ZERO_DISPARITY, 0.0, cv::Size(out_gdc_width, out_gdc_height));
    // 提取校正后内参（投影矩阵Pl/Pr的前3x3）
    cv::Mat K_l0 = Pl0(cv::Rect(0,0,3,3)).clone();
    cv::Mat K_r0 = Pr0(cv::Rect(0,0,3,3)).clone();
    // 计算alpha=0时的FOV
    auto [fov_l0, _] = calculatePinholeFOV(K_l0, out_gdc_width, out_gdc_height);
    auto [fov_r0, __] = calculatePinholeFOV(K_r0, out_gdc_width, out_gdc_height);

	// ------------ 步骤2：计算alpha=1时的双目FOV（FOV_max） ------------
    cv::Mat Rl1, Rr1, Pl1, Pr1, Q1;
    cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                      R_rl, t_rl, Rl1, Rr1, Pl1, Pr1, Q1,
                      cv::CALIB_ZERO_DISPARITY, 1.0, cv::Size(out_gdc_width, out_gdc_height));

    cv::Mat K_l1 = Pl1(cv::Rect(0,0,3,3)).clone();
    cv::Mat K_r1 = Pr1(cv::Rect(0,0,3,3)).clone();
    auto [fov_l1, ___] = calculatePinholeFOV(K_l1, out_gdc_width, out_gdc_height);
    auto [fov_r1, ____] = calculatePinholeFOV(K_r1, out_gdc_width, out_gdc_height);

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
        cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                          R_rl, t_rl, Rl, Rr, Pl, Pr, Q,
                          cv::CALIB_ZERO_DISPARITY, alpha, cv::Size(out_gdc_width, out_gdc_height));
        cv::Mat K_l = Pl(cv::Rect(0,0,3,3)).clone();
        cv::Mat K_r = Pr(cv::Rect(0,0,3,3)).clone();
        auto [hfov_l, _] = calculatePinholeFOV(K_l, out_gdc_width, out_gdc_height);
        auto [hfov_r, __] = calculatePinholeFOV(K_r, out_gdc_width, out_gdc_height);
        
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
    cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(in_gdc_width, in_gdc_height),
                      R_rl, t_rl, Rl_final, Rr_final, Pl_final, Pr_final, Q_final,
                      cv::CALIB_ZERO_DISPARITY, alpha, cv::Size(out_gdc_width, out_gdc_height));
    cv::Mat K_l_final = Pl_final(cv::Rect(0,0,3,3)).clone();
    cv::Mat K_r_final = Pr_final(cv::Rect(0,0,3,3)).clone();
    actual_hfov_l = calculatePinholeFOV(K_l_final, out_gdc_width, out_gdc_height).first;
    actual_hfov_r = calculatePinholeFOV(K_r_final, out_gdc_width, out_gdc_height).first;

    return alpha;										
}

double HobotMipiCap::find_best_fov_scale(const cv::Mat& Kl, const cv::Mat& Dl,
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

		if (invalid == 0)
		{
			best_scale = fov;
			break;
		}
	}

	return best_scale;
}

bool HobotMipiCap::computeFisheyeStereoParamsFromFOV(
	 double target_hfov,
	 const cv::Mat &Kl, const cv::Mat &Dl,
	 const cv::Mat &Kr, const cv::Mat &Dr,
	 const cv::Mat &R_rl, const cv::Mat &t_rl,
	 cv::Size in_size, cv::Size out_size,
	 double &out_balance, double &out_fov_scale,
	 double &actual_hfov_l, double &actual_hfov_r,
	 double tol,	// 默认 3.0°
	 int max_iter) // 默认 10
{
	constexpr double SCALE_LO = 0.05;
	constexpr double SCALE_HI = 2.0;

	out_balance = 0.0;
	out_fov_scale = 1.0;
	actual_hfov_l = 0.0;
	actual_hfov_r = 0.0;

	if (target_hfov <= 0.0 || in_size.area() <= 0 || out_size.area() <= 0)
	{
		RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
						"Invalid params: target_hfov=%.1f, in=%dx%d, out=%dx%d",
						target_hfov, in_size.width, in_size.height, out_size.width, out_size.height);
		return false;
	}

	double lo = SCALE_LO, hi = SCALE_HI;
	double best_diff = std::numeric_limits<double>::max();

	for (int i = 0; i < max_iter; ++i)
	{
		double mid = 0.5 * (lo + hi);

		// stereoRectify 拿到 Pl/Pr，从中提取 fx 算 HFOV
		cv::Mat Rl, Rr, Pl, Pr, Q;
		cv::fisheye::stereoRectify(
			 Kl, Dl, Kr, Dr, in_size, R_rl, t_rl,
			 Rl, Rr, Pl, Pr, Q,
			 cv::CALIB_ZERO_DISPARITY, out_size, 0.0, mid);

		double hl = 2.0 * std::atan2(out_size.width * 0.5, Pl.at<double>(0, 0)) * 180.0 / CV_PI;
		double hr = 2.0 * std::atan2(out_size.width * 0.5, Pr.at<double>(0, 0)) * 180.0 / CV_PI;
		double avg = 0.5 * (hl + hr);
		double diff = std::abs(avg - target_hfov);

		if (diff < best_diff)
		{
			best_diff = diff;
			out_fov_scale = mid;
			actual_hfov_l = hl;
			actual_hfov_r = hr;
		}

		if (diff < tol)
		{
			RCLCPP_INFO(rclcpp::get_logger("mipi_cap"),
							"Converged at iter %d: fov_scale=%.6f, HFOV=[%.2f°, %.2f°], target=%.2f°",
							i, out_fov_scale, hl, hr, target_hfov);
			return true;
		}

		// fov_scale ↑ → fx ↓ → HFOV ↑（单调递增）
		if (avg < target_hfov)
			lo = mid;
		else
			hi = mid;

		if (hi - lo < 1e-7)
			break;
	}

	RCLCPP_WARN(rclcpp::get_logger("mipi_cap"),
					"Not fully converged: fov_scale=%.6f, HFOV=[%.2f°, %.2f°], target=%.2f°, diff=%.2f°",
					out_fov_scale, actual_hfov_l, actual_hfov_r, target_hfov, best_diff);
	return false;
}

std::shared_ptr<GdcBinBuf_ST> HobotMipiCap::gen_gdc_bin(int in_width, int in_height,int out_width, int out_height,
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

	void *bin_buf_ptr = nullptr;
	uint64_t bin_buf_size;
	int64_t alloc_flags = 0;
	int offset = 0;

	auto ret = gen_gdc_cfg(&gdc_param, &wnds, 1, (void**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"gen_gdc_cfg failed, ret = %d\n", ret);
		return nullptr;
	}

    hb_mem_common_buf_t *bin_buf = new hb_mem_common_buf_t;
	memset(bin_buf, 0, sizeof(hb_mem_common_buf_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
	ret = hb_mem_alloc_com_buf(bin_buf_size, alloc_flags, bin_buf);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	free_gdc_cfg(bin_buf_ptr);
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


std::shared_ptr<GdcBinBuf_ST> HobotMipiCap::gen_gdc_bin_rotation(int gdc_width, int gdc_height,int out_width, int out_height,
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

	void *bin_buf_ptr = nullptr;
	uint64_t bin_buf_size;
	int64_t alloc_flags = 0;
	int offset = 0;

	auto ret = gen_gdc_cfg(&gdc_param, &wnds, 1, (void**)&bin_buf_ptr, &bin_buf_size);
	if (ret != 0 || bin_buf_ptr == nullptr) {
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"gen_gdc_cfg failed, ret = %d\n", ret);
		return nullptr;
	}

    hb_mem_common_buf_t *bin_buf = new hb_mem_common_buf_t;
	memset(bin_buf, 0, sizeof(hb_mem_common_buf_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
	ret = hb_mem_alloc_com_buf(bin_buf_size, alloc_flags, bin_buf);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	free_gdc_cfg(bin_buf_ptr);
	auto gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;
	return gdc_bin_ptr;
}

std::shared_ptr<GdcBinBuf_ST> HobotMipiCap::gen_gdc_bin_json(std::string file) {
	void *bin_buf_ptr = nullptr;
#if 0
	uint64_t bin_buf_size;
	int64_t alloc_flags = 0;
	int offset = 0;
	
	auto ret = hbn_gen_gdc_bin_json(file.c_str(), NULL, (void**)&bin_buf_ptr, &bin_buf_size);
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
        free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	memcpy(bin_buf->virt_addr, bin_buf_ptr, bin_buf_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, bin_buf_size);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
        free_gdc_cfg(bin_buf_ptr);
		RCLCPP_ERROR(rclcpp::get_logger("mipi_cap"),"hb_mem_flush_buf for bin failed, ret = %d\n", ret);
		return nullptr;
	}
	free_gdc_cfg(bin_buf_ptr);
	auto gdc_bin_ptr = std::make_shared<GdcBinBuf_ST>();
	gdc_bin_ptr->bin_buf = bin_buf;
	gdc_bin_ptr->bin_buf_size = bin_buf_size;
	return gdc_bin_ptr;
#endif
    return nullptr;
}

}  // namespace mipi_cam
