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

#include "hobot_mipi_node.hpp"
#include "hobot_mipi_eval_log.hpp"
using mipi_cam::eval::EVAL_EV_IMU_ERROR;
using mipi_cam::eval::EVAL_EV_PUB_OUT;

#include <sstream>
#include <stdarg.h>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include "rcpputils/env.hpp"
#include "rcutils/env.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"

#define PUB_BUF_NUM 5
namespace mipi_cam {

MipiCamNode::MipiCamNode(const rclcpp::NodeOptions& node_options)
    : m_bIsInit(0),
      Node("mipi_cam", node_options),
      camera_calibration_info_(new sensor_msgs::msg::CameraInfo()) {
  nodePare_ = std::make_shared<struct NodePara>();

  std::string tros_distro = std::string(std::getenv("TROS_DISTRO")? std::getenv("TROS_DISTRO") : "");
  nodePare_->config_path_ = "/opt/tros/" + tros_distro + "/lib/mipi_cam/config/";
  nodePare_->video_device_name_ = "";
  nodePare_->channel_ = 0;
  nodePare_->channel2_ = 2;
  nodePare_->camera_info_url_ = "";
  nodePare_->camera_calibration_file_path_ = "";
  nodePare_->out_format_name_ = "bgr8"; //nv12, bgr8;
  nodePare_->gdc_bin_file_ = "";
  nodePare_->image_width_ = 1088;
  nodePare_->image_height_ = 1280;
  nodePare_->sub_image_width_ = 960;
  nodePare_->sub_image_height_ = 540;
  nodePare_->framerate_ = 30;
  nodePare_->sub_framerate_ = 30;
  nodePare_->rotation_ = 0.0;
  nodePare_->cal_rotation_ = 0.0;
  nodePare_->device_mode_ = "single"; //single, dual;
  nodePare_->dual_combine_ = 0;
  nodePare_->lpwm_enable_ = false;
  nodePare_->gdc_enable_ = true;
  nodePare_->frame_ts_type_ = "sensor"; //sensor,realtime;
  nodePare_->link_type_ = 0; 
  nodePare_->link_port_ = 0; 
  nodePare_->gsml_cfg_file_ = ""; //sensor,realtime;
  nodePare_->cal_alpha_ = 0.0; 
  nodePare_->stream_mode_ = 0; //0: slave stream can gdc; 1: slave stream can't gdc.
  nodePare_->sub_stream_enable_ = false;
  frame_id_ = "camera_link";
  io_method_name_ = "ros"; //shared_mem, ros;
  double framerate = 30.0;
  double sub_framerate = 30.0;
  imu_type_ = "";

  this->declare_parameter<std::string>("frame_id", frame_id_);
  this->declare_parameter<std::string>("io_method", io_method_name_);
  this->declare_parameter<std::string>("config_path", nodePare_->config_path_);
  this->declare_parameter<std::string>("video_device", nodePare_->video_device_name_);
  this->declare_parameter<int>("channel", nodePare_->channel_);
  this->declare_parameter<int>("channel2", nodePare_->channel2_);
  this->declare_parameter<std::string>("camera_info_url", nodePare_->camera_info_url_);
  this->declare_parameter<std::string>("camera_calibration_file_path", nodePare_->camera_calibration_file_path_);
  this->declare_parameter<std::string>("out_format", nodePare_->out_format_name_);
  this->declare_parameter<std::string>("gdc_bin_file", nodePare_->gdc_bin_file_);
  this->declare_parameter<int>("image_width", nodePare_->image_width_);
  this->declare_parameter<int>("image_height", nodePare_->image_height_);
  this->declare_parameter<int>("sub_image_width", nodePare_->sub_image_width_);
  this->declare_parameter<int>("sub_image_height", nodePare_->sub_image_height_);
  this->declare_parameter<double>("framerate", framerate);
  this->declare_parameter<double>("sub_framerate", sub_framerate);
  this->declare_parameter<double>("rotation", nodePare_->rotation_);
  this->declare_parameter<double>("cal_rotation", nodePare_->cal_rotation_);
  this->declare_parameter<std::string>("device_mode", nodePare_->device_mode_);
  this->declare_parameter<int>("dual_combine", nodePare_->dual_combine_);
  this->declare_parameter<bool>("lpwm_enable", nodePare_->lpwm_enable_);
  this->declare_parameter<bool>("gdc_enable", nodePare_->gdc_enable_);
  this->declare_parameter<std::string>("frame_ts_type", nodePare_->frame_ts_type_);
  this->declare_parameter<int>("link_type", nodePare_->link_type_); // 0:表示mipi接口，1：表示解串器接口。
  this->declare_parameter<int>("link_port", nodePare_->link_port_);
  this->declare_parameter<std::string>("gsml_cfg_file", nodePare_->gsml_cfg_file_);
  this->declare_parameter<std::string>("imu_type", imu_type_);
  this->declare_parameter<double>("cal_alpha", nodePare_->cal_alpha_);
  this->declare_parameter<int>("stream_mode", nodePare_->stream_mode_);
  this->declare_parameter<bool>("sub_stream_enable", nodePare_->sub_stream_enable_);
  this->declare_parameter<std::string>("imu_calib_file", "config/imu_calibration.yaml");
  // 评测埋点开关（默认关闭，见 hobot_mipi_eval_log.hpp 输出契约）
  this->declare_parameter<bool>("eval_log_enable", false);
  this->declare_parameter<std::string>("eval_log_events", "all");

  this->get_parameter<std::string>("frame_id", frame_id_);
  this->get_parameter<std::string>("io_method", io_method_name_);
  this->get_parameter<std::string>("config_path", nodePare_->config_path_);
  this->get_parameter<std::string>("video_device", nodePare_->video_device_name_);
  this->get_parameter<int>("channel", nodePare_->channel_);
  this->get_parameter<int>("channel2", nodePare_->channel2_);
  this->get_parameter<std::string>("camera_info_url", nodePare_->camera_info_url_);
  this->get_parameter<std::string>("camera_calibration_file_path", nodePare_->camera_calibration_file_path_);
  this->get_parameter<std::string>("out_format", nodePare_->out_format_name_);
  this->get_parameter<std::string>("gdc_bin_file", nodePare_->gdc_bin_file_);
  this->get_parameter<int>("image_width", nodePare_->image_width_);
  this->get_parameter<int>("image_height", nodePare_->image_height_);
  this->get_parameter<int>("sub_image_width", nodePare_->sub_image_width_);
  this->get_parameter<int>("sub_image_height", nodePare_->sub_image_height_);
  this->get_parameter<double>("framerate", framerate);
  this->get_parameter<double>("sub_framerate", sub_framerate);
  this->get_parameter<double>("rotation", nodePare_->rotation_);
  this->get_parameter<double>("cal_rotation", nodePare_->cal_rotation_);
  this->get_parameter<std::string>("device_mode", nodePare_->device_mode_);
  this->get_parameter<int>("dual_combine", nodePare_->dual_combine_);
  this->get_parameter<bool>("lpwm_enable", nodePare_->lpwm_enable_);
  this->get_parameter<bool>("gdc_enable", nodePare_->gdc_enable_);
  this->get_parameter<std::string>("frame_ts_type", nodePare_->frame_ts_type_);
  this->get_parameter<int>("link_type", nodePare_->link_type_);
  this->get_parameter<int>("link_port", nodePare_->link_port_);
  this->get_parameter<std::string>("gsml_cfg_file", nodePare_->gsml_cfg_file_);
  this->get_parameter<std::string>("imu_type", imu_type_);
  this->get_parameter<double>("cal_alpha", nodePare_->cal_alpha_);
  this->get_parameter<int>("stream_mode", nodePare_->stream_mode_);
  this->get_parameter<bool>("sub_stream_enable", nodePare_->sub_stream_enable_);

  nodePare_->framerate_ = static_cast<int>(framerate);
  nodePare_->sub_framerate_ = static_cast<int>(sub_framerate);

  this->get_parameter<std::string>("imu_calib_file", imu_calib_file_path_);

  // 配置评测埋点单例（平台后端经 EVAL_LOG 宏读取，零侵入：默认关闭）
  mipi_cam::eval::EvalLog::instance().configure(
      this->get_parameter("eval_log_enable").as_bool(),
      mipi_cam::eval::eval_event_mask_from_names(
          this->get_parameter("eval_log_events").as_string()));

  nodePare_->sync_awb_ = this->declare_parameter<bool>("sync_awb", false);
  nodePare_->sync_ccm_ = this->declare_parameter<bool>("sync_ccm", false);
  nodePare_->sync_ae_ = this->declare_parameter<bool>("sync_ae", false);
  nodePare_->print_isp_log_ = this->declare_parameter<bool>("print_isp_log", false);

  RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
              "\n node params:" \
    "\n config_path: %s" \
    "\n video_device_name: %s" \
    "\n channel: %d" \
    "\n channel2: %d" \
    "\n camera_info_url: %s" \
    "\n camera_calibration_file_path: %s" \
    "\n              out_format_name: %s" \
    "\n                 gdc_bin_file: %s" \
    "\n                  image_width: %d" \
    "\n                 image_height: %d" \
    "\n               sub_image_width: %d" \
    "\n              sub_image_height: %d" \
    "\n              sub_stream_enable: %s" \
    "\n                    framerate: %d" \
    "\n                    sub_framerate: %d" \
    "\n                     rotation: %f" \
    "\n                     cal_rotation: %f" \
    "\n                  device_mode: %s" \
    "\n                 dual_combine: %d" \
    "\n                  lpwm_enable: %s" \
    "\n                   gdc_enable: %s" \
    "\n                frame_ts_type: %s" \
    "\n                     frame_id: %s" \
    "\n                    link_type: %d" \
    "\n                    link_port: %d" \
    "\n               gsml_cfg_file: %s" \
    "\n               io_method_name: %s" \
    "\n                    cal_alpha: %.3f" \
    "\n                    sync_awb: %d" \
    "\n                    sync_ccm: %d" \
    "\n                    sync_ae: %d" \
    "\n                    print_ips_log: %d",
              nodePare_->config_path_.c_str(),
              nodePare_->video_device_name_.c_str(),
              nodePare_->channel_,
              nodePare_->channel2_,
              nodePare_->camera_info_url_.c_str(),
              nodePare_->camera_calibration_file_path_.c_str(),
              nodePare_->out_format_name_.c_str(),
              nodePare_->gdc_bin_file_.c_str(),
              nodePare_->image_width_,
              nodePare_->image_height_,
              nodePare_->sub_image_width_,
              nodePare_->sub_image_height_,
              (nodePare_->sub_stream_enable_ ? "true" : "false"),
              nodePare_->framerate_,
              nodePare_->sub_framerate_,
              nodePare_->rotation_,
              nodePare_->cal_rotation_,
              nodePare_->device_mode_.c_str(),
              nodePare_->dual_combine_,
              (nodePare_->lpwm_enable_ ? "true" : "false"),
              (nodePare_->gdc_enable_ ? "true" : "false"),
              nodePare_->frame_ts_type_.c_str(),
              frame_id_.c_str(),
              nodePare_->link_type_,
              nodePare_->link_port_,
              nodePare_->gsml_cfg_file_.c_str(),
              io_method_name_.c_str(),
              nodePare_->cal_alpha_,
              nodePare_->sync_awb_,
              nodePare_->sync_ccm_,
              nodePare_->sync_ae_,
              nodePare_->print_isp_log_
  );

  init();
  std::cout << std::endl;
}

MipiCamNode::~MipiCamNode() {
  RCLCPP_WARN(rclcpp::get_logger("mipi_node"), "shutting down");
  for (auto timer : timer_) {
    timer->join();
  }
  timer_.clear();
  if (mipiCam_ptr_) {
    mipiCam_ptr_->stop();
    mipiCam_ptr_->deInit();
    RCLCPP_WARN(rclcpp::get_logger("mipi_node"), "shutting down end11111");
    //mipiCam_ptr_ = nullptr;
    RCLCPP_WARN(rclcpp::get_logger("mipi_node"), "shutting down end");
  }
  for (auto pub : Pub_info_) {
    pub->image_pub_.reset();
    pub->info_pub_.reset();
    pub->info_pub2_.reset();
  }
  for (auto pub : Pub_hbmem_info_) {
    pub->publisher_hbmem_.reset();
    pub->info_pub_.reset();
    pub->info_pub2_.reset();
  }

  is_imu_running_ = false;
  for (auto timer : imu_timer_) {
    timer->join();
  }
  imu_timer_.clear();
  if (imu_manager_) {
    imu_manager_->release_sensor();
  }
  std::cout << std::endl;
}

void MipiCamNode::init() {
  if (m_bIsInit) return;
  mipiCam_ptr_ = MipiCam::create_mipicam();
  if (!mipiCam_ptr_ || mipiCam_ptr_->init(nodePare_)) {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_node"),
                      "[%s]->mipinode init failure.\n",
                      __func__);
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("mipi_node"),
      "[MipiCamNode::%s]->Initing '%s' at %dx%d via %s at %i FPS",
      __func__,
      nodePare_->config_path_.c_str(),
      nodePare_->image_width_,
      nodePare_->image_height_,
      io_method_name_.c_str(),
      nodePare_->framerate_);

  if (io_method_name_.compare("ros") == 0) {
    if (nodePare_->device_mode_.compare("dual") == 0) {
      if (nodePare_->dual_combine_ == 1) {
        //Pub_info_.resize(3);
        auto pub_info1 = std::make_shared<Publisher_info>();
        auto pub_info2 = std::make_shared<Publisher_info>();
        auto pub_info3 = std::make_shared<Publisher_info>();
        init_DualCalibration(pub_info1.get(), pub_info2.get(), "image_left_raw/camera_info", "image_right_raw/camera_info", nodePare_->camera_calibration_file_path_);
        init_DualCalibration(pub_info3.get(), "image_combine_raw/left/camera_info", "image_combine_raw/right/camera_info", nodePare_->camera_calibration_file_path_);
        init_publisher(pub_info1, "image_left_raw", "left", frame_id_);
        init_publisher(pub_info2, "image_right_raw", "right", frame_id_);
        init_publisher(pub_info3, "image_combine_raw", "combine", frame_id_);
        Pub_info_.push_back(pub_info1);
        Pub_info_.push_back(pub_info2);
        Pub_info_.push_back(pub_info3);
        if (nodePare_->sub_stream_enable_) {
          pub_info1 = std::make_shared<Publisher_info>();
          pub_info2 = std::make_shared<Publisher_info>();
          pub_info3 = std::make_shared<Publisher_info>();
          init_DualCalibration_Sub(pub_info1.get(), pub_info2.get(), "sub_image_left_raw/camera_info", "sub_image_right_raw/camera_info", nodePare_->camera_calibration_file_path_);
          init_DualCalibration_Sub(pub_info3.get(), "sub_image_combine_raw/left/camera_info", "sub_image_combine_raw/right/camera_info", nodePare_->camera_calibration_file_path_);
          init_publisher(pub_info1, "sub_image_left_raw", "sub_left", frame_id_);
          init_publisher(pub_info2, "sub_image_right_raw", "sub_right", frame_id_);
          init_publisher(pub_info3, "sub_image_combine_raw", "sub_combine", frame_id_);
          Pub_info_.push_back(pub_info1);
          Pub_info_.push_back(pub_info2);
          Pub_info_.push_back(pub_info3);
        }
      } else if (nodePare_->dual_combine_ == 2) {
        //Pub_info_.resize(1);
        auto pub_info1 = std::make_shared<Publisher_info>();
        init_DualCalibration(pub_info1.get(), "image_combine_raw/left/camera_info", "image_combine_raw/right/camera_info", nodePare_->camera_calibration_file_path_);
        init_publisher(pub_info1, "image_combine_raw", "combine", frame_id_);
        Pub_info_.push_back(pub_info1);
        if (nodePare_->sub_stream_enable_) {
          pub_info1 = std::make_shared<Publisher_info>();
          init_DualCalibration_Sub(pub_info1.get(), "sub_image_combine_raw/left/camera_info", "sub_image_combine_raw/right/camera_info", nodePare_->camera_calibration_file_path_);
          init_publisher(pub_info1, "sub_image_combine_raw", "sub_combine", frame_id_);
          Pub_info_.push_back(pub_info1);
        }
      } else {
        //Pub_info_.resize(2);
        auto pub_info1 = std::make_shared<Publisher_info>();
        auto pub_info2 = std::make_shared<Publisher_info>();
        init_DualCalibration(pub_info1.get(), pub_info2.get(), "image_left_raw/camera_info", "image_right_raw/camera_info", nodePare_->camera_calibration_file_path_);
        init_publisher(pub_info1, "image_left_raw", "left", frame_id_);
        init_publisher(pub_info2, "image_right_raw", "right", frame_id_);
        Pub_info_.push_back(pub_info1);
        Pub_info_.push_back(pub_info2);
        if (nodePare_->sub_stream_enable_) {
          auto pub_info1 = std::make_shared<Publisher_info>();
          auto pub_info2 = std::make_shared<Publisher_info>();
          init_DualCalibration_Sub(pub_info1.get(), pub_info2.get(), "sub_image_left_raw/camera_info", "sub_image_right_raw/camera_info", nodePare_->camera_calibration_file_path_);
          init_publisher(pub_info1, "sub_image_left_raw", "sub_left", frame_id_);
          init_publisher(pub_info2, "sub_image_right_raw", "sub_right", frame_id_);
          Pub_info_.push_back(pub_info1);
          Pub_info_.push_back(pub_info2);
        }
      }
    } else if (nodePare_->device_mode_.compare("multi") == 0) {
      auto pub_info1 = std::make_shared<Publisher_info>();
      init_publisher(pub_info1, "image_combine_raw", "combine", frame_id_);
      Pub_info_.push_back(pub_info1);
    } else if ((nodePare_->device_mode_.compare("single") == 0) ||
        (nodePare_->device_mode_.compare("") == 0)) {
      auto pub_info1 = std::make_shared<Publisher_info>();
      init_Calibration(pub_info1.get(), "image_raw/camera_info", nodePare_->camera_calibration_file_path_);
      init_publisher(pub_info1, "image_raw", "single", frame_id_);
      Pub_info_.push_back(pub_info1);
      if (nodePare_->sub_stream_enable_) {
        pub_info1 = std::make_shared<Publisher_info>();
        init_Calibration(pub_info1.get(), "sub_image_raw/camera_info", nodePare_->camera_calibration_file_path_);
        init_publisher(pub_info1, "sub_image_raw", "sub_single", frame_id_);
        Pub_info_.push_back(pub_info1);
      }
    } else {
      return;
    }
  } else if (io_method_name_.compare("shared_mem") == 0) {
    std::string ros_zerocopy_env = rcpputils::get_env_var("RMW_FASTRTPS_USE_QOS_FROM_XML");
    if (ros_zerocopy_env.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Launching with zero-copy, but env of `RMW_FASTRTPS_USE_QOS_FROM_XML` is not set. "
                              << "Transporting data without zero-copy!");
    } else {
      if ("1" == ros_zerocopy_env) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Enabling zero-copy");
      } else {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "env of `RMW_FASTRTPS_USE_QOS_FROM_XML` is [" << ros_zerocopy_env
                                                                          << "], which should be set to 1. "
                                                                          << "Data transporting without zero-copy!");
      }
    }
    if (nodePare_->device_mode_.compare("dual") == 0) {

      if (nodePare_->dual_combine_ == 1) {
        auto pub_info1 = std::make_shared<Publisher_hbmem_info>();
        auto pub_info2 = std::make_shared<Publisher_hbmem_info>();
        auto pub_info3 = std::make_shared<Publisher_hbmem_info>();
        init_DualCalibration(pub_info1.get(), pub_info2.get(), "hbmem_left_img/camera_info", "hbmem_right_img/camera_info", nodePare_->camera_calibration_file_path_);
        init_DualCalibration(pub_info3.get(), "hbmem_combine_img/left/camera_info", "hbmem_combine_img/right/camera_info", nodePare_->camera_calibration_file_path_);
        init_publisher_hbmem(pub_info1, "hbmem_left_img", "left");
        init_publisher_hbmem(pub_info2, "hbmem_right_img", "right");
        init_publisher_hbmem(pub_info3, "hbmem_combine_img", "combine");
        Pub_hbmem_info_.push_back(pub_info1);
        Pub_hbmem_info_.push_back(pub_info2);
        Pub_hbmem_info_.push_back(pub_info3);
        if (nodePare_->sub_stream_enable_) {
          pub_info1 = std::make_shared<Publisher_hbmem_info>();
          pub_info2 = std::make_shared<Publisher_hbmem_info>();
          pub_info3 = std::make_shared<Publisher_hbmem_info>();
          init_DualCalibration(pub_info1.get(), pub_info2.get(), "sub_hbmem_left_img/camera_info", "sub_hbmem_right_img/camera_info", nodePare_->camera_calibration_file_path_);
          init_DualCalibration(pub_info3.get(), "sub_hbmem_combine_img/left/camera_info", "sub_hbmem_combine_img/right/camera_info", nodePare_->camera_calibration_file_path_);
          init_publisher_hbmem(pub_info1, "sub_hbmem_left_img", "sub_left");
          init_publisher_hbmem(pub_info2, "sub_hbmem_right_img", "sub_right");
          init_publisher_hbmem(pub_info3, "sub_hbmem_combine_img", "sub_combine");
          Pub_hbmem_info_.push_back(pub_info1);
          Pub_hbmem_info_.push_back(pub_info2);
          Pub_hbmem_info_.push_back(pub_info3);
        }
      } else if (nodePare_->dual_combine_ == 2) {
        auto pub_info1 = std::make_shared<Publisher_hbmem_info>();
        init_DualCalibration(pub_info1.get(), "hbmem_combine_img/left/camera_info", "hbmem_combine_img/right/camera_info", nodePare_->camera_calibration_file_path_);
        init_publisher_hbmem(pub_info1, "hbmem_combine_img", "combine");
        Pub_hbmem_info_.push_back(pub_info1);
        if (nodePare_->sub_stream_enable_) {
          pub_info1 = std::make_shared<Publisher_hbmem_info>();
          init_DualCalibration(pub_info1.get(), "sub_hbmem_combine_img/left/camera_info", "sub_hbmem_combine_img/right/camera_info", nodePare_->camera_calibration_file_path_);
          init_publisher_hbmem(pub_info1, "sub_hbmem_combine_img", "sub_combine");
          Pub_hbmem_info_.push_back(pub_info1);
        }
      } else {
        auto pub_info1 = std::make_shared<Publisher_hbmem_info>();
        auto pub_info2 = std::make_shared<Publisher_hbmem_info>();
        init_DualCalibration(pub_info1.get(), pub_info2.get(), "hbmem_left_img/camera_info", "hbmem_right_img/camera_info", nodePare_->camera_calibration_file_path_);
        init_publisher_hbmem(pub_info1, "hbmem_left_img", "left");
        init_publisher_hbmem(pub_info2, "hbmem_right_img", "right");
        Pub_hbmem_info_.push_back(pub_info1);
        Pub_hbmem_info_.push_back(pub_info2);
        if (nodePare_->sub_stream_enable_) {
          pub_info1 = std::make_shared<Publisher_hbmem_info>();
          pub_info2 = std::make_shared<Publisher_hbmem_info>();
          init_DualCalibration(pub_info1.get(), pub_info2.get(), "sub_hbmem_left_img/camera_info", "sub_hbmem_right_img/camera_info", nodePare_->camera_calibration_file_path_);
          init_publisher_hbmem(pub_info1, "sub_hbmem_left_img", "sub_left");
          init_publisher_hbmem(pub_info2, "sub_hbmem_right_img", "sub_right");
          Pub_hbmem_info_.push_back(pub_info1);
          Pub_hbmem_info_.push_back(pub_info2);
        }
      }
    } else if ((nodePare_->device_mode_.compare("single") == 0) ||
        (nodePare_->device_mode_.compare("") == 0)) {
      auto pub_info1 = std::make_shared<Publisher_hbmem_info>();
      init_Calibration(pub_info1.get(), "hbmem_img/camera_info", nodePare_->camera_calibration_file_path_);
      init_publisher_hbmem(pub_info1, "hbmem_img", "single");
      Pub_hbmem_info_.push_back(pub_info1);
      if (nodePare_->sub_stream_enable_) {
        pub_info1 = std::make_shared<Publisher_hbmem_info>();
        init_Calibration(pub_info1.get(), "sub_hbmem_img/camera_info", nodePare_->camera_calibration_file_path_);
        init_publisher_hbmem(pub_info1, "sub_hbmem_img", "sub_single");
        Pub_hbmem_info_.push_back(pub_info1);
      }
    } else {
      return;
    }

  } else {
    return;
  }

  // start the camera

  if (0 != mipiCam_ptr_->start()) {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_node"),
                      "mipi camera start failed!");
    rclcpp::shutdown();
    return;
  }

  const int period_ms = 1000.0 / nodePare_->framerate_;

  if (io_method_name_.compare("ros") == 0) {
    for (auto info : Pub_info_) {
      timer_.emplace_back(
          std::make_shared<std::thread>([this, info]() { while(rclcpp::ok()) {this->update(info);}})
      );
    }

  } else if (io_method_name_.compare("shared_mem") == 0) {
    for (auto info : Pub_hbmem_info_) {
      timer_.emplace_back(
          std::make_shared<std::thread>([this, info]() { while(rclcpp::ok()) {this->hbmemUpdate(info);}})
      );
    }
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_node"),
                     "starting timer " << period_ms);
  m_bIsInit = 1;

  imu_manager_ = std::make_shared<imu_sensor::ImuManager>();
  if (0 == imu_manager_->init_sensor(imu_type_)) {
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 10);

    if (imu_manager_->loadCalibrationFromEeprom())
    {
      RCLCPP_INFO(this->get_logger(),
                  "IMU calibration loaded from EEPROM");
      // 创建外参 topic publisher（transient_local: 后来的订阅者也能收到）
      auto qos = rclcpp::QoS(1).reliable().transient_local();
      pub_imu_extrinsic_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
          "/imu_extrinsic", 10);
      static_tf_broadcaster_ =
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "No IMU calibration in EEPROM, publishing raw IMU data");
    }
    is_imu_running_ = true;
    imu_timer_.emplace_back(std::make_shared<std::thread>([this]() { while(rclcpp::ok()) {this->read_imu_data();}}));
  } else if (!imu_type_.empty()) {
    // 评测埋点：imu_error（指定了 imu_type 但初始化失败）
    EVAL_LOG(EVAL_EV_IMU_ERROR, "err=imu_init_fail detail=type_%s", imu_type_.c_str());
  }

}

void MipiCamNode::init_publisher(std::shared_ptr<Publisher_info> Pub_info, std::string topic, std::string topic_type,
                                 std::string frame_id){
  Pub_info->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, PUB_BUF_NUM);
  Pub_info->frame_id = frame_id;
  Pub_info->topic_type = topic_type;
  Pub_info->time_start_ = std::chrono::system_clock::now();
  setPubRateLimit(Pub_info.get(), topic_type, topic);
}

void MipiCamNode::init_publisher_hbmem(std::shared_ptr<Publisher_hbmem_info>  Pub_info, std::string topic, std::string topic_type){
  Pub_info->publisher_hbmem_ = this->create_publisher<hbm_img_msgs::msg::HbmMsg1080P>(topic, rclcpp::SensorDataQoS());
  Pub_info->topic_type = topic_type;
  Pub_info->time_start_ = std::chrono::system_clock::now();
  setPubRateLimit(Pub_info.get(), topic_type, topic);
}

// 主/辅码流发布帧率设置：底层VIN以max(fps, sub_fps)统一出帧，
// 主码流topic(single/left/right/combine)按framerate发布，
// 辅码流topic(sub_*)按sub_framerate发布
void MipiCamNode::setPubRateLimit(Publisher_info_base *pub_info,
                                  const std::string &topic_type,
                                  const std::string &topic) {
  const bool is_sub_stream = topic_type.rfind("sub_", 0) == 0;
  const int target_fps =
      is_sub_stream ? nodePare_->sub_framerate_ : nodePare_->framerate_;
  if (target_fps > 0) {
    pub_info->target_fps_ = target_fps;
    pub_info->pub_interval_ns_ =
        static_cast<int64_t>(1000000000.0 / target_fps);
    pub_info->next_pub_ts_ns_ = 0;
    pub_info->last_pub_ts_ns_ = 0;
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
              "publisher topic:%s, type:%s, pub rate limit:%d fps%s",
              topic.c_str(), topic_type.c_str(), pub_info->target_fps_,
              (pub_info->target_fps_ > 0 ? "" : " (no limit)"));
}

// 发布帧率抽帧(发布时刻表法)：维护next_pub_ts_ns_发布时刻，帧ts到达或越过
// 该时刻(留10%周期容差，兼容sensor时间戳抖动)即发布，并把时刻表推进一个目标
// 周期。丢帧按时刻表均匀分布，平均发布帧率精确等于目标帧率，
// 不会因整数倍跳帧而偏低(如底层8fps目标7fps时按125*6+250ms模式发布，恰为7fps)。
// 目标帧率不低于底层实际帧率时不丢帧。时刻表落后于当前帧(码流停顿后恢复)时
// 重新锚定，避免连续补发。返回true表示允许发布。
bool MipiCamNode::pubRateAllow(Publisher_info_base &pub_info, int64_t frame_ts_ns) {
  if (pub_info.target_fps_ <= 0 || pub_info.pub_interval_ns_ <= 0) {
    return true;  // 未限速
  }
  const int64_t interval = pub_info.pub_interval_ns_;
  const int64_t tol = interval / 10;
  // 首帧；或时间戳回退(码流重启/时钟切换)：发布并重新锚定时刻表
  if (pub_info.next_pub_ts_ns_ == 0 || frame_ts_ns < pub_info.last_pub_ts_ns_) {
    pub_info.last_pub_ts_ns_ = frame_ts_ns;
    pub_info.next_pub_ts_ns_ = frame_ts_ns + interval;
    return true;
  }
  if (frame_ts_ns + tol >= pub_info.next_pub_ts_ns_) {
    // 到达发布时刻：发布并推进时刻表
    pub_info.last_pub_ts_ns_ = frame_ts_ns;
    pub_info.next_pub_ts_ns_ += interval;
    if (pub_info.next_pub_ts_ns_ <= frame_ts_ns) {
      // 时刻表已落后(帧停顿后恢复等)：按当前帧重新锚定，避免连续补发
      pub_info.next_pub_ts_ns_ = frame_ts_ns + interval;
    }
    return true;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("mipi_node"),
               "drop frame ts:%ld ns, next pub ts:%ld ns (target %d fps)",
               static_cast<long>(frame_ts_ns),
               static_cast<long>(pub_info.next_pub_ts_ns_), pub_info.target_fps_);
  return false;
}


void MipiCamNode::init_Calibration(Publisher_info_base*  Pub_info,
                                   std::string info_topic, std::string info_file){
  Pub_info->camera_calibration_info_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  if (!mipiCam_ptr_ || !mipiCam_ptr_->getCamCalibration(*Pub_info->camera_calibration_info_, info_file)) {
    Pub_info->camera_calibration_info_ = nullptr;
    RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                "get camera calibration parameters failed");
    return;
  }
  Pub_info->info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      info_topic, PUB_BUF_NUM);
  return;
}

void MipiCamNode::init_DualCalibration(Publisher_info_base*  Pub_info,
                                       std::string info_topic, std::string info_topic2, std::string info_file){
  Pub_info->camera_calibration_info_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  Pub_info->camera_calibration_info2_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  if (!mipiCam_ptr_ || !mipiCam_ptr_->getDualCamCalibration(*Pub_info->camera_calibration_info_,
                                                            *Pub_info->camera_calibration_info2_, info_file)) {
    Pub_info->camera_calibration_info_ = nullptr;
    Pub_info->camera_calibration_info2_ = nullptr;
    RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                "get camera calibration parameters failed");
    return;
  }

  Pub_info->info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      info_topic, PUB_BUF_NUM);
  Pub_info->info_pub2_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      info_topic2, PUB_BUF_NUM);
  return;
}


void MipiCamNode::init_DualCalibration(Publisher_info_base*  Pub_info, Publisher_info_base*  Pub_info2,
                                       std::string info_topic, std::string info_topic2, std::string info_file){
  Pub_info->camera_calibration_info_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  Pub_info2->camera_calibration_info_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  if (!mipiCam_ptr_ || !mipiCam_ptr_->getDualCamCalibration(*Pub_info->camera_calibration_info_,
                                                            *Pub_info2->camera_calibration_info_, info_file)) {
    Pub_info->camera_calibration_info_ = nullptr;
    Pub_info2->camera_calibration_info_ = nullptr;
    RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                "get camera calibration parameters failed");
    return;
  }
  Pub_info->info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      info_topic, PUB_BUF_NUM);
  Pub_info2->info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      info_topic2, PUB_BUF_NUM);
  return;
}

void MipiCamNode::init_DualCalibration_Sub(Publisher_info_base *Pub_info,
                                           std::string info_topic, std::string info_topic2, std::string info_file)
{
  Pub_info->camera_calibration_info_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  Pub_info->camera_calibration_info2_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  if (!mipiCam_ptr_ || !mipiCam_ptr_->getDualCamCalibrationSub(*Pub_info->camera_calibration_info_,
                                                               *Pub_info->camera_calibration_info2_, info_file))
  {
    Pub_info->camera_calibration_info_ = nullptr;
    Pub_info->camera_calibration_info2_ = nullptr;
    RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                "get camera calibration parameters failed");
    return;
  }

  Pub_info->info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      info_topic, PUB_BUF_NUM);
  Pub_info->info_pub2_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      info_topic2, PUB_BUF_NUM);
  return;
}

void MipiCamNode::init_DualCalibration_Sub(Publisher_info_base *Pub_info, Publisher_info_base *Pub_info2,
                                           std::string info_topic, std::string info_topic2, std::string info_file)
{
  Pub_info->camera_calibration_info_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  Pub_info2->camera_calibration_info_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  if (!mipiCam_ptr_ || !mipiCam_ptr_->getDualCamCalibrationSub(*Pub_info->camera_calibration_info_,
                                                               *Pub_info2->camera_calibration_info_, info_file))
  {
    Pub_info->camera_calibration_info_ = nullptr;
    Pub_info2->camera_calibration_info_ = nullptr;
    RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                "get camera calibration parameters failed");
    return;
  }
  Pub_info->info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      info_topic, PUB_BUF_NUM);
  Pub_info2->info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      info_topic2, PUB_BUF_NUM);
  return;
}

void MipiCamNode::update(std::shared_ptr<Publisher_info> pub_info) {
  if (mipiCam_ptr_ && mipiCam_ptr_->isCapturing() && (pub_info->image_pub_->get_subscription_count() > 0)) {
    auto img = std::make_unique<sensor_msgs::msg::Image>(rosidl_runtime_cpp::MessageInitialization::SKIP);
    img->header.frame_id = pub_info->frame_id;
    if (!mipiCam_ptr_->getImage(img->header.stamp,
                                img->encoding,
                                img->height,
                                img->width,
                                img->step,
                                img->data,
                                pub_info->topic_type)) {
      auto time_after = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(time_after - pub_info->time_start_).count();
      if (interval > 3000) {
        RCLCPP_WARN(rclcpp::get_logger("mipi_node"), "grab failed");
      }
      return;
    }
    // 帧率控制：按目标帧率抽帧，未到发布间隔的帧直接丢弃
    const int64_t frame_ts_ns =
        static_cast<int64_t>(img->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(img->header.stamp.nanosec);
    if (!pubRateAllow(*pub_info, frame_ts_ns)) {
      return;
    }
    save_jpg(img->header.stamp,img->encoding,img->width,img->height,(void *)&img->data[0]);
    save_yuv(img->header.stamp, (void *)&img->data[0], img->data.size());
    if (pub_info->info_pub_ && pub_info->camera_calibration_info_) {
      sensor_msgs::msg::CameraInfo camera_calibration_info = *pub_info->camera_calibration_info_;
      camera_calibration_info.header.stamp = img->header.stamp;
      camera_calibration_info.header.frame_id = pub_info->frame_id;
      pub_info->info_pub_->publish(camera_calibration_info);
    }
    if (pub_info->info_pub2_ && pub_info->camera_calibration_info2_) {
      sensor_msgs::msg::CameraInfo camera_calibration_info = *pub_info->camera_calibration_info2_;
      camera_calibration_info.header.stamp = img->header.stamp;
      camera_calibration_info.header.frame_id = pub_info->frame_id;
      pub_info->info_pub2_->publish(camera_calibration_info);
    }

    auto channel = pub_info->topic_type;
    auto frame_id = pub_info->frame_id;
    auto ts = img->header.stamp;
    pub_info->image_pub_->publish(std::move(img));
    // 评测埋点：pub_out（发布完成时刻，帧 ts 见帧内时间戳）
    EVAL_LOG(EVAL_EV_PUB_OUT, "topic=%s ts_frame=%lld",
             pub_info->image_pub_->get_topic_name(),
             static_cast<long long>(ts.sec) * 1000000000LL + ts.nanosec);
    RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
      "Published image on channel: %s, frame_id: %s, ts: %d.%d, delay: %.3f sec",
      channel.c_str(), frame_id.c_str(),
      ts.sec, ts.nanosec, this->now().seconds() - rclcpp::Time(ts).seconds()
    );
  } else {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void MipiCamNode::hbmemUpdate(std::shared_ptr<Publisher_hbmem_info> pub_info) {
  if (mipiCam_ptr_ && mipiCam_ptr_->isCapturing() && (pub_info->publisher_hbmem_->get_subscription_count() > 0)) {
    auto loanedMsg = pub_info->publisher_hbmem_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto& msg = loanedMsg.get();
      if (!mipiCam_ptr_->getImageMem(msg.time_stamp,
                                     msg.encoding,
                                     msg.height,
                                     msg.width,
                                     msg.step,
                                     msg.data,
                                     msg.data_size,
                                     pub_info->topic_type)) {
        auto time_after = std::chrono::system_clock::now();
        auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(time_after - pub_info->time_start_).count();
        if (interval > 3000) {
          RCLCPP_WARN(rclcpp::get_logger("mipi_node"), "hbmemUpdate grab img failed");
        }
        return;
      }
      // 帧率控制：按目标帧率抽帧，未到发布间隔的帧丢弃
      // (loaned消息随作用域回收，不publish、不递增index)
      const int64_t frame_ts_ns =
          static_cast<int64_t>(msg.time_stamp.sec) * 1000000000LL +
          static_cast<int64_t>(msg.time_stamp.nanosec);
      if (!pubRateAllow(*pub_info, frame_ts_ns)) {
        return;
      }
      std::string encode(msg.encoding.begin(), msg.encoding.end());
      save_jpg(msg.time_stamp,encode,msg.width,msg.height,(void *)&msg.data);
      save_yuv(msg.time_stamp, (void *)&msg.data, msg.data_size);
      msg.index = pub_info->mSendIdx++;
      if (pub_info->info_pub_) {
        pub_info->camera_calibration_info_->header.stamp = msg.time_stamp;
        pub_info->info_pub_->publish(*pub_info->camera_calibration_info_);
      }
      if (pub_info->info_pub2_) {
        pub_info->camera_calibration_info2_->header.stamp = msg.time_stamp;
        pub_info->info_pub2_->publish(*pub_info->camera_calibration_info2_);
      }
      // 评测埋点数据先取（publish 后 loanedMsg 已移交）
      const uint32_t pub_seq = pub_info->mSendIdx - 1;
      const int64_t pub_ts_ns =
          static_cast<long long>(msg.time_stamp.sec) * 1000000000LL +
          static_cast<long long>(msg.time_stamp.nanosec);
      pub_info->publisher_hbmem_->publish(std::move(loanedMsg));
      // 评测埋点：pub_out（hbmem 通路，序号由订阅端校验连续性）
      EVAL_LOG(EVAL_EV_PUB_OUT, "topic=%s seq=%u ts_frame=%lld",
               pub_info->publisher_hbmem_->get_topic_name(), pub_seq, pub_ts_ns);

    } else {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "borrow_loaned_message failed");
    }
  } else {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

namespace {

// EEPROM / Kalibr store T_cam_imu in camera RDF (X right, Y down, Z forward).
// frame_id (camera_link) is FLU (X forward, Y left, Z up): p_flu = R_flu_rdf * p_rdf.
void fill_camera_flu_imu_transform(const ImuRTTimeInfo_d_ST &rt, geometry_msgs::msg::Transform &out) {
  const double tx = static_cast<double>(rt.tx);
  const double ty = static_cast<double>(rt.ty);
  const double tz = static_cast<double>(rt.tz);
  out.translation.x = tz;
  out.translation.y = -tx;
  out.translation.z = -ty;

  const double r11 = static_cast<double>(rt.r13);
  const double r12 = static_cast<double>(rt.r23);
  const double r13 = static_cast<double>(rt.r33);
  const double r21 = -static_cast<double>(rt.r11);
  const double r22 = -static_cast<double>(rt.r21);
  const double r23 = -static_cast<double>(rt.r31);
  const double r31 = -static_cast<double>(rt.r12);
  const double r32 = -static_cast<double>(rt.r22);
  const double r33 = -static_cast<double>(rt.r32);

  tf2::Matrix3x3 rot(r11, r12, r13, r21, r22, r23, r31, r32, r33);
  tf2::Quaternion q;
  rot.getRotation(q);
  q.normalize();
  out.rotation.x = q.x();
  out.rotation.y = q.y();
  out.rotation.z = q.z();
  out.rotation.w = q.w();
}

}  // namespace

void MipiCamNode::read_imu_data() {
  geometry_msgs::msg::TransformStamped tf_msg;
  bool publish_extrinsic = false;
  if (imu_manager_->hasValidCalibration())
  {
    const auto &rt = imu_manager_->getCalibParams().r_t_info_;

    // 构造 TransformStamped（TF 和 Topic 共用同一份数据）
    tf_msg.header.frame_id = frame_id_; // 与图像 frame_id 一致
    tf_msg.child_frame_id = "imu_link";

    fill_camera_flu_imu_transform(rt, tf_msg.transform);
    publish_extrinsic = true;
    //saveImuCalibration(imu_calib_file_path_);
  }
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "imu_link";
  imu_sensor::ImuData_T imu_data;

  while (is_imu_running_ ) {
    size_t subscriber_count = pub_imu_->get_subscription_count();
    if (subscriber_count > 0) {
      imu_manager_->read_sensor_data(&imu_data);
      // ---- IMU 时间戳 ----
      auto sec = static_cast<int32_t>(imu_data.timestamp / 1e9);
      auto nanosec = static_cast<uint32_t>(
          imu_data.timestamp - sec * 1e9);
      imu_msg.header.stamp.set__sec(sec);
      imu_msg.header.stamp.set__nanosec(nanosec);
      imu_msg.linear_acceleration.x = imu_data.ax;
      imu_msg.linear_acceleration.y = imu_data.ay;
      imu_msg.linear_acceleration.z = imu_data.az;
      imu_msg.angular_velocity.x = imu_data.gx;
      imu_msg.angular_velocity.y = imu_data.gy;
      imu_msg.angular_velocity.z = imu_data.gz;
      pub_imu_->publish(imu_msg);

      // ---- 发布外参（与 IMU 同频，时间戳同步） ----
      if (publish_extrinsic)
      {
        tf_msg.header.stamp.set__sec(sec);
        tf_msg.header.stamp.set__nanosec(nanosec);

        if (static_tf_broadcaster_)
        {
          static_tf_broadcaster_->sendTransform(tf_msg);
        }
        if (pub_imu_extrinsic_)
        {
          pub_imu_extrinsic_->publish(tf_msg);
        }
      }
    } else {
      usleep(1000*1000);
    }
    usleep(1*1000);
  }
}


void MipiCamNode::save_yuv(const builtin_interfaces::msg::Time stamp,
                           void *data, int data_size) {
  std::string yuv_path = "./yuv/";
  uint64_t time_stamp = (stamp.sec * 1000 + stamp.nanosec / 1000000);;
  if (access(yuv_path.c_str(), F_OK) == 0) {

    std::string yuv_file = "./yuv/" + std::to_string(time_stamp) + ".yuv";
    RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                "save yuv image: %s", yuv_file.c_str());
    std::ofstream out(yuv_file, std::ios::out|std::ios::binary);
    out.write(reinterpret_cast<char*>(data), data_size);
    out.close();
  }
}

void MipiCamNode::save_jpg(const builtin_interfaces::msg::Time stamp, std::string encode, int w, int h, void *data){
  std::string jpg_path = "./jpg/";
  uint64_t time_stamp = (stamp.sec * 1000 + stamp.nanosec / 1000000);;
  if (access(jpg_path.c_str(), F_OK) == 0) {

    std::string jpg_file = "./jpg/" + std::to_string(time_stamp) + ".jpg";
    RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                "save jpg image: %s", jpg_file.c_str());
    if (encode == "nv12") {
      cv::Mat src_mat(h * 1.5, w, CV_8UC1, data);
      cv::Mat img_bgr;
      cv::cvtColor(src_mat, img_bgr, cv::COLOR_YUV2BGR_NV12);
      cv::imwrite(jpg_file, img_bgr);
    } else if (encode == "bgr8") {
      cv::Mat img_bgr(h, w, CV_8UC3, data);
      cv::imwrite(jpg_file, img_bgr);
    }
  }
}

void MipiCamNode::saveImuCalibration(const std::string &file_path)
{
  // 确保目录存在
  std::string dir = file_path.substr(0, file_path.find_last_of('/'));
  if (!dir.empty())
  {
    mkdir(dir.c_str(), 0755);
  }
  std::ofstream ofs(file_path);
  if (!ofs.is_open())
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to open IMU calibration file for writing: %s",
                 file_path.c_str());
    return;
  }

  ofs << std::fixed << std::setprecision(9);

  const auto &imu_calib_params_ = imu_manager_->getCalibParams();
  const auto &acc_m = imu_calib_params_.acc_mislign_;
  const auto &gyro_m = imu_calib_params_.gyro_mislign_;
  const auto &acc_s = imu_calib_params_.acc_scale_;
  const auto &gyro_s = imu_calib_params_.gyro_scale_;
  const auto &acc_b = imu_calib_params_.acc_bias_;
  const auto &gyro_b = imu_calib_params_.gyro_bias_;
  const auto &acc_nw = imu_calib_params_.acc_n_w_;
  const auto &gyro_nw = imu_calib_params_.gyro_n_w_;
  const auto &rt = imu_calib_params_.r_t_info_;

  ofs << "# IMU Calibration Parameters" << std::endl;
  ofs << "# Read from camera module EEPROM" << std::endl;
  ofs << "# Generated by hobot_mipi_cam" << std::endl;
  ofs << std::endl;

  ofs << "accelerometer:" << std::endl;

  ofs << "  misalignment:" << std::endl;
  ofs << "    rows: 3" << std::endl;
  ofs << "    cols: 3" << std::endl;
  ofs << "    data: ["
      << acc_m.m00 << ", " << acc_m.m01 << ", " << acc_m.m02 << ", "
      << acc_m.m10 << ", " << acc_m.m11 << ", " << acc_m.m12 << ", "
      << acc_m.m20 << ", " << acc_m.m21 << ", " << acc_m.m22
      << "]" << std::endl;

  ofs << "  scale:" << std::endl;
  ofs << "    data: ["
      << acc_s.s0 << ", " << acc_s.s1 << ", " << acc_s.s2
      << "]" << std::endl;

  ofs << "  bias:" << std::endl;
  ofs << "    data: ["
      << acc_b.b0 << ", " << acc_b.b1 << ", " << acc_b.b2
      << "]" << std::endl;

  ofs << "  noise_density: " << acc_nw.n << std::endl;
  ofs << "  random_walk: " << acc_nw.w << std::endl;
  ofs << std::endl;

  ofs << "gyroscope:" << std::endl;

  ofs << "  misalignment:" << std::endl;
  ofs << "    rows: 3" << std::endl;
  ofs << "    cols: 3" << std::endl;
  ofs << "    data: ["
      << gyro_m.m00 << ", " << gyro_m.m01 << ", " << gyro_m.m02 << ", "
      << gyro_m.m10 << ", " << gyro_m.m11 << ", " << gyro_m.m12 << ", "
      << gyro_m.m20 << ", " << gyro_m.m21 << ", " << gyro_m.m22
      << "]" << std::endl;

  ofs << "  scale:" << std::endl;
  ofs << "    data: ["
      << gyro_s.s0 << ", " << gyro_s.s1 << ", " << gyro_s.s2
      << "]" << std::endl;

  ofs << "  bias:" << std::endl;
  ofs << "    data: ["
      << gyro_b.b0 << ", " << gyro_b.b1 << ", " << gyro_b.b2
      << "]" << std::endl;

  ofs << "  noise_density: " << gyro_nw.n << std::endl;
  ofs << "  random_walk: " << gyro_nw.w << std::endl;
  ofs << std::endl;

  ofs << "imu_camera_extrinsic:" << std::endl;

  ofs << "  rotation:" << std::endl;
  ofs << "    rows: 3" << std::endl;
  ofs << "    cols: 3" << std::endl;
  ofs << "    data: ["
      << rt.r11 << ", " << rt.r12 << ", " << rt.r13 << ", "
      << rt.r21 << ", " << rt.r22 << ", " << rt.r23 << ", "
      << rt.r31 << ", " << rt.r32 << ", " << rt.r33
      << "]" << std::endl;

  ofs << "  translation:" << std::endl;
  ofs << "    data: ["
      << rt.tx << ", " << rt.ty << ", " << rt.tz
      << "]" << std::endl;

  ofs << "  timeshift: " << rt.timeshift << std::endl;
  ofs << "  reproject_error: " << rt.reporject << std::endl;

  ofs.close();

  RCLCPP_INFO(this->get_logger(),
              "IMU calibration saved to: %s", file_path.c_str());
}

}  // namespace mipi_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mipi_cam::MipiCamNode)
