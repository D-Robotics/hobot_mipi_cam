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

  this->get_parameter<std::string>("imu_calib_file", imu_calib_file_path_);

  nodePare_->sync_awb_ = this->declare_parameter<bool>("sync_awb", false);
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
    "\n                    framerate: %d" \
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
              nodePare_->framerate_,
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
  }

}

void MipiCamNode::init_publisher(std::shared_ptr<Publisher_info> Pub_info, std::string topic, std::string topic_type,
                                 std::string frame_id){
  Pub_info->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, PUB_BUF_NUM);
  Pub_info->frame_id = frame_id;
  Pub_info->topic_type = topic_type;
  Pub_info->time_start_ = std::chrono::system_clock::now();
}

void MipiCamNode::init_publisher_hbmem(std::shared_ptr<Publisher_hbmem_info>  Pub_info, std::string topic, std::string topic_type){
  Pub_info->publisher_hbmem_ = this->create_publisher<hbm_img_msgs::msg::HbmMsg1080P>(topic, rclcpp::SensorDataQoS());
  Pub_info->topic_type = topic_type;
  Pub_info->time_start_ = std::chrono::system_clock::now();
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

    pub_info->image_pub_->publish(std::move(img));
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
      pub_info->publisher_hbmem_->publish(std::move(loanedMsg));

    } else {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "borrow_loaned_message failed");
    }
  } else {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void MipiCamNode::read_imu_data() {
  geometry_msgs::msg::TransformStamped tf_msg;
  bool publish_extrinsic = false;
  if (imu_manager_->hasValidCalibration())
  {
    const auto &rt = imu_manager_->getCalibParams().r_t_info_;

    // 构造 TransformStamped（TF 和 Topic 共用同一份数据）
    tf_msg.header.frame_id = frame_id_; // 与图像 frame_id 一致
    tf_msg.child_frame_id = "imu_link";

    tf_msg.transform.translation.x = static_cast<double>(rt.tx);
    tf_msg.transform.translation.y = static_cast<double>(rt.ty);
    tf_msg.transform.translation.z = static_cast<double>(rt.tz);

    // R 矩阵 → 四元数
    tf2::Matrix3x3 rot(
        rt.r11, rt.r12, rt.r13,
        rt.r21, rt.r22, rt.r23,
        rt.r31, rt.r32, rt.r33);
    tf2::Quaternion q;
    rot.getRotation(q);
    q.normalize();

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
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
