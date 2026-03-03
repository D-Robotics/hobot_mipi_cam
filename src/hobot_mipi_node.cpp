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
  nodePare_->image_width_ = 1920;
  nodePare_->image_height_ = 1080;
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
  nodePare_->cal_alpha_ = 0.0; 
  nodePare_->sub_stream_flag_ = true; 
  frame_id_ = "default_cam";
  frame_id2_ = "default_cam2";
  io_method_name_ = "ros"; //shared_mem, ros;
  double framerate = 30.0;

  this->declare_parameter<std::string>("frame_id", frame_id_);
  this->declare_parameter<std::string>("frame_id2", frame_id2_);
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
  this->declare_parameter<double>("cal_alpha", nodePare_->cal_alpha_);
  this->declare_parameter<std::string>("left_img_topic", left_img_topic_);
  this->declare_parameter<std::string>("right_img_topic", right_img_topic_);
  this->declare_parameter<std::string>("combine_img_topic", combine_img_topic_);
  this->declare_parameter<std::string>("left_cam_info_topic", left_cam_info_topic_);
  this->declare_parameter<std::string>("right_cam_info_topic", right_cam_info_topic_);

  this->get_parameter<std::string>("frame_id", frame_id_);
  this->get_parameter<std::string>("frame_id2", frame_id2_);
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
  this->get_parameter<double>("cal_alpha", nodePare_->cal_alpha_);
  this->get_parameter<std::string>("left_img_topic", left_img_topic_);
  this->get_parameter<std::string>("right_img_topic", right_img_topic_);
  this->get_parameter<std::string>("combine_img_topic", combine_img_topic_);
  this->get_parameter<std::string>("left_cam_info_topic", left_cam_info_topic_);
  this->get_parameter<std::string>("right_cam_info_topic", right_cam_info_topic_);

  nodePare_->framerate_ = static_cast<int>(framerate);

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
    "\n                  device_mode: %s" \
    "\n                 dual_combine: %d" \
    "\n                  lpwm_enable: %s" \
    "\n                   gdc_enable: %s" \
    "\n                frame_ts_type: %s" \
    "\n                     frame_id: %s" \
    "\n                     frame_id2: %s" \
    "\n                    link_type: %d" \
    "\n                    link_port: %d" \
    "\n               io_method_name: %s" \
    "\n               left_img_topic: %s" \
    "\n               right_img_topic: %s" \
    "\n               combine_img_topic: %s" \
    "\n               left_cam_info_topic: %s" \
    "\n               right_cam_info_topic: %s",
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
    nodePare_->device_mode_.c_str(),
    nodePare_->dual_combine_,
    (nodePare_->lpwm_enable_ ? "true" : "false"),
    (nodePare_->gdc_enable_ ? "true" : "false"),
    nodePare_->frame_ts_type_.c_str(),
    frame_id_.c_str(),
    frame_id2_.c_str(),
    nodePare_->link_type_,
    nodePare_->link_port_,
    io_method_name_.c_str(),
    left_img_topic_.c_str(),
    right_img_topic_.c_str(),
    combine_img_topic_.c_str(),
    left_cam_info_topic_.c_str(),
    right_cam_info_topic_.c_str()
  );

  init();
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
  for (auto &pub : Pub_info_) {
    pub.image_pub_.reset();
    pub.info_pub_.reset();
    pub.info_pub2_.reset();
  }
  for (auto &pub : Pub_hbmem_info_) {
    pub.publisher_hbmem_.reset();
    pub.info_pub_.reset();
    pub.info_pub2_.reset();
  }
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
        Pub_info_.resize(3);
        init_DualCalibration(&Pub_info_[0], &Pub_info_[1], left_cam_info_topic_, right_cam_info_topic_, nodePare_->camera_calibration_file_path_);
        init_publisher(Pub_info_[0], left_img_topic_, "left", frame_id_);
        init_publisher(Pub_info_[1], right_img_topic_, "right", frame_id2_);
        init_publisher(Pub_info_[2], combine_img_topic_, "combine", frame_id_);
      } else if (nodePare_->dual_combine_ == 2) {
        Pub_info_.resize(1);
        init_DualCalibration(&Pub_info_[0], left_cam_info_topic_, right_cam_info_topic_, nodePare_->camera_calibration_file_path_);
        init_publisher(Pub_info_[0], combine_img_topic_, "combine", frame_id_);
      } else {
        Pub_info_.resize(2);
        init_DualCalibration(&Pub_info_[0], &Pub_info_[1], left_cam_info_topic_, right_cam_info_topic_, nodePare_->camera_calibration_file_path_);
        init_publisher(Pub_info_[0], left_img_topic_, "left", frame_id_);
        init_publisher(Pub_info_[1], right_img_topic_, "right", frame_id2_);
      }
    } else if ((nodePare_->device_mode_.compare("single") == 0) ||
      (nodePare_->device_mode_.compare("") == 0)) {
      Pub_info_.resize(2);
      init_Calibration(&Pub_info_[0], "camera_info", nodePare_->camera_calibration_file_path_);
      init_publisher(Pub_info_[0], "image_raw", "single", frame_id_);
      if (nodePare_->sub_stream_flag_) {
        init_publisher(Pub_info_[1], "sub_image_raw", "sub_single", frame_id_);
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
        Pub_hbmem_info_.resize(3);
        init_DualCalibration(&Pub_info_[0], &Pub_info_[1], "image_left_raw/camera_info", "image_right_raw/camera_info", nodePare_->camera_calibration_file_path_);
        init_publisher_hbmem(Pub_hbmem_info_[0], "hbmem_left_img", "left");
        init_publisher_hbmem(Pub_hbmem_info_[1], "hbmem_right_img", "right");
        init_publisher_hbmem(Pub_hbmem_info_[2], "hbmem_combine_img", "combine");
      } else if (nodePare_->dual_combine_ == 2) {
        Pub_hbmem_info_.resize(1);
        init_DualCalibration(&Pub_hbmem_info_[0], "image_left_raw/camera_info", "image_right_raw/camera_info", nodePare_->camera_calibration_file_path_);
        init_publisher_hbmem(Pub_hbmem_info_[0], "hbmem_combine_img", "combine");
      } else {
        Pub_hbmem_info_.resize(2);
        init_DualCalibration(&Pub_info_[0], &Pub_info_[1], "image_left_raw/camera_info", "image_right_raw/camera_info", nodePare_->camera_calibration_file_path_);
        init_publisher_hbmem(Pub_hbmem_info_[0], "hbmem_left_img", "left");
        init_publisher_hbmem(Pub_hbmem_info_[1], "hbmem_right_img", "right");
      }
    } else if ((nodePare_->device_mode_.compare("single") == 0) ||
      (nodePare_->device_mode_.compare("") == 0)) {
      Pub_hbmem_info_.resize(2);
      init_Calibration(&Pub_hbmem_info_[0], "camera_info", nodePare_->camera_calibration_file_path_);
      init_publisher_hbmem(Pub_hbmem_info_[0], "hbmem_img", "single");
      if (nodePare_->sub_stream_flag_) {
        init_publisher_hbmem(Pub_hbmem_info_[1], "sub_hbmem_img", "sub_single");
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
    for (Publisher_info_st& info : Pub_info_) {
      //timer_.push_back(this->create_wall_timer(
      //  std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
      //  std::bind(&MipiCamNode::update, this, info)));
      //timer_tmp_ = this->create_wall_timer(
      //  std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
      //  std::bind(&MipiCamNode::update, this, info));
      // std::bind(&MipiCamNode::update, this, info);
      timer_.emplace_back(
        std::make_shared<std::thread>([this, &info]() { while(rclcpp::ok()) {this->update(&info);}})
      );
    }

  } else if (io_method_name_.compare("shared_mem") == 0) {
    for (Publisher_hbmem_info_st& info : Pub_hbmem_info_) {
      //timer_.push_back(this->create_wall_timer(
      //  std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
      //  std::bind(&MipiCamNode::hbmemUpdate, this, info)));
      timer_.emplace_back(
        std::make_shared<std::thread>([this, &info]() { while(rclcpp::ok()) {this->hbmemUpdate(&info);}})
      );  
    }
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_node"),
                     "starting timer " << period_ms);
  m_bIsInit = 1;
}

void MipiCamNode::init_publisher(Publisher_info_st&  Pub_info, std::string topic, std::string topic_type,
                    std::string frame_id){
  Pub_info.image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, PUB_BUF_NUM);
  Pub_info.frame_id = frame_id;
  Pub_info.frame_id2 = frame_id2_;
  Pub_info.topic_type = topic_type;
  Pub_info.time_start_ = std::chrono::system_clock::now();
}

void MipiCamNode::init_publisher_hbmem(Publisher_hbmem_info_st&  Pub_info, std::string topic, std::string topic_type){
  Pub_info.publisher_hbmem_ = this->create_publisher<hbm_img_msgs::msg::HbmMsg1080P>(topic, rclcpp::SensorDataQoS());
  Pub_info.topic_type = topic_type;
  Pub_info.time_start_ = std::chrono::system_clock::now();
}


void MipiCamNode::init_Calibration(Publisher_info_base_st*  Pub_info,
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

void MipiCamNode::init_DualCalibration(Publisher_info_base_st*  Pub_info,
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


void MipiCamNode::init_DualCalibration(Publisher_info_base_st*  Pub_info, Publisher_info_base_st*  Pub_info2,
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


void MipiCamNode::update(Publisher_info_st* pub_info) {
  if (mipiCam_ptr_ && mipiCam_ptr_->isCapturing()) {
    auto img = std::make_unique<sensor_msgs::msg::Image>(rosidl_runtime_cpp::MessageInitialization::SKIP);
    img->header.frame_id = pub_info->frame_id;
    if (std::string(pub_info->image_pub_->get_topic_name()) == "/image_right_raw") {
      img->header.frame_id = pub_info->frame_id2;
    }
    
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
#if 0
    if ("realtime" == nodePare_->frame_ts_type_) {
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      img->header.stamp.sec = ts.tv_sec;
      img->header.stamp.nanosec = ts.tv_nsec;
    }
#endif
    save_jpg(img->header.stamp,img->encoding,img->width,img->height,(void *)&img->data[0]);
    save_yuv(img->header.stamp, (void *)&img->data[0], img->data.size());

    if (pub_info && pub_info->info_pub_ && pub_info->camera_calibration_info_) {
      sensor_msgs::msg::CameraInfo camera_calibration_info = *pub_info->camera_calibration_info_;
      camera_calibration_info.header.stamp = img->header.stamp;
      camera_calibration_info.header.frame_id = pub_info->frame_id;
      if (std::string(pub_info->info_pub_->get_topic_name()) == right_cam_info_topic_) {
        camera_calibration_info.header.frame_id = pub_info->frame_id2;
      }
      pub_info->info_pub_->publish(camera_calibration_info);
    }

    pub_info->image_pub_->publish(std::move(img));
    // RCLCPP_WARN(this->get_logger(), "=>image_pub_ topic name: %s", pub_info->image_pub_->get_topic_name());
  }
}

void MipiCamNode::hbmemUpdate(Publisher_hbmem_info_st* pub_info) {
  if (mipiCam_ptr_ && mipiCam_ptr_->isCapturing()) {
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
#if 0
      if ("realtime" == nodePare_->frame_ts_type_) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        msg.time_stamp.sec = ts.tv_sec;
        msg.time_stamp.nanosec = ts.tv_nsec;
      }
#endif
      std::string encode(msg.encoding.begin(), msg.encoding.end());
      save_jpg(msg.time_stamp,encode,msg.width,msg.height,(void *)&msg.data);
      save_yuv(msg.time_stamp, (void *)&msg.data, msg.data_size);
      msg.index = pub_info->mSendIdx++;
      pub_info->publisher_hbmem_->publish(std::move(loanedMsg));
      if (pub_info->info_pub_) {
        pub_info->camera_calibration_info_->header.stamp = msg.time_stamp;
        pub_info->info_pub_->publish(*pub_info->camera_calibration_info_);
      }
      if (pub_info->info_pub2_) {
        pub_info->camera_calibration_info2_->header.stamp = msg.time_stamp;
        pub_info->info_pub2_->publish(*pub_info->camera_calibration_info2_);
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "borrow_loaned_message failed");
    }
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

}  // namespace mipi_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mipi_cam::MipiCamNode)
