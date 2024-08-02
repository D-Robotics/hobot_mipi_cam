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

#define PUB_BUF_NUM 5
namespace mipi_cam {

MipiCamNode::MipiCamNode(const rclcpp::NodeOptions& node_options)
    : m_bIsInit(0),
      Node("mipi_cam", node_options),
      camera_calibration_info_(new sensor_msgs::msg::CameraInfo()) {

  getParams();
  init();
}

MipiCamNode::~MipiCamNode() {
  RCLCPP_WARN(rclcpp::get_logger("mipi_node"), "shutting down");
  if (mipiCam_ptr_) {
    mipiCam_ptr_->stop();
    mipiCam_ptr_->deInit();
    mipiCam_ptr_ = nullptr;
  }
}

void MipiCamNode::getParams() {
  std::string tros_distro
      = std::string(std::getenv("TROS_DISTRO")? std::getenv("TROS_DISTRO") : "");
  // declare params
  this->declare_parameter("config_path", "/opt/tros/" + tros_distro + "/lib/mipi_cam/config/");
  this->declare_parameter("channel", 0);
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 30.0);  // 10.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 1080);  // 480);
  this->declare_parameter("image_width", 1920);   // 640);
  this->declare_parameter("io_method", "ros");
  this->declare_parameter("out_format", "bgr8");   // nv12
  this->declare_parameter("video_device", "");  // "F37");
  this->declare_parameter("camera_calibration_file_path", "");
  this->declare_parameter("gdc_bin_file", "");
  this->declare_parameter("device_mode", "single");
  this->declare_parameter("dual_combine", 0);
  this->declare_parameter("frame_ts_type", nodePare_.frame_ts_type_);
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  for (auto& parameter :
       parameters_client->get_parameters({"config_path",
                                          "camera_info_url",
                                          "out_format",
                                          "channel",
                                          "frame_id",
                                          "framerate",
                                          "image_height",
                                          "image_width",
                                          "io_method",
                                          "video_device",
                                          "camera_calibration_file_path",
                                          "gdc_bin_file",
                                          "device_mode",
                                          "dual_combine",
                                          "frame_ts_type"
                                          })) {
    if (parameter.get_name() == "config_path") {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "config_path value: %s",
                  parameter.value_to_string().c_str());
      nodePare_.config_path_ = parameter.value_to_string();
    } else if (parameter.get_name() == "channel") {
      nodePare_.channel_ = parameter.as_int();
    } else if (parameter.get_name() == "camera_info_url") {
      nodePare_.camera_info_url_ = parameter.value_to_string();
    } else if (parameter.get_name() == "out_format") {
      nodePare_.out_format_name_ = parameter.value_to_string();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "out_format value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "frame_id") {
      frame_id_ = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "framerate: %f",
                  parameter.as_double());
      nodePare_.framerate_ = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      nodePare_.image_height_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "image_height_ value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "image_width") {
      nodePare_.image_width_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "image_width_ value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "io_method") {
      io_method_name_ = parameter.value_to_string();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "io_method_name_: %s",
                  io_method_name_.c_str());
    } else if (parameter.get_name() == "video_device") {
      nodePare_.video_device_name_ = parameter.value_to_string();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "video_device value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "camera_calibration_file_path") {
      nodePare_.camera_calibration_file_path_ = parameter.value_to_string();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "camera_calibration_file_path value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "gdc_bin_file") {
      nodePare_.gdc_bin_file_ = parameter.value_to_string();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "gdc_bin_file value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "device_mode") {
      nodePare_.device_mode_ = parameter.value_to_string();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "device_mode value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "dual_combine") {
      nodePare_.dual_combine_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "dual_combine value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "frame_ts_type") {
      nodePare_.frame_ts_type_ = parameter.value_to_string();
      RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                  "frame_ts_type value: %s",
                  nodePare_.frame_ts_type_.c_str());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                  "Invalid parameter name: %s",
                  parameter.get_name().c_str());
    }
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

  while (frame_id_ == "") {
    RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mipi_node"),
        "Required Parameters not set...waiting until they are set");
    getParams();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(
    rclcpp::get_logger("mipi_node"),
    "[MipiCamNode::%s]->Initing '%s' at %dx%d via %s at %i FPS",
    __func__,
    nodePare_.config_path_.c_str(),
    nodePare_.image_width_,
    nodePare_.image_height_,
    io_method_name_.c_str(),
    nodePare_.framerate_);

  if (io_method_name_.compare("ros") == 0) {
    if (nodePare_.device_mode_.compare("dual") == 0) {

      if (nodePare_.dual_combine_ == 1) {
        Pub_info_.resize(3);
        init_DualCalibration(&Pub_info_[0], &Pub_info_[1], "camera_left_info", "camera_left_info", nodePare_.camera_calibration_file_path_);
        init_publisher(Pub_info_[0], "image_left_raw", "left", frame_id_);
        init_publisher(Pub_info_[1], "image_right_raw", "right", frame_id_);
        init_publisher(Pub_info_[2], "image_combine_raw", "combine", frame_id_);
      } else if (nodePare_.dual_combine_ == 2) {
        Pub_info_.resize(1);
        init_DualCalibration(&Pub_info_[0], "camera_left_info", "camera_right_info", nodePare_.camera_calibration_file_path_);
        init_publisher(Pub_info_[0], "image_combine_raw", "combine", frame_id_);
      } else {
        Pub_info_.resize(2);
        init_DualCalibration(&Pub_info_[0], &Pub_info_[1], "camera_left_info", "camera_left_info", nodePare_.camera_calibration_file_path_);
        init_publisher(Pub_info_[0], "image_left_raw", "left", frame_id_);
        init_publisher(Pub_info_[1], "image_right_raw", "right", frame_id_);        
      }
    } else if ((nodePare_.device_mode_.compare("single") == 0) ||
      (nodePare_.device_mode_.compare("") == 0)) {
      Pub_info_.resize(1);
      init_Calibration(&Pub_info_[0], "camera_info", nodePare_.camera_calibration_file_path_);
      init_publisher(Pub_info_[0], "image_raw", "single", frame_id_);
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
    if (nodePare_.device_mode_.compare("dual") == 0) {

      if (nodePare_.dual_combine_ == 1) {
        Pub_hbmem_info_.resize(3);
        init_DualCalibration(&Pub_info_[0], &Pub_info_[1], "camera_left_info", "camera_left_info", nodePare_.camera_calibration_file_path_);
        init_publisher_hbmem(Pub_hbmem_info_[0], "hbmem_left_img", "left");
        init_publisher_hbmem(Pub_hbmem_info_[1], "hbmem_right_img", "right");
        init_publisher_hbmem(Pub_hbmem_info_[2], "hbmem_combine_img", "combine");
      } else if (nodePare_.dual_combine_ == 2) {
        Pub_hbmem_info_.resize(1);
        init_DualCalibration(&Pub_hbmem_info_[0], "camera_left_info", "camera_right_info", nodePare_.camera_calibration_file_path_);
        init_publisher_hbmem(Pub_hbmem_info_[0], "hbmem_combine_img", "combine");
      } else {
        Pub_hbmem_info_.resize(2);
        init_DualCalibration(&Pub_info_[0], &Pub_info_[1], "camera_left_info", "camera_left_info", nodePare_.camera_calibration_file_path_);
        init_publisher_hbmem(Pub_hbmem_info_[0], "hbmem_left_img", "left");
        init_publisher_hbmem(Pub_hbmem_info_[1], "hbmem_right_img", "right");
      }
    } else if ((nodePare_.device_mode_.compare("single") == 0) ||
      (nodePare_.device_mode_.compare("") == 0)) {
      Pub_hbmem_info_.resize(1);
      init_Calibration(&Pub_hbmem_info_[0], "camera_info", nodePare_.camera_calibration_file_path_);
      init_publisher_hbmem(Pub_hbmem_info_[0], "hbmem_img", "single");
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
  const int period_ms = 1000.0 / nodePare_.framerate_;


  if (io_method_name_.compare("ros") == 0) {
    for (Publisher_info_st& info : Pub_info_) {
      //timer_.push_back(this->create_wall_timer(
      //  std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
      //  std::bind(&MipiCamNode::update, this, info)));
      //timer_tmp_ = this->create_wall_timer(
      //  std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
      //  std::bind(&MipiCamNode::update, this, info));
      // std::bind(&MipiCamNode::update, this, info);
      timer_.push_back(this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        [this, &info]() {
          this->update(&info);
        }));
    }

  } else if (io_method_name_.compare("shared_mem") == 0) {
    for (Publisher_hbmem_info_st& info : Pub_hbmem_info_) {
      //timer_.push_back(this->create_wall_timer(
      //  std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
      //  std::bind(&MipiCamNode::hbmemUpdate, this, info)));
      timer_.push_back(this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        [this, &info]() {
          this->hbmemUpdate(&info);
        }));
    }
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_node"),
                     "starting timer " << period_ms);
  m_bIsInit = 1;
}

void MipiCamNode::init_publisher(Publisher_info_st&  Pub_info, std::string topic, std::string topic_type,
                    std::string frame_id){
  Pub_info.image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic, PUB_BUF_NUM);
  Pub_info.img_ = std::make_unique<sensor_msgs::msg::Image>();
  Pub_info.img_->header.frame_id = frame_id;
  Pub_info.topic_type = topic_type;
}

void MipiCamNode::init_publisher_hbmem(Publisher_hbmem_info_st&  Pub_info, std::string topic, std::string topic_type){
  Pub_info.publisher_hbmem_ = this->create_publisher<hbm_img_msgs::msg::HbmMsg1080P>(topic, rclcpp::SensorDataQoS());
  Pub_info.topic_type = topic_type;

}


void MipiCamNode::init_Calibration(Publisher_info_base_st*  Pub_info,
                    std::string info_topic, std::string info_file){
  Pub_info->camera_calibration_info_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  if (!mipiCam_ptr_->getCamCalibration(*Pub_info->camera_calibration_info_, info_file)) {
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
  if (!mipiCam_ptr_->getDualCamCalibration(*Pub_info->camera_calibration_info_, 
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
  if (!mipiCam_ptr_->getDualCamCalibration(*Pub_info->camera_calibration_info_, 
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
  if (mipiCam_ptr_->isCapturing()) {
    if (!mipiCam_ptr_->getImage(pub_info->img_->header.stamp,
                          pub_info->img_->encoding,
                          pub_info->img_->height,
                          pub_info->img_->width,
                          pub_info->img_->step,
                          pub_info->img_->data,
                          pub_info->topic_type)) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_node"), "grab failed");
      return;
    }
    
    if ("realtime" == nodePare_.frame_ts_type_) {
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      pub_info->img_->header.stamp.sec = ts.tv_sec;
      pub_info->img_->header.stamp.nanosec = ts.tv_nsec;
    }
    save_yuv(pub_info->img_->header.stamp, (void *)&pub_info->img_->data[0], pub_info->img_->data.size());
    pub_info->image_pub_->publish(*pub_info->img_);
    if (pub_info->info_pub_) {
      pub_info->camera_calibration_info_->header.stamp = pub_info->img_->header.stamp;
      pub_info->info_pub_->publish(*pub_info->camera_calibration_info_);
    }
    if (pub_info->info_pub2_) {
      pub_info->camera_calibration_info2_->header.stamp = pub_info->img_->header.stamp;
      pub_info->info_pub2_->publish(*pub_info->camera_calibration_info2_);
    }
  }
}

void MipiCamNode::hbmemUpdate(Publisher_hbmem_info_st* pub_info) {
  if (mipiCam_ptr_->isCapturing()) {
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
        RCLCPP_ERROR(rclcpp::get_logger("mipi_node"),
                    "hbmemUpdate grab img failed");
        return;
      }

      if ("realtime" == nodePare_.frame_ts_type_) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        msg.time_stamp.sec = ts.tv_sec;
        msg.time_stamp.nanosec = ts.tv_nsec;
      }
      save_yuv(msg.time_stamp, (void *)&msg.data, msg.data_size);
      msg.index = pub_info->mSendIdx++;
      pub_info->publisher_hbmem_->publish(std::move(loanedMsg));
      if (pub_info->info_pub_) {
        pub_info->camera_calibration_info_->header.stamp = msg.time_stamp;
        pub_info->info_pub_->publish(*pub_info->camera_calibration_info_);
      }
      if (pub_info->info_pub_) {
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

}  // namespace mipi_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mipi_cam::MipiCamNode)