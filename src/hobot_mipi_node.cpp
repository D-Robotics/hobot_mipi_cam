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
      img_(new sensor_msgs::msg::Image()),
      camera_calibration_info_(new sensor_msgs::msg::CameraInfo()) {

  getParams();
  // init();
}

MipiCamNode::~MipiCamNode() {
  RCLCPP_WARN(rclcpp::get_logger("mipi_node"), "shutting down");
  if (mipiCam_ptr_) {
    mipiCam_ptr_->stop();
    mipiCam_ptr_->deInit();
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
  if (io_method_name_.compare("ros") == 0) {
    image_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("image_raw", PUB_BUF_NUM);
  }
  info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", PUB_BUF_NUM);

  while (frame_id_ == "") {
    RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mipi_node"),
        "Required Parameters not set...waiting until they are set");
    getParams();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  img_->header.frame_id = frame_id_;
  camera_calibration_info_->header.frame_id = frame_id_;
  RCLCPP_INFO(
      rclcpp::get_logger("mipi_node"),
      "[MipiCamNode::%s]->Initing '%s' at %dx%d via %s at %i FPS",
      __func__,
      nodePare_.config_path_.c_str(),
      nodePare_.image_width_,
      nodePare_.image_height_,
      io_method_name_.c_str(),
      nodePare_.framerate_);
  // set the IO method
  if (io_method_name_.compare("shared_mem") == 0) {
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

    // 创建hbmempub
    publisher_hbmem_ =
        this->create_publisher<hbm_img_msgs::msg::HbmMsg1080P>(
            "hbmem_img", rclcpp::SensorDataQoS());
  }

  // start the camera
  if (0 != mipiCam_ptr_->start()) {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_node"),
                      "mipi camera start failed!");
    rclcpp::shutdown();
    return;
  }
  const int period_ms = 1000.0 / nodePare_.framerate_;
  if (!mipiCam_ptr_->getCamCalibration(*camera_calibration_info_,
                        nodePare_.camera_calibration_file_path_)) {
    camera_calibration_info_ = nullptr;
    RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                "get camera calibration parameters failed");
  }
  if (io_method_name_.compare("shared_mem") != 0) {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&MipiCamNode::update, this));
  } else {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&MipiCamNode::hbmemUpdate, this));
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_node"),
                     "starting timer " << period_ms);
  m_bIsInit = 1;
}

bool MipiCamNode::sendCalibration(const builtin_interfaces::msg::Time& stamp) {
  if (camera_calibration_info_ != nullptr) {
    camera_calibration_info_->header.stamp = stamp;
    info_pub_->publish(*camera_calibration_info_);
    return true;
  } else {
    return false;
  }
}

void MipiCamNode::update() {
  if (mipiCam_ptr_->isCapturing()) {
    if (!mipiCam_ptr_->getImage(img_->header.stamp,
                            img_->encoding,
                            img_->height,
                            img_->width,
                            img_->step,
                            img_->data)) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_node"), "grab failed");
      return;
    }
    if ("realtime" == nodePare_.frame_ts_type_) {
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      img_->header.stamp.sec = ts.tv_sec;
      img_->header.stamp.nanosec = ts.tv_nsec;
    }
    save_yuv(img_->header.stamp, (void *)&img_->data[0], img_->data.size());
    image_pub_->publish(*img_);
    if (sendCalibration(img_->header.stamp)) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_node"), "publish camera info.\n");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "Unable to publish camera info.\n");
    }
  }
}

void MipiCamNode::hbmemUpdate() {
  if (mipiCam_ptr_->isCapturing()) {
    auto loanedMsg = publisher_hbmem_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto& msg = loanedMsg.get();
      if (!mipiCam_ptr_->getImageMem(msg.time_stamp,
                                  msg.encoding,
                                  msg.height,
                                  msg.width,
                                  msg.step,
                                  msg.data,
                                  msg.data_size)) {
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
      msg.index = mSendIdx++;
      publisher_hbmem_->publish(std::move(loanedMsg));
      if (sendCalibration(msg.time_stamp)) {
        RCLCPP_INFO(rclcpp::get_logger("mipi_node"), "publish camera info.");
      } else {
        RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                    "Unable to publish camera info.");
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
