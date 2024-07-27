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

#ifndef HOBOT_MIPI_NODE_HPP_
#define HOBOT_MIPI_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cam.hpp"

// #include <vector>
#include <memory>
#include <string>

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/string.hpp>

#include "hb_mem_mgr.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"

namespace mipi_cam
{

class MipiCamNode : public rclcpp::Node {
 public:
  MipiCamNode(const rclcpp::NodeOptions & node_options);
  ~MipiCamNode();
  void init();
  void update();
  void hbmemUpdate();

 private:
  void getParams();
  bool sendCalibration(const builtin_interfaces::msg::Time &stamp);
  void save_yuv(const builtin_interfaces::msg::Time stamp, void *data, int data_size);

  std::shared_ptr<MipiCam> mipiCam_ptr_;

  // shared image message
  sensor_msgs::msg::Image::UniquePtr img_;
  sensor_msgs::msg::CameraInfo::UniquePtr camera_calibration_info_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;

  int32_t mSendIdx = 0;
  rclcpp::TimerBase::SharedPtr timer_hbmem_;
  rclcpp::Publisher<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr publisher_hbmem_;

  // parameters
  std::string frame_id_;
  std::string io_method_name_;  // hbmem zero mem copy
  struct NodePara nodePare_;
  int m_bIsInit;
};
}  // namespace mipi_cam
#endif  // HOBOT_MIPI_NODE_HPP_
