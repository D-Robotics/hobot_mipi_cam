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

//#include "hb_mem_mgr.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"

namespace mipi_cam
{

typedef struct Publisher_info_base {
  // shared image message
  sensor_msgs::msg::CameraInfo::UniquePtr camera_calibration_info_;
  sensor_msgs::msg::CameraInfo::UniquePtr camera_calibration_info2_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub2_ = nullptr;
} Publisher_info_base_st;

typedef struct Publisher_info : Publisher_info_base_st  {
  // shared image message
  sensor_msgs::msg::Image::UniquePtr img_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_ = nullptr;
  std::string topic_type;
} Publisher_info_st;

typedef struct Publisher_hbmem_info : Publisher_info_base_st {
  // shared image message
  int mSendIdx;
  rclcpp::Publisher<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr publisher_hbmem_;
  std::string topic_type;
} Publisher_hbmem_info_st;

class MipiCamNode : public rclcpp::Node {
 public:
  MipiCamNode(const rclcpp::NodeOptions & node_options);
  ~MipiCamNode();
  void init();
  void update(Publisher_info_st* pub_info);
  void hbmemUpdate(Publisher_hbmem_info_st* pub_info);

 private:
  void getParams();
  void save_yuv(const builtin_interfaces::msg::Time stamp, void *data, int data_size);
  void init_publisher(Publisher_info_st&  Pub_info, std::string topic, std::string topic_type,
                      std::string frame_id);
  void init_publisher_hbmem(Publisher_hbmem_info_st&  Pub_info, std::string topic, std::string topic_type);
  
  void init_Calibration(Publisher_info_base_st*  Pub_info,
                    std::string info_topic, std::string info_file);
  void init_DualCalibration(Publisher_info_base_st*  Pub_info,
                    std::string info_topic, std::string info_topic2, std::string info_file);
  void init_DualCalibration(Publisher_info_base_st*  Pub_info, Publisher_info_base_st*  Pub_info2,
                    std::string info_topic, std::string info_topic2, std::string info_file);

  std::shared_ptr<MipiCam> mipiCam_ptr_;

  std::vector<rclcpp::TimerBase::SharedPtr> timer_;
  int32_t mSendIdx = 0;

  rclcpp::TimerBase::SharedPtr timer_tmp_;

  std::vector<Publisher_info_st> Pub_info_;
  std::vector<Publisher_hbmem_info_st> Pub_hbmem_info_;

  sensor_msgs::msg::CameraInfo::UniquePtr camera_calibration_info_;
  

  // parameters
  std::string frame_id_;
  std::string io_method_name_;  // hbmem zero mem copy
  struct NodePara nodePare_;
  int m_bIsInit;
};
}  // namespace mipi_cam
#endif  // HOBOT_MIPI_NODE_HPP_
