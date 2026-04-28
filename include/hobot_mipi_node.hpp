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
#include "hobot_mipi_calibration.hpp"

// #include <vector>
#include <memory>
#include <string>

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/string.hpp>

#include <fstream>
#include <iomanip>
#include <sys/stat.h> // mkdir

//#include "hb_mem_mgr.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "imu_manager.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
namespace mipi_cam
{

class Publisher_info_base {
  public:
  // shared image message
  sensor_msgs::msg::CameraInfo::UniquePtr camera_calibration_info_ = nullptr;
  sensor_msgs::msg::CameraInfo::UniquePtr camera_calibration_info2_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub2_ = nullptr;
  std::chrono::time_point<std::chrono::system_clock> time_start_;
};

class Publisher_info : public Publisher_info_base  {
  public:
  // shared image message
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_ = nullptr;
  std::string frame_id = "";
  std::string topic_type;
};

class Publisher_hbmem_info : public Publisher_info_base {
  public:
  // shared image message
  int mSendIdx;
  rclcpp::Publisher<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr publisher_hbmem_;
  std::string topic_type;
};

class MipiCamNode : public rclcpp::Node {
 public:
  MipiCamNode(const rclcpp::NodeOptions & node_options);
  ~MipiCamNode();
  void init();
  void update(std::shared_ptr<Publisher_info> pub_info);
  void hbmemUpdate(std::shared_ptr<Publisher_hbmem_info> pub_info);
  void read_imu_data();

 private:
  void save_yuv(const builtin_interfaces::msg::Time stamp, void *data, int data_size);
  void save_jpg(const builtin_interfaces::msg::Time stamp, std::string encode, int w, int h, void *data);
  void init_publisher(std::shared_ptr<Publisher_info>  Pub_info, std::string topic, std::string topic_type,
                      std::string frame_id);
  void init_publisher_hbmem(std::shared_ptr<Publisher_hbmem_info>  Pub_info, std::string topic, std::string topic_type);
  
  void init_Calibration(Publisher_info_base*  Pub_info,
                    std::string info_topic, std::string info_file);
  void init_DualCalibration(Publisher_info_base*  Pub_info,
                    std::string info_topic, std::string info_topic2, std::string info_file);
  void init_DualCalibration(Publisher_info_base*  Pub_info, Publisher_info_base*  Pub_info2,
                    std::string info_topic, std::string info_topic2, std::string info_file);
  void init_DualCalibration_Sub(Publisher_info_base *Pub_info,
                            std::string info_topic, std::string info_topic2, std::string info_file);
  void init_DualCalibration_Sub(Publisher_info_base *Pub_info, Publisher_info_base *Pub_info2,
                            std::string info_topic, std::string info_topic2, std::string info_file);

  std::shared_ptr<MipiCam> mipiCam_ptr_;

  std::vector<std::shared_ptr<std::thread>> timer_;
  int32_t mSendIdx = 0;

  rclcpp::TimerBase::SharedPtr timer_tmp_ = nullptr;

  std::vector<std::shared_ptr<Publisher_info>> Pub_info_;
  std::vector<std::shared_ptr<Publisher_hbmem_info>> Pub_hbmem_info_;

  sensor_msgs::msg::CameraInfo::UniquePtr camera_calibration_info_ = nullptr;
  

  // parameters
  std::string frame_id_;
  std::string io_method_name_;  // hbmem zero mem copy
  //struct NodePara nodePare_;
  std::shared_ptr<struct NodePara> nodePare_;
  int m_bIsInit;

  std::atomic_bool is_imu_running_ = false;
  std::string imu_type_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  std::shared_ptr<imu_sensor::ImuManager> imu_manager_;
  std::vector<std::shared_ptr<std::thread>> imu_timer_;

  std::string imu_calib_file_path_; // 标定文件保存路径
  void saveImuCalibration(const std::string &file_path);

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_imu_extrinsic_;
};
}  // namespace mipi_cam
#endif  // HOBOT_MIPI_NODE_HPP_
