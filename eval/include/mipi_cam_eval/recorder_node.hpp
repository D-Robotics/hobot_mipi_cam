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

// 评测接收端 recorder（缝2）：订阅被测 topic，按行落 CSV 元数据。
//
// CSV 列契约（分析链只认此格式）：
//   topic,seq,stamp_ns,recv_mono_ns,width,height,encoding
//   - seq：hbmem 消息为自增 index（接收丢帧检测）；其余消息为空
//   - stamp_ns：消息 header.stamp（图像默认 monotonic 域，IMU 为 REALTIME 域）
//   - recv_mono_ns：回调进入时刻 CLOCK_MONOTONIC
// 图像数据不落盘，只记元数据。
//
// 时钟旁车文件 <csv_path>.clock（域归一依据，ADR-0001）：
//   realtime_ns,mono_ns,offset_ns   （offset = realtime − monotonic）

#ifndef MIPI_CAM_EVAL_RECORDER_NODE_HPP_
#define MIPI_CAM_EVAL_RECORDER_NODE_HPP_

#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#ifdef EVAL_WITH_HBMEM
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

namespace mipi_cam_eval {

class RecorderNode : public rclcpp::Node {
 public:
  explicit RecorderNode(const rclcpp::NodeOptions &options);
  ~RecorderNode() override;

  const std::string &csv_path() const { return csv_path_; }

 private:
  void write_row(const std::string &topic, const std::string &seq,
                 int64_t stamp_ns, uint32_t width, uint32_t height,
                 const std::string &encoding);
  // 双钟同刻采样并写旁车文件（构造时调用一次）
  void sample_and_write_clock();

  std::mutex csv_mtx_;
  std::ofstream csv_;
  std::string csv_path_;
  int64_t clock_offset_ns_ = 0;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

}  // namespace mipi_cam_eval

#endif  // MIPI_CAM_EVAL_RECORDER_NODE_HPP_
