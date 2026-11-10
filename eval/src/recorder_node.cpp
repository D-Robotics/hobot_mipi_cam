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

#include "mipi_cam_eval/recorder_node.hpp"

#include <ctime>
#include <stdexcept>

namespace {

int64_t clock_now_ns(clockid_t clk) {
  struct timespec ts;
  clock_gettime(clk, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int64_t mono_now_ns() { return clock_now_ns(CLOCK_MONOTONIC); }
int64_t realtime_now_ns() { return clock_now_ns(CLOCK_REALTIME); }

int64_t stamp_to_ns(const builtin_interfaces::msg::Time &t) {
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

}  // namespace

namespace mipi_cam_eval {

RecorderNode::RecorderNode(const rclcpp::NodeOptions &options)
    : Node("mipi_cam_recorder", options) {
  csv_path_ = this->declare_parameter<std::string>("csv_path", "eval_recorder.csv");
  const auto image_topics = this->declare_parameter<std::vector<std::string>>(
      "image_topics", std::vector<std::string>{});
  const auto imu_topics = this->declare_parameter<std::vector<std::string>>(
      "imu_topics", std::vector<std::string>{});
  const auto info_topics = this->declare_parameter<std::vector<std::string>>(
      "info_topics", std::vector<std::string>{});
  const auto hbmem_topics = this->declare_parameter<std::vector<std::string>>(
      "hbmem_topics", std::vector<std::string>{});

  csv_.open(csv_path_, std::ios::out | std::ios::trunc);
  if (!csv_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "cannot open csv file: %s", csv_path_.c_str());
    throw std::runtime_error("cannot open csv file: " + csv_path_);
  }
  csv_ << "topic,seq,stamp_ns,recv_mono_ns,width,height,encoding\n";
  csv_.flush();
  sample_and_write_clock();

  // 传感器类 QoS：深度1+best_effort（与被测端 SensorDataQoS 发布兼容，
  // 亦兼容可靠发布；recorder 不应反向给被测端制造反压）
  const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  for (const auto &t : image_topics) {
    subs_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
        t, sensor_qos,
        [this, t](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          write_row(t, "", stamp_to_ns(msg->header.stamp), msg->width,
                    msg->height, msg->encoding);
        }));
  }
  for (const auto &t : imu_topics) {
    subs_.push_back(this->create_subscription<sensor_msgs::msg::Imu>(
        t, sensor_qos,
        [this, t](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          write_row(t, "", stamp_to_ns(msg->header.stamp), 0, 0, "imu");
        }));
  }
  for (const auto &t : info_topics) {
    subs_.push_back(this->create_subscription<sensor_msgs::msg::CameraInfo>(
        t, sensor_qos,
        [this, t](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          write_row(t, "", stamp_to_ns(msg->header.stamp), msg->width,
                    msg->height, "camera_info");
        }));
  }
#ifdef EVAL_WITH_HBMEM
  for (const auto &t : hbmem_topics) {
    subs_.push_back(this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        t, sensor_qos,
        [this, t](hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg) {
          write_row(t, std::to_string(msg->index),
                    stamp_to_ns(msg->time_stamp), msg->width, msg->height,
                    std::string(msg->encoding.begin(), msg->encoding.end()));
        }));
  }
#endif

  RCLCPP_INFO(this->get_logger(),
              "recorder started: csv=%s, image=%zu imu=%zu info=%zu hbmem=%zu, "
              "clock_offset_ns=%lld",
              csv_path_.c_str(), image_topics.size(), imu_topics.size(),
              info_topics.size(), hbmem_topics.size(),
              static_cast<long long>(clock_offset_ns_));
}

RecorderNode::~RecorderNode() {
  std::lock_guard<std::mutex> lk(csv_mtx_);
  if (csv_.is_open()) {
    csv_.flush();
    csv_.close();
  }
}

void RecorderNode::sample_and_write_clock() {
  // 双钟背靠背采样（mono 夹 rt，取中点对齐，误差≈单次采样间隔）
  const int64_t mn1 = mono_now_ns();
  const int64_t rt = realtime_now_ns();
  const int64_t mn2 = mono_now_ns();
  const int64_t mn = mn1 + (mn2 - mn1) / 2;
  clock_offset_ns_ = rt - mn;

  std::ofstream f(csv_path_ + ".clock", std::ios::out | std::ios::trunc);
  f << "realtime_ns,mono_ns,offset_ns\n"
    << rt << ',' << mn << ',' << clock_offset_ns_ << '\n';
}

void RecorderNode::write_row(const std::string &topic, const std::string &seq,
                             int64_t stamp_ns, uint32_t width, uint32_t height,
                             const std::string &encoding) {
  const int64_t recv = mono_now_ns();
  std::lock_guard<std::mutex> lk(csv_mtx_);
  if (!csv_.is_open()) return;
  csv_ << topic << ',' << seq << ',' << stamp_ns << ',' << recv << ','
       << width << ',' << height << ',' << encoding << '\n';
  csv_.flush();  // 每行落盘：采集期可实时观测，崩溃不丢已收数据
}

}  // namespace mipi_cam_eval
