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

// 缝2：recorder（ROS2 topic 边界）测试。
// 合成发布者灌可控 stamp 的消息，断言 CSV 行为（stamp 保真、recv 单调、
// 时钟旁车文件 offset ≈ realtime − monotonic）。

#include <gtest/gtest.h>

#include <chrono>
#include <ctime>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "mipi_cam_eval/recorder_node.hpp"

namespace {

struct CsvRow {
  std::string topic, seq, encoding;
  long long stamp_ns = 0, recv_mono_ns = 0;
  long width = 0, height = 0;
};

std::vector<std::string> split_csv_line(const std::string &line) {
  std::vector<std::string> out;
  size_t start = 0;
  while (true) {
    const size_t c = line.find(',', start);
    if (c == std::string::npos) {
      out.push_back(line.substr(start));
      break;
    }
    out.push_back(line.substr(start, c - start));
    start = c + 1;
  }
  return out;
}

std::vector<CsvRow> read_csv(const std::string &path) {
  std::vector<CsvRow> rows;
  std::ifstream f(path);
  std::string line;
  std::getline(f, line);  // header
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    const auto fields = split_csv_line(line);
    if (fields.size() != 7) continue;
    CsvRow r;
    r.topic = fields[0];
    r.seq = fields[1];
    r.stamp_ns = std::stoll(fields[2]);
    r.recv_mono_ns = std::stoll(fields[3]);
    r.width = std::stol(fields[4]);
    r.height = std::stol(fields[5]);
    r.encoding = fields[6];
    rows.push_back(r);
  }
  return rows;
}

int64_t mono_now_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int64_t realtime_now_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

}  // namespace

class RecorderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_shutdown_ = true;
    }
    csv_path_ = "/tmp/mipi_cam_eval_recorder_" +
                std::to_string(++instance_counter_) + ".csv";
  }

  void TearDown() override {
    if (owns_shutdown_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::string csv_path_;
  static int instance_counter_;
  bool owns_shutdown_ = false;
};

int RecorderTest::instance_counter_ = 0;

TEST_F(RecorderTest, ImageAndImuToCsv) {
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("csv_path", csv_path_);
  opts.append_parameter_override("image_topics",
                                 std::vector<std::string>{"/eval_test/image_raw"});
  opts.append_parameter_override("imu_topics",
                                 std::vector<std::string>{"/eval_test/imu"});
  opts.append_parameter_override(
      "info_topics", std::vector<std::string>{"/eval_test/image_raw/camera_info"});

  auto rec = std::make_shared<mipi_cam_eval::RecorderNode>(opts);
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(rec);

  auto pub_node = std::make_shared<rclcpp::Node>("recorder_test_pub");
  ex.add_node(pub_node);
  auto img_pub = pub_node->create_publisher<sensor_msgs::msg::Image>(
      "/eval_test/image_raw", rclcpp::SensorDataQoS());
  auto imu_pub = pub_node->create_publisher<sensor_msgs::msg::Imu>(
      "/eval_test/imu", 10);
  auto info_pub = pub_node->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/eval_test/image_raw/camera_info", 10);

  // 等待订阅建立
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((img_pub->get_subscription_count() < 1 ||
          imu_pub->get_subscription_count() < 1 ||
          info_pub->get_subscription_count() < 1) &&
         std::chrono::steady_clock::now() < deadline) {
    ex.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GE(img_pub->get_subscription_count(), 1u);
  ASSERT_GE(imu_pub->get_subscription_count(), 1u);
  ASSERT_GE(info_pub->get_subscription_count(), 1u);

  const int64_t img_base_ns = 1000000000000LL;  // 任意已知基准
  const int64_t imu_base_ns = 1700000000000000000LL;  // REALTIME 域风格的大值
  for (int i = 0; i < 5; ++i) {
    sensor_msgs::msg::Image img;
    img.header.stamp.sec = static_cast<int32_t>((img_base_ns + i * 33333333LL) / 1000000000LL);
    img.header.stamp.nanosec = static_cast<uint32_t>(
        (img_base_ns + i * 33333333LL) % 1000000000LL);
    img.width = 1920;
    img.height = 1080;
    img.encoding = "nv12";
    img_pub->publish(img);

    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = img.header.stamp;
    info.width = 1920;
    info.height = 1080;
    info_pub->publish(info);

    if (i < 3) {
      sensor_msgs::msg::Imu imu;
      imu.header.stamp.sec = static_cast<int32_t>((imu_base_ns + i * 5000000LL) / 1000000000LL);
      imu.header.stamp.nanosec = static_cast<uint32_t>(
          (imu_base_ns + i * 5000000LL) % 1000000000LL);
      imu_pub->publish(imu);
    }
    ex.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  // 排空回调：轮询 CSV 直到行数齐（慢环境/DDS 首包延迟容忍至 10s）
  std::vector<CsvRow> rows;
  const auto row_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < row_deadline) {
    for (int i = 0; i < 10; ++i) {
      ex.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rows = read_csv(csv_path_);
    if (rows.size() >= 13) break;
  }
  ASSERT_EQ(rows.size(), 13u);  // 5 image + 5 info + 3 imu

  int n_img = 0, n_imu = 0, n_info = 0;
  long long prev_recv = -1;
  std::vector<long long> img_stamps;
  for (const auto &r : rows) {
    EXPECT_GE(r.recv_mono_ns, prev_recv);  // 全局接收时刻不回退
    prev_recv = r.recv_mono_ns;
    if (r.topic == "/eval_test/image_raw") {
      ++n_img;
      EXPECT_EQ(1920, r.width);
      EXPECT_EQ(1080, r.height);
      EXPECT_EQ("nv12", r.encoding);
      EXPECT_TRUE(r.seq.empty());
      img_stamps.push_back(r.stamp_ns);
    } else if (r.topic == "/eval_test/imu") {
      ++n_imu;
      EXPECT_EQ("imu", r.encoding);
    } else if (r.topic == "/eval_test/image_raw/camera_info") {
      ++n_info;
      EXPECT_EQ("camera_info", r.encoding);
    } else {
      FAIL() << "unexpected topic: " << r.topic;
    }
  }
  EXPECT_EQ(5, n_img);
  EXPECT_EQ(3, n_imu);
  EXPECT_EQ(5, n_info);

  // image stamp 保真且递增
  for (size_t i = 1; i < img_stamps.size(); ++i) {
    EXPECT_LT(img_stamps[i - 1], img_stamps[i]);
  }
  EXPECT_EQ(img_base_ns, img_stamps.front());

  // 时钟旁车：offset ≈ realtime − monotonic（容差 50ms）
  std::ifstream cf(csv_path_ + ".clock");
  ASSERT_TRUE(cf.is_open());
  std::string header_line;
  std::getline(cf, header_line);
  long long rt = 0, mn = 0, offset = 0;
  char comma;
  cf >> rt >> comma >> mn >> comma >> offset;
  const int64_t own_offset = realtime_now_ns() - mono_now_ns();
  EXPECT_LT(std::llabs(offset - own_offset), 50LL * 1000000LL);

  ex.remove_node(rec);
  ex.remove_node(pub_node);
  rec.reset();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
