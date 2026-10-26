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

#ifndef MIPI_MIPI_COMM_HPP_
#define MIPI_MIPI_COMM_HPP_

#include <string>
#include <mutex>
#include <queue>
#include <memory>
#include <atomic>

extern "C" {
#include <string.h>
#include <stdlib.h>
}

namespace mipi_cam
{

struct NodePara {
  std::string config_path_;
  std::string video_device_name_;
  int channel_;
  int channel2_;
  std::string camera_info_url_;
  std::string camera_calibration_file_path_;
  std::string out_format_name_;
  std::string gdc_bin_file_;
  int image_width_;
  int image_height_;
  int sub_image_width_;
  int sub_image_height_;
  int framerate_;
  int sub_framerate_;
  double rotation_ = 0.0;
  double cal_rotation_ = 0.0;
  int stream_mode_; //0:表示正常模式，1：表示输出gdc矫正前和矫正后的双码流。
  std::string device_mode_; //"single":单目设备模式，"dual"：双目设备模式
  int dual_combine_; //当device_mode_=="dual"时生效，0：表示不支持拼接,输出左右图，1：标志支持拼接，并输出左右图+拼接图，2：表示支持拼接，只输出拼接图。
  // The type of timestamp for publishing messages
  // "realtime": Uses the system CLOCK_REALTIME time when the image data is obtained
  // "sensor"/"default": Uses the time when the sensor captures the image, which is the time obtained through the getFrame interface
  bool lpwm_enable_;
  bool gdc_enable_;
  std::string frame_ts_type_ {"sensor"};
  int link_type_; // 0:表示mipi接口，1：表示解串器接口。
  int link_port_; // 0~3
  std::string gsml_cfg_file_;
  double cal_alpha_;
  bool sub_stream_enable_;
  bool sync_awb_;
  bool sync_ccm_;
  bool sync_ae_;
  bool print_isp_log_;
  bool diag_enable_ = true;
  int diag_period_ms_ = 10000;
};

typedef struct {
  std::string config_path;
  std::string sensor_type;
  std::string out_format_name;
  int width;
  int height;
  int sub_width;
  int sub_height;
  int fps;
  int sub_fps;
  int channel_;
  int channel2_;
  double rotation_ = 0.0;
  double cal_rotation_ = 0.0;
  int stream_mode_; //0:表示正常模式，1：表示输出gdc矫正前和矫正后的双码流。
  std::string device_mode_; //"single":单目设备模式，"dual"：双目设备模式
  int dual_combine_; //当device_mode_=="dual"时生效，0：表示不支持拼接,输出左右图，1：标志支持拼接，并输出左右图+拼接图，2：表示支持拼接，只输出拼接图。
  bool lpwm_enable_;
  bool gdc_enable_;
  std::string gdc_bin_file_;
  std::string frame_ts_type_ {"sensor"};
  int link_type_; // 0:表示mipi接口，1：表示解串器接口。
  int link_port_; // 0~3
  std::string gsml_cfg_file_;
  double cal_alpha_;
  bool sub_stream_enable_;
  bool sync_awb_;
  bool sync_ccm_;
  bool sync_ae_;
  bool print_isp_log_;
  bool diag_enable_ = true;
  int diag_period_ms_ = 10000;
} MIPI_CAP_INFO_ST;

typedef struct sensor_id {
  int i2c_bus;           // sensor挂在哪条总线上
  int i2c_dev_addr;      // sensor i2c设备地址
  int i2c_addr_width;    // 总线地址宽（1/2字节）
  int det_reg;           // 读取的寄存器地址
  char sensor_name[10];  // sensor名字
} SENSOR_ID_T;

typedef struct {
  bool reset_flag;
  int reset_gpio;
  int reset_level;
  int i2c_bus;
  int mipi_host;
} BOARD_CONFIG_ST;

#define I2C_ADDR_8    1
#define I2C_ADDR_16   2

#define ARRAY_SIZE(a) ((sizeof(a) / sizeof(a[0])))


class VideoBuffer;
class BuffQueueManage : public std::enable_shared_from_this<BuffQueueManage>{
 public:
  void creat_buff(unsigned int num) {
    num = num < 10 ? (num == 0 ? 1 : num) : 10;
    for (int i = 0; i < num; i++) {
      auto buffer_ptr = std::make_shared<VideoBuffer>(shared_from_this());
      push_empty_que(buffer_ptr);
    }
    return;
  };
  std::mutex queue_mtx_;
  std::queue<std::shared_ptr<VideoBuffer>> q_empty_;
  std::queue<std::shared_ptr<VideoBuffer>> q_buff_;
  std::shared_ptr<VideoBuffer> get_empty_buff() {
    std::unique_lock<std::mutex> lk(queue_mtx_);
    std::shared_ptr<VideoBuffer> buff_ptr = nullptr;
    if (q_buff_.size() >= 3) {
      buff_ptr = q_buff_.front();
      q_buff_.pop();
    } else if (!q_empty_.empty()) {
      buff_ptr = q_empty_.front();
      q_empty_.pop();
    }
    return buff_ptr;
  }
  std::shared_ptr<VideoBuffer> get_data_buff() {
    std::unique_lock<std::mutex> lk(queue_mtx_);
    std::shared_ptr<VideoBuffer> buff_ptr = nullptr;
    if (!q_buff_.empty()) {
      buff_ptr = q_buff_.front();
      q_buff_.pop();
    }
    return buff_ptr;
  }
  // 诊断用只读水位（只加 getter，不改变队列行为）。
  size_t size_data() {
    std::unique_lock<std::mutex> lk(queue_mtx_);
    return q_buff_.size();
  }
  size_t size_empty() {
    std::unique_lock<std::mutex> lk(queue_mtx_);
    return q_empty_.size();
  }
  void push_empty_que(std::shared_ptr<VideoBuffer> buff_ptr) {
    if (buff_ptr) {
      std::unique_lock<std::mutex> lk(queue_mtx_);
      q_empty_.push(buff_ptr);
    }
  }

  void push_data_que(std::shared_ptr<VideoBuffer> buff_ptr) {
    if (buff_ptr) {
      std::unique_lock<std::mutex> lk(queue_mtx_);
      q_buff_.push(buff_ptr);
    }
  }
  virtual ~BuffQueueManage() {};
};


class VideoBuffer : public std::enable_shared_from_this<VideoBuffer> {
 public:

  VideoBuffer(const std::shared_ptr<BuffQueueManage>& manager) {
    w_manager = manager;
  }
  VideoBuffer(const VideoBuffer& other) {
    timestamp = other.timestamp;
    frame_id = other.frame_id;
    width = other.width;
    height = other.height;
    stride = other.stride;
    buff_size = other.buff_size;
    data_size = other.data_size;
    buff = other.buff;
    raw_buff = other.raw_buff;
    encode = other.encode;
    w_manager = other.w_manager;
  }
  uint64_t timestamp;
  uint32_t frame_id;
  int width;
  int height;
  int stride;
  uint32_t buff_size;
  uint32_t data_size;
  std::vector<char> buff;
  std::vector<char> raw_buff;
  std::string encode;
  std::weak_ptr<BuffQueueManage> w_manager = {};
  void return_empty_que() {
    if (auto manager_ptr = w_manager.lock()) {
      std::unique_lock<std::mutex> lk(manager_ptr->queue_mtx_);
      manager_ptr->q_empty_.push(shared_from_this());
    }
  }
  void return_data_que() {
    if (auto manager_ptr = w_manager.lock()) {
      std::unique_lock<std::mutex> lk(manager_ptr->queue_mtx_);
      manager_ptr->q_buff_.push(shared_from_this());
    }
  }
  void setManager(std::shared_ptr<BuffQueueManage>& manager) {
    w_manager = manager;  // 直接赋值shared_ptr
  }
};


/**
 * @brief frame
 */
struct Frame {
  unsigned long long timestamp_ns; // ns
  long long timestamp_ms; // ms
  uint32_t frame_id;
  int width;
  int height;
  int stride;
  std::string encode;
  std::vector<char> buf;
};

class FrameQueue {
 public:
  FrameQueue(size_t max_size = 5) : max_size_(max_size) {
  }

  void push(std::shared_ptr<VideoBuffer> frame_ptr) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (queue_.size() >= max_size_) {
      // drop old frame（计入累计丢弃，供诊断直接读取）
      queue_.pop_front();
      dropped_total_.fetch_add(1, std::memory_order_relaxed);
    }
    queue_.push_back(frame_ptr);
  }

  std::shared_ptr<VideoBuffer> pop() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (queue_.empty()) return nullptr;
    auto frame_ptr = queue_.front();
    queue_.pop_front();
    return frame_ptr;
  }

  std::shared_ptr<VideoBuffer> peek() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (queue_.empty()) return nullptr;
    return queue_.front();
  }

  // 丢弃队头所有时间戳早于 ts 的帧，返回实际丢弃数量（诊断统计用；
  // 原有调用点忽略返回值，行为不变）。
  size_t popUntil(long long ts) {
    std::lock_guard<std::mutex> lock(mtx_);
    size_t dropped = 0;
    while (!queue_.empty() && queue_.front()->timestamp < ts) {
      queue_.pop_front();
      ++dropped;
    }
    if (dropped > 0) {
      dropped_total_.fetch_add(dropped, std::memory_order_relaxed);
    }
    return dropped;
  }

  // 累计丢弃帧数（满溢丢老帧 + popUntil 丢帧），诊断直接读取，无需推算。
  uint64_t dropped_total() const {
    return dropped_total_.load(std::memory_order_relaxed);
  }

 private:
  std::deque<std::shared_ptr<VideoBuffer>> queue_;
  std::mutex mtx_;
  size_t max_size_;
  std::atomic<uint64_t> dropped_total_{0};
};

// popen运行cmd，并获取cmd返回结果
int exec_cmd_ex(const char *cmd, char *res, int max);


}  // namespace mipi_cam
#endif  // MIPI_MIPI_COMM_HPP_
