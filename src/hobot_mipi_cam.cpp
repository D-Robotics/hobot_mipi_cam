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

#define __STDC_CONSTANT_MACROS
#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cam.hpp"
#include "hobot_mipi_factory.hpp"

#include "sensor_msgs/distortion_models.hpp"
#include <rclcpp/rclcpp.hpp>

#include <errno.h>
#include <malloc.h>
#include <unistd.h>

#include <assert.h>
#include <fcntl.h> /* low-level i/o */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include <opencv2/opencv.hpp>
#ifdef PLATFORM_X5
#include "GC820/nano2D.h"
#include "GC820/nano2D_util.h"
#endif

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace mipi_cam {

// popen运行cmd，并获取cmd返回结果
int exec_cmd_ex(const char *cmd, char* res, int max) {
  if (cmd == NULL || res == NULL || max <= 0)
    return -1;
  FILE *pp = popen(cmd, "r");
  if (!pp) {
   RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
     "error, cannot popen cmd: %s\n", cmd);
    return -1;
  }
  int length;
  char tmp[1024] = {0};
  length = max;
  if (max > 1024)
    length = 1024;
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "[%s]->cmd %s, fp=0x%x, len=%d.\n", __func__, cmd, pp, max);
  while (fgets(tmp, length, pp) != NULL) {
    // printf("exec_cmd_ex -- tmp:%s\n", tmp);
    sscanf(tmp, "%s", res);
  }
  pclose(pp);
  return strlen(res);
}

class MipiCamIml : public MipiCam {
 public:
  MipiCamIml();
  ~MipiCamIml();

  // 初始化摄像机
  // 输入参数：para是node传入的参数，包括sensor类型、名称，图像的宽、高等等。
  // 返回值：0，初始化成功，-1，初始化失败。
  int init(struct NodePara &para);

  // 反初始化摄像机；
  // 返回值：0，反初始化成功；-1，反初始化失败。
  int deInit();

  // 启动摄像机的码流；
  // 返回值：0，启动成功；-1，启动失败。
  int start();

  // 停止摄像机的码流；
  // 返回值：0，停止成功；-1，停止失败。
  int stop();

  // grabs a new image from the camera
  bool getImage(
    builtin_interfaces::msg::Time & stamp,
    std::string & encoding,
    uint32_t & height, uint32_t & width,
    uint32_t & step, std::vector<uint8_t> & data, std::string channel);

  // grabs a new image from the camera
  bool getCombineImage(
    builtin_interfaces::msg::Time & stamp,
    std::string & encoding,
    uint32_t & height, uint32_t & width,
    uint32_t & step, std::vector<uint8_t> & data);

  // grabs a new hbmem's image hbmem from the camera
  bool getImageMem(
    builtin_interfaces::msg::Time & stamp,
    std::array<uint8_t, 12> & encoding,
    uint32_t & height, uint32_t & width, uint32_t & step,
    std::array<uint8_t, 6220800> & data, uint32_t & data_size, std::string channel);

    // grabs a new hbmem's image hbmem from the camera
  bool getCombineImageMem(
    builtin_interfaces::msg::Time & stamp,
    std::array<uint8_t, 12> & encoding,
    uint32_t & height, uint32_t & width, uint32_t & step,
    std::array<uint8_t, 6220800> & data, uint32_t & data_size);

  // gen camera calibration
  bool getCamCalibration(sensor_msgs::msg::CameraInfo& cam_info,
                           const std::string &file_path);
  
  bool getDualCamCalibration(sensor_msgs::msg::CameraInfo &cam_info_l,
                sensor_msgs::msg::CameraInfo &cam_info_r, const std::string &file_path);

  bool isCapturing();

 private:
  inline void NV12_TO_BGR24(unsigned char *_src, unsigned char *_RGBOut, int width, int height);

  typedef struct camera_image_s {
    int width;
    int height;
    int image_size;
    char * image;
    ~camera_image_s() {
      if (image != NULL) {
        free(image);
        image = NULL;
      }
    }
  } camera_image_t;

  camera_image_t *image_nv12_ = nullptr;
  bool lsInit_;
  bool is_capturing_;
  std::shared_ptr<HobotMipiCap> mipiCap_ptr_;
  struct NodePara nodePare_;
  MIPI_CAP_INFO_ST cap_info_;
};

std::shared_ptr<MipiCam> MipiCam::create_mipicam() {
  return std::make_shared<MipiCamIml>();
}

MipiCamIml::MipiCamIml()
    : lsInit_(false),
      is_capturing_(false) {
}

MipiCamIml::~MipiCamIml() {
  stop();
  deInit();
}



int MipiCamIml::init(struct NodePara &para) {
  if (lsInit_) {
    return 0;
  }
  memcpy(&nodePare_, &para, sizeof(nodePare_));
  auto board_type = getBoardType();

  mipiCap_ptr_ = createMipiCap(board_type);
  if (!mipiCap_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s]->cap %s create capture failture.\r\n",
      __func__, board_type.c_str());
    return -1;
  }
  cap_info_.config_path = nodePare_.config_path_;
  cap_info_.sensor_type = nodePare_.video_device_name_;
  cap_info_.width = nodePare_.image_width_;
  cap_info_.height = nodePare_.image_height_;
  cap_info_.fps = nodePare_.framerate_;
  cap_info_.channel_ = nodePare_.channel_;
  cap_info_.device_mode_ = nodePare_.device_mode_;
  cap_info_.dual_combine_ = nodePare_.dual_combine_;
  cap_info_.gdc_bin_file_ = nodePare_.gdc_bin_file_;

  if (mipiCap_ptr_->initEnv() < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
    "[%s]->init %s's mipi host and gpio failure: %s\r\n", __func__, board_type.c_str());
    return -1;
  }

  if (mipiCap_ptr_->init(cap_info_) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s]->cap capture init failture.\r\n", __func__);
    return -5;
  }
  mipiCap_ptr_->getCapInfo(cap_info_);
  nodePare_.image_width_ = cap_info_.width;
  nodePare_.image_height_ = cap_info_.height;

  RCLCPP_WARN(rclcpp::get_logger("mipi_cam"),
    "[%s]->cap %s init success.\r\n", __func__, cap_info_.sensor_type.c_str());
  lsInit_ = true;
  return 0;
}

int MipiCamIml::deInit() {
  int ret = 0;
  if (lsInit_) {
    lsInit_ = false;
    if (true == is_capturing_) {
      stop();
    }
    if (image_nv12_ != nullptr) {
      free(image_nv12_);
      image_nv12_ = nullptr;
    }
    ret = mipiCap_ptr_->deInit();
    mipiCap_ptr_ = nullptr;
  }

  return ret;
}

int MipiCamIml::start() {
  if (!lsInit_ || is_capturing_) {
    return -1;
  }
  int ret = 0;
  if (mipiCap_ptr_->start()) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s]->cap capture start failture.\r\n", __func__);
  }

  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
              "[%s]->w:h=%d:%d.\n",
              __func__,
              nodePare_.image_width_,
              nodePare_.image_height_);
  is_capturing_ = true;
  if (nodePare_.out_format_name_ == "bgr8") {
    image_nv12_ = reinterpret_cast<camera_image_t *>(calloc(1, sizeof(camera_image_t)));
    image_nv12_->width = nodePare_.image_width_;
    image_nv12_->height = nodePare_.image_height_;
    image_nv12_->image_size = nodePare_.image_width_ * nodePare_.image_height_ * 1.5 * 2;
    image_nv12_->image = reinterpret_cast<char *>(calloc(image_nv12_->image_size, sizeof(char *)));
  }
  return ret;
}

int MipiCamIml::stop() {
  int ret = 0;
  if (true == is_capturing_) {
    is_capturing_ = false;
    ret = mipiCap_ptr_->stop();
  }
  is_capturing_ = false;
  RCLCPP_WARN(rclcpp::get_logger("mipi_cam"),
    "mipi_cam is stoped");
  return ret;
}

bool MipiCamIml::isCapturing() { return is_capturing_; }

bool MipiCamIml::getImage(builtin_interfaces::msg::Time &stamp,
                        std::string &encoding,
                        uint32_t &height,
                        uint32_t &width,
                        uint32_t &step,
                        std::vector<uint8_t> &data, std::string channel) {
  if (!is_capturing_) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s][%-%d] Camera isn't captureing", __FILE__, __func__, __LINE__);
    return false;
  }
  if ((nodePare_.image_width_ == 0) || (nodePare_.image_height_ == 0)) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "Invalid publish width:%d height: %d! Please check the image_width "
      "and image_height parameters!",
      nodePare_.image_width_,
      nodePare_.image_height_);
    return false;
  }
  struct timespec time_start = {0, 0};
  int64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  uint64_t timestamp;
  int data_size;
  if (channel == "combine") {
    data_size = nodePare_.image_width_ * nodePare_.image_height_ * 1.5 * 2;
  } else {
    data_size = nodePare_.image_width_ * nodePare_.image_height_ * 1.5;
  }
  if ((nodePare_.out_format_name_ == "bgr8") && image_nv12_) {
    if (mipiCap_ptr_->getFrame(channel,
          reinterpret_cast<int *>(&width),
          reinterpret_cast<int *>(&height),
          reinterpret_cast<void *>(image_nv12_->image),
          image_nv12_->image_size,
          reinterpret_cast<unsigned int *>(&data_size),
          timestamp)) {
      return false;
    }
    data_size = width * height * 3;
    data.resize(data_size);  // step * height);

    uint64_t msStart_bgr = 0, msEnd_bgr = 0;
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msStart_bgr = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    NV12_TO_BGR24((unsigned char *)image_nv12_->image,
                  (unsigned char *)&data[0], width, height);
    encoding = "bgr8";
    step = width * 3;
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msEnd_bgr = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
            "NV12_TO_BGR24 laps ms= %d", (msEnd_bgr - msStart_bgr));
  } else if (nodePare_.out_format_name_ == "gray") {
    data_size = nodePare_.image_width_ * nodePare_.image_height_;
    data.resize(data_size);  // step * height);
    if (mipiCap_ptr_->getFrame(channel,
          reinterpret_cast<int *>(&width),
          reinterpret_cast<int *>(&height),
          reinterpret_cast<void *>(&data[0]),
          data_size,
          reinterpret_cast<unsigned int *>(&data_size),
          timestamp, true))
      return false;
    encoding = "mono8";
    step = width;
  } else {
    data.resize(data_size);  // step * height);
    if (mipiCap_ptr_->getFrame(channel,
          reinterpret_cast<int *>(&width),
          reinterpret_cast<int *>(&height),
          reinterpret_cast<void *>(&data[0]),
          data_size,
          reinterpret_cast<unsigned int *>(&data_size),
          timestamp))
      return false;
    encoding = "nv12";
    step = width;
  }
  stamp.sec = timestamp / 1e9;
  stamp.nanosec = timestamp - stamp.sec * 1e9;

  uint64_t timestamp_sys;
  {
    struct timeval tv;
	  gettimeofday(&tv, NULL);
    timestamp_sys = (tv.tv_sec * 1000 + tv.tv_usec/1000);
  }

  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
            "publish laps ms= %d", (timestamp_sys - timestamp/1000000));

  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_cam"),
             "getImage channel=" << channel.data()
             << ", enc=" << encoding.data()
             << ", width=" << width
             << ", height=" << height
             << ", step=" << step
             << ", sz=" << data_size
             << ", ts=" << stamp.sec << "." << stamp.nanosec
              << ", laps ms=" << msEnd - msStart);

  return true;
}


bool MipiCamIml::getImageMem(
    builtin_interfaces::msg::Time &stamp,
    std::array<uint8_t, 12> &encoding,
    uint32_t &height,
    uint32_t &width,
    uint32_t &step,
    std::array<uint8_t, 6220800> &data,
    uint32_t &data_size, std::string channel) {
  if (!is_capturing_) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s][%-%d] Camera isn't captureing", __FILE__, __func__, __LINE__);
    return false;
  }
  if ((nodePare_.image_width_ == 0) || (nodePare_.image_height_ == 0)) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "Invalid publish width:%d height: %d! Please check the image_width "
      "and image_height parameters!",
      nodePare_.image_width_,
      nodePare_.image_height_);
    return false;
  }
  // get the image
  struct timespec time_start = {0, 0};
  uint64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  uint64_t timestamp;
  data_size = nodePare_.image_width_ * nodePare_.image_height_ * 1.5;
  if ((nodePare_.out_format_name_ == "bgr8") && image_nv12_) {
    if (mipiCap_ptr_->getFrame(channel,
          reinterpret_cast<int *>(&width),
          reinterpret_cast<int *>(&height),
          reinterpret_cast<void *>(image_nv12_->image),
          image_nv12_->image_size,
          reinterpret_cast<unsigned int *>(&data_size),
          timestamp))
      return false;
    data_size = width * height * 3;
    if (data_size > 6220800) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "rgb image data size %d > HbmMsg1080P size(6220800)", data_size);
      return false;
    }

    uint64_t msStart_bgr = 0, msEnd_bgr = 0;
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msStart_bgr = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }

    NV12_TO_BGR24((unsigned char *)image_nv12_->image,
                  (unsigned char *)data.data(), width, height);
    memcpy(encoding.data(), "bgr8", strlen("bgr8"));
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      msEnd_bgr = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
            "NV12_TO_BGR24 laps ms= %d", (msEnd_bgr - msStart_bgr));
  } else if (nodePare_.out_format_name_ == "gray") {
    if (mipiCap_ptr_->getFrame(channel,
          reinterpret_cast<int *>(&width),
          reinterpret_cast<int *>(&height),
          reinterpret_cast<void *>(data.data()),
          6220800,
          reinterpret_cast<unsigned int *>(&data_size),
          timestamp, true))
    return false;
     memcpy(encoding.data(), "mono8", strlen("mono8"));
  } else {
    if (mipiCap_ptr_->getFrame(channel,
          reinterpret_cast<int *>(&width),
          reinterpret_cast<int *>(&height),
          reinterpret_cast<void *>(data.data()),
          6220800,
          reinterpret_cast<unsigned int *>(&data_size),
          timestamp))
    return false;
     memcpy(encoding.data(), "nv12", strlen("nv12"));
  }
  stamp.sec = timestamp / 1e9;
  stamp.nanosec = timestamp - stamp.sec * 1e9;
  step = width;

  uint64_t timestamp_sys;
  {
    struct timeval tv;
	  gettimeofday(&tv, NULL);
    timestamp_sys = (tv.tv_sec * 1000 + tv.tv_usec/1000);
  }

  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
            "publish laps ms= %d", (timestamp_sys - timestamp/1000000));

  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_cam"),
             "getImageMem channel=" << channel.data()
             << ", enc=" << encoding.data()
             << ", width=" << width
             << ", height=" << height
             << ", step=" << step
             << ", sz=" << data_size
             << ", ts=" << stamp.sec << "." << stamp.nanosec
              << ", laps ms=" << msEnd - msStart);
  return true;
}

bool MipiCamIml::getCamCalibration(sensor_msgs::msg::CameraInfo &cam_info,
                                  const std::string &file_path) {

  try {
    std::string cal_file;
    if ((file_path.length() == 0) || (file_path == "default")) {
      MIPI_CAP_INFO_ST cap_info;
      mipiCap_ptr_->getCapInfo(cap_info);
      std::string sensor_name = cap_info.sensor_type;
      std::transform(sensor_name.begin(), sensor_name.end(), sensor_name.begin(), [](unsigned char c){
        return std::toupper(c);
      });
      cal_file = cap_info.config_path + "/" + sensor_name + "_calibration.yaml";
    } else {
      cal_file = file_path;
    }
    std::string camera_name;
    std::ifstream fin(cal_file.c_str());
    if (!fin) {
      RCLCPP_WARN(rclcpp::get_logger("mipi_cam"),
          "Camera calibration file: %s is not exist!"
          "\nIf you need calibration msg, please make sure the calibration file path is correct and the calibration file exists!",
          cal_file.c_str());
      return false;
    }
    YAML::Node calibration_doc = YAML::Load(fin);
    if (calibration_doc["camera_name"]) {
      camera_name = calibration_doc["camera_name"].as<std::string>();
    } else {
      camera_name = "unknown";
    }
    cam_info.width = calibration_doc["image_width"].as<int>();
    cam_info.height = calibration_doc["image_height"].as<int>();

    const YAML::Node &camera_matrix = calibration_doc["camera_matrix"];
    const YAML::Node &camera_matrix_data = camera_matrix["data"];
    for (int i = 0; i < 9; i++) {
      cam_info.k[i] = camera_matrix_data[i].as<double>();
    }
    const YAML::Node &rectification_matrix =
        calibration_doc["rectification_matrix"];
    const YAML::Node &rectification_matrix_data = rectification_matrix["data"];
    for (int i = 0; i < 9; i++) {
      cam_info.r[i] = rectification_matrix_data[i].as<double>();
    }
    const YAML::Node &projection_matrix = calibration_doc["projection_matrix"];
    const YAML::Node &projection_matrix_data = projection_matrix["data"];
    for (int i = 0; i < 12; i++) {
      cam_info.p[i] = projection_matrix_data[i].as<double>();
    }

    if (calibration_doc["distortion_model"]) {
      cam_info.distortion_model =
          calibration_doc["distortion_model"].as<std::string>();
    } else {
      cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
                 "Camera calibration file did not specify distortion model, "
                  "assuming plumb bob");
    }
    const YAML::Node &distortion_coefficients =
        calibration_doc["distortion_coefficients"];
    int d_rows, d_cols;
    d_rows = distortion_coefficients["rows"].as<int>();
    d_cols = distortion_coefficients["cols"].as<int>();
    const YAML::Node &distortion_coefficients_data =
        distortion_coefficients["data"];
    cam_info.d.resize(d_rows * d_cols);
    for (int i = 0; i < d_rows * d_cols; ++i) {
      cam_info.d[i] = distortion_coefficients_data[i].as<double>();
    }
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
      "[getCamCalibration]->parse calibration file successfully");
    return true;
  } catch (YAML::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "Unable to parse camera calibration file normally:%s",
      e.what());
    return false;
  }
}

bool MipiCamIml::getDualCamCalibration(sensor_msgs::msg::CameraInfo &cam_info_l,sensor_msgs::msg::CameraInfo &cam_info_r,
                                  const std::string &file_path) {

  try {
    std::string cal_file = file_path;
    std::string camera_name;
    std::ifstream fin(cal_file.c_str());
    if (!fin) {
      RCLCPP_WARN(rclcpp::get_logger("mipi_cam"),
          "Camera calibration file: %s is not exist!"
          "\nIf you need calibration msg, please make sure the calibration file path is correct and the calibration file exists!",
          cal_file.c_str());
      return false;
    }
    YAML::Node calibration_doc = YAML::Load(fin);
    const YAML::Node &cal_l_doc = calibration_doc["cam0"];
    const YAML::Node &T_cn_cnm1 = calibration_doc["T_cn_cnm1"];

    cam_info_l.width = cal_l_doc["resolution"][0].as<int>();
    cam_info_l.height = cal_l_doc["resolution"][1].as<int>();

    cam_info_l.d.resize(cal_l_doc["distortion_coeffs"].size());
    for (int i = 0; i < cal_l_doc["distortion_coeffs"].size(); ++i) {
      cam_info_l.d[i] = cal_l_doc["distortion_coeffs"][i].as<double>();
    }

    cam_info_l.k[0] = cal_l_doc["intrinsics"][0].as<double>();
    cam_info_l.k[1] = 0.0;
    cam_info_l.k[2] = cal_l_doc["intrinsics"][1].as<double>();
    cam_info_l.k[3] = 0.0;
    cam_info_l.k[4] = cal_l_doc["intrinsics"][2].as<double>();
    cam_info_l.k[5] = cal_l_doc["intrinsics"][3].as<double>();
    cam_info_l.k[6] = 0.0;
    cam_info_l.k[7] = 0.0;
    cam_info_l.k[8] = 1.0;

    cam_info_l.r[0] = 1.0;
    cam_info_l.r[1] = 0.0;
    cam_info_l.r[2] = 0.0;
    cam_info_l.r[3] = 0.0;
    cam_info_l.r[4] = 1.0;
    cam_info_l.r[5] = 0.0;
    cam_info_l.r[6] = 0.0;
    cam_info_l.r[7] = 0.0;
    cam_info_l.r[8] = 1.0;

    cam_info_l.p[0] = cal_l_doc["intrinsics"][0].as<double>();
    cam_info_l.p[1] = 0.0;
    cam_info_l.p[2] = cal_l_doc["intrinsics"][1].as<double>();
    cam_info_l.p[3] = 0.0;
    cam_info_l.p[4] = 0.0;
    cam_info_l.p[5] = cal_l_doc["intrinsics"][2].as<double>();
    cam_info_l.p[6] = cal_l_doc["intrinsics"][3].as<double>();
    cam_info_l.p[7] = 0.0;
    cam_info_l.p[8] = 0.0;
    cam_info_l.p[9] = 0.0;
    cam_info_l.p[10] = 1.0;
    cam_info_l.p[11] = 0.0;

    const YAML::Node &cal_r_doc = calibration_doc["cam1"];

    cam_info_r.width = cal_r_doc["resolution"][0].as<int>();
    cam_info_r.height = cal_r_doc["resolution"][1].as<int>();

    cam_info_r.d.resize(cal_r_doc["distortion_coeffs"].size());
    for (int i = 0; i < cal_r_doc["distortion_coeffs"].size(); ++i) {
      cam_info_r.d[i] = cal_r_doc["distortion_coeffs"][i].as<double>();
    }

    cam_info_r.k[0] = cal_r_doc["intrinsics"][0].as<double>();
    cam_info_r.k[1] = 0.0;
    cam_info_r.k[2] = cal_r_doc["intrinsics"][1].as<double>();
    cam_info_r.k[3] = 0.0;
    cam_info_r.k[4] = cal_r_doc["intrinsics"][2].as<double>();
    cam_info_r.k[5] = cal_r_doc["intrinsics"][3].as<double>();
    cam_info_r.k[6] = 0.0;
    cam_info_r.k[7] = 0.0;
    cam_info_r.k[8] = 1.0;

    cam_info_r.r[0] = T_cn_cnm1[0][0].as<double>();
    cam_info_r.r[1] = T_cn_cnm1[0][1].as<double>();
    cam_info_r.r[2] = T_cn_cnm1[0][2].as<double>();
    cam_info_r.r[3] = T_cn_cnm1[1][0].as<double>();
    cam_info_r.r[4] = T_cn_cnm1[1][1].as<double>();
    cam_info_r.r[5] = T_cn_cnm1[1][2].as<double>();
    cam_info_r.r[6] = T_cn_cnm1[2][0].as<double>();
    cam_info_r.r[7] = T_cn_cnm1[2][1].as<double>();
    cam_info_r.r[8] = T_cn_cnm1[2][2].as<double>();

    cam_info_r.p[0] = cal_r_doc["intrinsics"][0].as<double>();
    cam_info_r.p[1] = 0.0;
    cam_info_r.p[2] = cal_r_doc["intrinsics"][1].as<double>();
    cam_info_r.p[3] = 0.0;
    cam_info_r.p[4] = 0.0;
    cam_info_r.p[5] = cal_r_doc["intrinsics"][2].as<double>();
    cam_info_r.p[6] = cal_r_doc["intrinsics"][3].as<double>();
    cam_info_r.p[7] = 0.0;
    cam_info_r.p[8] = 0.0;
    cam_info_r.p[9] = 0.0;
    cam_info_r.p[10] = 1.0;
    cam_info_r.p[11] = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
      "[getCamCalibration]->parse calibration file successfully");
    return true;
  } catch (YAML::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "Unable to parse camera calibration file normally:%s",
      e.what());
    return false;
  }
}

#ifdef PLATFORM_X5

inline void MipiCamIml::NV12_TO_BGR24(unsigned char *_src, unsigned char *_RGBOut, int width, int height) {
#if 0
  do {
    int error = n2d_open();
    if (N2D_IS_ERROR(error)) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),"open context failed! error=%d.\n", error);
      break;
    }
    std::shared_ptr<n2d_buffer_t> src1 = std::make_shared<n2d_buffer_t>();
    std::shared_ptr<n2d_buffer_t> src2 = std::make_shared<n2d_buffer_t>();
    /* switch to default device and core */
    error = n2d_switch_device(N2D_DEVICE_0);
    if (N2D_IS_ERROR(error))
    {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),"n2d_switch_device failed! error=%d.\n", error);
      break;
    }
    error = n2d_switch_core(N2D_CORE_0);
    if (N2D_IS_ERROR(error)) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),"n2d_switch_core failed! error=%d.\n", error);
      break;
    }

    error = n2d_util_allocate_buffer(width, height, N2D_NV12, N2D_0, N2D_LINEAR, N2D_TSC_DISABLE, src1.get());
    if (N2D_IS_ERROR(error))
    {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),"n2d_util_allocate_buffer, error=%d.\n", error);
      break;
    }
    memcpy(src1->memory, _src, width * height * 1.5);

    error = n2d_util_allocate_buffer(width, height, N2D_RGB888, N2D_0, N2D_LINEAR, N2D_TSC_DISABLE, src2.get());
    if (N2D_IS_ERROR(error)) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),"n2d_util_allocate_buffer, error=%d.\n", error);
      n2d_free(src1.get());
      break;
    }

    error = n2d_blit(src2.get(), N2D_NULL, src1.get(), N2D_NULL, N2D_BLEND_NONE);
    if (N2D_IS_ERROR(error)) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),"blit error, error=%d.\n", error);
      n2d_free(src1.get());
      n2d_free(src2.get());
      break;
    }
    error = n2d_commit();
    if (N2D_IS_ERROR(error)) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),"blit error, error=%d.\n", error);
      n2d_free(src1.get());
      n2d_free(src2.get());
      break;
    }
    memcpy(_RGBOut, src2->memory,  width * height * 3);
    n2d_free(src1.get());
    n2d_free(src2.get());
    return;
  } while(0);
#endif
  cv::Mat src(height * 3 / 2, width, CV_8UC1, (void*)_src);
  cv::Mat bgr_mat;
  cv::cvtColor(src, bgr_mat, cv::COLOR_YUV2BGR_NV12);
  memcpy(_RGBOut, bgr_mat.ptr<uint8_t>(), height * width * 3);
  return;
}
#else

#include <arm_neon.h>
const uint8_t Y_SUBS[8] = { 16, 16, 16, 16, 16, 16, 16, 16 };
const uint8_t UV_SUBS[8] = { 128, 128, 128, 128, 128, 128, 128, 128 };

inline void MipiCamIml::NV12_TO_BGR24(unsigned char *_src, unsigned char *_RGBOut, int width, int height) {
  unsigned char *src = (unsigned char*)_src;
  unsigned char *RGBOut = (unsigned char*)_RGBOut;

  int i, j;
  int nWH = width * height;
  unsigned char *pY1 = src;
  unsigned char *pY2 = src + width;
  unsigned char *pUV = src + nWH;

  uint8x8_t Y_SUBvec = vld1_u8(Y_SUBS);
  uint8x8_t UV_SUBvec = vld1_u8(UV_SUBS);

  // int width2 = width >> 1;
  int width3 = (width << 2) - width;
  int width9 = (width << 3) + width;
  unsigned char *RGBOut1 = RGBOut;
  unsigned char *RGBOut2 = RGBOut1 + width3;
  // unsigned char *RGBOut1 = RGBOut + 3 * width * (height - 2);
  // unsigned char *RGBOut2 = RGBOut1 + width3;

  unsigned char tempUV[8];
  // YUV 4:2:0
  // #pragma omp parallel for num_threads(4)
  for (j = 0; j < height; j += 2) {
      for (i = 0; i < width; i += 8) {
          tempUV[0] = pUV[1];
          tempUV[1] = pUV[3];
          tempUV[2] = pUV[5];
          tempUV[3] = pUV[7];

          tempUV[4] = pUV[0];
          tempUV[5] = pUV[2];
          tempUV[6] = pUV[4];
          tempUV[7] = pUV[6];

          pUV += 8;
          uint8x8_t nUVvec = vld1_u8(tempUV);
          int16x8_t nUVvec16 = vmovl_s8((int8x8_t)vsub_u8(nUVvec, UV_SUBvec));  // 减后区间-128到127
          int16x4_t V_4 = vget_low_s16((int16x8_t)nUVvec16);
          int16x4x2_t V16x4x2 = vzip_s16(V_4, V_4);
          // int16x8_t V16x8_;
          // memcpy(&V16x8_, &V16x4x2, 16);
          // int16x8_t* V16x8 = (int16x8_t*)(&V16x8_);
          int16x8_t* V16x8 = reinterpret_cast<int16x8_t*>(&V16x4x2);
          int16x4_t U_4 = vget_high_s16(nUVvec16);
          int16x4x2_t U16x4x2 = vzip_s16(U_4, U_4);
          int16x8_t* U16x8 = reinterpret_cast<int16x8_t*>(&U16x4x2);

          // 公式1
          int16x8_t VV1 = vmulq_n_s16(*V16x8, 102);
          int16x8_t UU1 = vmulq_n_s16(*U16x8, 129);
          int16x8_t VVUU1 = vmlaq_n_s16(vmulq_n_s16(*V16x8, 52), *U16x8, 25);

          uint8x8_t nYvec;
          uint8x8x3_t RGB;
          uint16x8_t Y16;
          // 上行
          nYvec = vld1_u8(pY1);
          pY1 += 8;
          // 公式1
          Y16 = vmulq_n_u16(vmovl_u8(vqsub_u8(nYvec, Y_SUBvec)), 74);  // 公式1

          RGB.val[0] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)UU1), 6));
          RGB.val[1] = vqmovun_s16(vshrq_n_s16((int16x8_t)vsubq_u16(Y16, (uint16x8_t)VVUU1), 6));
          RGB.val[2] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)VV1), 6));
          vst3_u8(RGBOut1, RGB);
          RGBOut1 += 24;

          // 下行
          nYvec = vld1_u8(pY2);
          pY2 += 8;
          // 公式1
          Y16 = vmulq_n_u16(vmovl_u8(vqsub_u8(nYvec, Y_SUBvec)), 74);  // 公式1
          RGB.val[0] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)UU1), 6));
          RGB.val[1] = vqmovun_s16(vshrq_n_s16((int16x8_t)vsubq_u16(Y16, (uint16x8_t)VVUU1), 6));
          RGB.val[2] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)VV1), 6));
          vst3_u8(RGBOut2, RGB);
          RGBOut2 += 24;
      }
      RGBOut1 += width3;
      RGBOut2 += width3;
      // RGBOut1 -= width9;
      // RGBOut2 -= width9;
      pY1 += width;
      pY2 += width;
  }
}
#endif

}  // namespace mipi_cam
