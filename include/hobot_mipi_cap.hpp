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

#ifndef HOBOT_MIPI_CAP_HPP_
#define HOBOT_MIPI_CAP_HPP_

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <queue>
#include <thread>
#include <mutex>
#if defined(PLATFORM_S100) || defined(PLATFORM_S600)
#include "hb_gdc_cfg.h"
#include "gdc_cfg.h"
#else
#include "gdc_cfg.h"
#include "gdc_bin_cfg.h"
#endif  
#include "hobot_mipi_comm.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "hobot_mipi_calibration.hpp"
#include "opencv2/opencv.hpp"

namespace mipi_cam {

typedef struct gdc_binbuf_s {
  hb_mem_common_buf_t *bin_buf = nullptr;
  uint64_t bin_buf_size;
  ~gdc_binbuf_s() {
    if (bin_buf != NULL) {
      hb_mem_free_buf(bin_buf->fd);
      bin_buf = NULL;
    }
  }
}GdcBinBuf_ST;

class HobotMipiCap {
 public:
  HobotMipiCap(/* args */) {}
  virtual ~HobotMipiCap() {}

  // 初始化设备环境，如X3的sensor GPIO配置和时钟配置
  // 返回值：0，成功；-1，配置失败
  virtual int initEnv() = 0;

  // 初始化相关sensor的VIO pipeline；
  // 输入参数：MIPI_CAP_INFO_ST的结构信息。
  // 返回值：0，初始化成功；-1，初始化失败。
  virtual int init(MIPI_CAP_INFO_ST &info) = 0;

  // 反初始化相关sensor的VIO pipeline ；
  // 返回值：0，反初始化成功；-1，反初始化失败。
  virtual int deInit() = 0;

  // 启动相关sensor的VIO pipeline的码流；
  // 返回值：0，启动成功；-1，启动失败。
  virtual int start() = 0;

  // 停止相关sensor的VIO pipeline的码流；
  // 返回值：0，停止成功；-1，停止失败。
  virtual int stop() = 0;

  // 如果有 vps ，就 输出vps 的分层数据 channel--"single":单sensor，"sub_single":单sensor的子码流， "left": 双目的左sensor，"right":双目的右sensor，"combine"：左右sensor拼合的图像。
  // "sub_left": 双目的左sensor的子码流，"sub_right":双目的右sensor的子码流，"sub_combine"：左右sensor的子码流拼合的图像。
  virtual std::shared_ptr<VideoBuffer> getFrame(std::string channel) = 0;


  // 获取cap的info信息；
  // 输入输出参数：MIPI_CAP_INFO_ST的结构信息。
  // 返回值：0，初始化成功；-1，初始化失败。
  virtual int getCapInfo(MIPI_CAP_INFO_ST &info) {
    info = cap_info_;
    return 0;
  }

  virtual int setCamInfo(std::vector<sensor_msgs::msg::CameraInfo> info) {
    cam_info_ = info;
    return 0;
  }

  virtual std::vector<sensor_msgs::msg::CameraInfo>* getCalCamInfo() {
    return &cal_cam_info_;
  }

  virtual std::vector<sensor_msgs::msg::CameraInfo>* getCalCamInfoSub()
  {
    return &cal_cam_info_sub_;
  }

#if 1
 protected:
  MIPI_CAP_INFO_ST cap_info_;
  std::vector<sensor_msgs::msg::CameraInfo> cam_info_;
  std::vector<sensor_msgs::msg::CameraInfo> cal_cam_info_;
  std::vector<sensor_msgs::msg::CameraInfo> cal_cam_info_sub_;

  std::shared_ptr<std::thread> sync_task_ = nullptr;
  std::shared_ptr<std::thread> sub_sync_task_ = nullptr;

  std::vector<std::shared_ptr<BuffQueueManage>> v_buff_que_manger_;
  std::shared_ptr<BuffQueueManage> combine_buff_que_manger_;
  std::vector<std::shared_ptr<FrameQueue>> v_frame_que_;
  std::vector<std::shared_ptr<BuffQueueManage>> v_sub_buff_que_manger_;
  std::shared_ptr<BuffQueueManage> sub_combine_buff_que_manger_;
  std::vector<std::shared_ptr<FrameQueue>> v_sub_frame_que_;

  void free_gdc_cfg(void *buf_ptr);
  int32_t gen_gdc_cfg(param_t *param, window_t *wnds, uint32_t wnd_num, void **cfg_buf, uint64_t *cfg_size);
  virtual void sync_task();
  virtual void sub_sync_task();
  virtual bool isSynced(const std::vector<std::shared_ptr<VideoBuffer>> &frames, long long tolerance);

  virtual std::shared_ptr<GdcBinBuf_ST> get_gdc_bin(std::string gdc_bin_file);
  virtual std::vector<std::shared_ptr<GdcBinBuf_ST>> gen_gdc_bin_stereo(int gdc_width, int gdc_height,int out_width, int out_height, 
		std::vector<sensor_msgs::msg::CameraInfo> &cam_info, std::vector<sensor_msgs::msg::CameraInfo> &cal_cam_info,
    double rotation = 0.0, double cal_rotate = 0.0, double cla_alpha = 0.0, bool pre_rotation = false);
  virtual std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin(int gdc_width, int gdc_height,int out_width, int out_height,
       sensor_msgs::msg::CameraInfo *cam_info, sensor_msgs::msg::CameraInfo *cal_cam_info,
       double rotation = 0.0, double cal_rotate = 0.0, double cla_alpha = 0.0, bool pre_rotation = false);
  virtual std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin_rotation(int gdc_width, int gdc_height,int out_width, int out_height, double rotation);
  virtual std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin_json(std::string file);

  virtual std::pair<double, double> calculatePinholeFOV(const cv::Mat& K_rect, int width, int hight);
  virtual double computeInitAlpha(double target_hfov, double fov_min, double fov_max);
  virtual double computeStereoAlphaFromFOV(double target_hfov, const cv::Mat& Kl, const cv::Mat& Dl,
    const cv::Mat& Kr, const cv::Mat& Dr, const cv::Mat& R_rl, const cv::Mat& t_rl,
    int in_gdc_width, int in_gdc_height, int out_gdc_width, int out_gdc_height, double& actual_hfov_l, double& actual_hfov_r);
  virtual double find_best_fov_scale(const cv::Mat& Kl, const cv::Mat& Dl, const cv::Mat& Kr, const cv::Mat& Dr,
      const cv::Mat& R_rl, const cv::Mat& t_rl, cv::Size in_size, cv::Size out_size);

  virtual bool computeFisheyeStereoParamsFromFOV(
      double target_hfov,
      const cv::Mat &Kl, const cv::Mat &Dl,
      const cv::Mat &Kr, const cv::Mat &Dr,
      const cv::Mat &R_rl, const cv::Mat &t_rl,
      cv::Size in_size, cv::Size out_size,
      double &out_balance, double &out_fov_scale,
      double &actual_hfov_l, double &actual_hfov_r,
      double tol = 3.0,
      int max_iter = 10);
#endif
};

}  // namespace mipi_cam


#endif  // HOBOT_MIPI_CAP_HPP_
