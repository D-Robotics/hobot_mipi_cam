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

#ifndef HOBOT_MIPI_CAP_IML_HPP_
#define HOBOT_MIPI_CAP_IML_HPP_
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <queue>
#include <thread>
#include <mutex>
#include "hobot_mipi_cap.hpp"
#include "hobot_mipi_comm.hpp"
#include "hb_camera_interface.h"
#include "hbn_api.h"
#include "vp_sensors.h"
#include "vse_cfg.h"
#include "gdc_cfg.h"
#include "gdc_bin_cfg.h"
#include "codec_cfg.h"
#include "GC820/nano2D.h"
#include "GC820/nano2D_util.h"
#include "opencv2/opencv.hpp"
#include "hobot_mipi_calibration.hpp"

namespace mipi_cam {

#define PIPES_TOTAL 1

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

typedef struct pipe_contex_s {
	hbn_vflow_handle_t vflow_fd;
	hbn_vnode_handle_t vin_node_handle;
	hbn_vnode_handle_t isp_node_handle;
	hbn_vnode_handle_t vse_node_handle;
	hbn_vnode_handle_t gdc_node_handle;
  hbn_vnode_handle_t gdc_node_handle_r;
	hbn_vnode_handle_t vpu_node_handle;
  hbn_vnode_handle_t stream_handle;
  hbn_vnode_handle_t sub_stream_handle;
	camera_handle_t cam_fd;
	vp_sensor_config_t sensor_config;
  vp_csi_config_t csi_config;
  std::shared_ptr<GdcBinBuf_ST> gdc_bin;
  std::shared_ptr<GdcBinBuf_ST> gdc_bin_r;
  int gdc_bin_buf_is_valid;
  int gdc_init_valid;
  int gdc_init_valid_r;
  int sub_stream_channel;
  bool gdc_resize_enable;
  MIPI_CAP_INFO_ST *cap_info_;
  std::shared_ptr<sensor_otp_t_> awb_otp_data;
}pipe_contex_t;

class HobotMipiCapIml : public HobotMipiCap {
 public:
  HobotMipiCapIml() {}
  ~HobotMipiCapIml() {}

  // 初始化设备环境，如X3的sensor GPIO配置和时钟配置
  // 返回值：0，成功；-1，配置失败
  virtual int initEnv();

  // 初始化相关sensor的VIO pipeline；
  // 输入参数：info--sensor的配置参数。
  // 返回值：0，初始化成功；-1，初始化失败。
  int init(MIPI_CAP_INFO_ST &info);

  // 反初始化相关sensor的VIO pipeline ；
  // 返回值：0，反初始化成功；-1，反初始化失败。
  int deInit();

  // 启动相关sensor的VIO pipeline的码流；
  // 返回值：0，启动成功；-1，启动失败。
  int start();

  // 停止相关sensor的VIO pipeline的码流；
  // 返回值：0，停止成功；-1，停止失败。
  int stop();

  // 如果有 vps ，就 输出vps 的分层数据 channel--"single":单sensor，"left": 双目的左sensor，"right":双目的右sensor，"combine"：左右sensor拼合的图像。
  std::shared_ptr<VideoBuffer> getFrame(std::string channel);

  // 获取cap的info信息；
  // 输入输出参数：MIPI_CAP_INFO_ST的结构信息。
  // 返回值：0，初始化成功；-1，初始化失败。
  int getCapInfo(MIPI_CAP_INFO_ST &info);

  std::vector<sensor_msgs::msg::CameraInfo>* getCalCamInfo() {
    return &cal_cam_info_;
  }

  int setCamInfo(std::vector<sensor_msgs::msg::CameraInfo> info) {
    cam_info_ = info;
    return 0;
  }

 protected:
  //遍历初始话的mipi host.
  void listMipiHost(std::vector<int> &mipi_hosts, std::vector<int> &started,
                    std::vector<int> &stoped);
  bool analysis_board_config();

  // 探测已经连接的sensor
  bool detectSensor(SENSOR_ID_T &sensor_info, int i2c_bus);

  int selectSensor(std::string &sensor, int &host, int &i2c_bus);

  void multiFrameTask();
  void subMultiFrameTask();
  void singleFrameTask();
  void sync_task();
  void sub_sync_task();
  bool isSynced(const std::vector<std::shared_ptr<VideoBuffer>> &frames, long long tolerance);
  int getVnodeFrame(hbn_vnode_handle_t handle, int channel, std::shared_ptr<VideoBuffer> buff_ptr);

  int create_and_run_vflow(pipe_contex_t *pipe_contex);
  int creat_vse_node(pipe_contex_t *pipe_contex);
  int creat_isp_node(pipe_contex_t *pipe_contex);
  int creat_vin_node(pipe_contex_t *pipe_contex);
  int creat_gdc_node(pipe_contex_t *pipe_contex);
  int creat_gdc_node_r(pipe_contex_t *pipe_contex);
  int creat_camera_node(camera_config_t* camera_config,int64_t* cam_fd);
  std::shared_ptr<GdcBinBuf_ST> get_gdc_bin(std::string gdc_bin_file);
  std::vector<std::shared_ptr<GdcBinBuf_ST>> gen_gdc_bin_stereo(int gdc_width, int gdc_height,int out_width, int out_height,
		std::vector<sensor_msgs::msg::CameraInfo> &cam_info, std::vector<sensor_msgs::msg::CameraInfo> &cal_cam_info,
    double rotation = 0.0, double cal_rotate = 0.0, double cla_alpha = 0.0, bool pre_rotation = false);
  std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin(int gdc_width, int gdc_height,int out_width, int out_height,
       sensor_msgs::msg::CameraInfo *cam_info, sensor_msgs::msg::CameraInfo *cal_cam_info,
       double rotation = 0.0, double cal_rotate = 0.0, double cla_alpha = 0.0, bool pre_rotation = false);
  std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin_rotation(int gdc_width, int gdc_height,int out_width, int out_height, double rotation);
  std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin_json(std::string file);

  //计算针孔模型下校正后内参对应的FOV（角度） K_rect 校正后的相机内参矩阵，校正后的图像高度和宽度
  std::pair<double, double> calculatePinholeFOV(const cv::Mat& K_rect, int width, int height);
  std::pair<double, double> calculateFisheyeFOV(const cv::Mat& K_rect, int width, int height);
  //基于目标FOV和有效FOV范围，计算初始alpha，目标水平FOV，双目交集的最小FOV和最大FOV，返回初始alpha
  double computeInitAlpha(double target_hfov, double fov_min, double fov_max);

  //根据目标水平FOV，计算双目立体校正的alpha参数（保证双目FOV一致性）
  double computeStereoAlphaFromFOV(double target_hfov, const cv::Mat& kl, const cv::Mat& Dl,  
                                                       const cv::Mat& Kr, const cv::Mat& Dr,
                                                       const cv::Mat& R_rl, const cv::Mat& t_rl,
                                                       int in_gdc_width, int in_gdc_height,
                                                       int out_gdc_width, int out_gdc_height,
                                                       const std::string& distortion_model,
                                                       double& actual_hfov_l, double& actual_hfov_r);
  double find_best_fov_scale(const cv::Mat& Kl, const cv::Mat& Dl,
                             const cv::Mat& Kr, const cv::Mat& Dr,
                             const cv::Mat& R_rl, const cv::Mat& t_rl,
                             cv::Size in_size,
                             cv::Size out_size);
  // -----------------------------------------------------------------------------------------------------
  
  int config_awb_otp(pipe_contex_t *pipe_contex); 

  bool m_inited_ = false;
  bool started_ = false;
  bool combine_flag_ = false;
  int vin_enable_ = true;
  int vps_enable_ = true;
  MIPI_CAP_INFO_ST cap_info_;
  int entry_index_ = 0;
  int sensor_bus_ = 2;
  int pipeline_id_ = 0;
  char cal_tpye_ = 0; //0x00:针孔标定；0x01：鱼眼标定。
  std::string eeprom_name_ = "";
  std::vector<int> mipi_started_;
  std::vector<int> mipi_stoped_;
  std::map<int, BOARD_CONFIG_ST> board_config_m_;
  std::map<int, std::vector<std::string>> host_sensor_m_;
  std::shared_ptr<std::thread> multi_frame_task_ = nullptr;
  std::shared_ptr<std::thread> sync_task_ = nullptr;
  std::shared_ptr<std::thread> sub_multi_frame_task_ = nullptr;
  std::shared_ptr<std::thread> sub_sync_task_ = nullptr;

  std::vector<sensor_msgs::msg::CameraInfo> cam_info_;
  std::vector<sensor_msgs::msg::CameraInfo> cal_cam_info_;
  std::vector<std::shared_ptr<GdcBinBuf_ST>> gdc_bin_buf_;
  std::vector<Imu_params> imu_info_;
  std::vector<std::shared_ptr<sensor_otp_t_>> awb_otp_data_;

  std::mutex queue_mtx_;

  typedef struct hbn_cfg {
    vin_attr_t vin_attr; 
    isp_cfg_t isp_attr;
    vse_cfg_t vse_attr;
    codec_cfg_t codec_attr;
  } hbn_cfg_t;

  typedef struct gdc_cfg {
    param_t gdc_param; 
    window_t  wnds;
    uint32_t wnd_num;
  } gdc_cfg_t;

  std::vector<std::shared_ptr<gdc_cfg_t>> v_gdc_cfg_;

  camera_config_t g_camera_config[PIPES_TOTAL];
  deserial_config_t g_deserial_config[PIPES_TOTAL];
  mipi_config_t g_mipi_config[PIPES_TOTAL];
  hbn_cfg_t g_hbn_cfg[PIPES_TOTAL];
  mipi_host_cfg_t g_mipi_host_cfg[PIPES_TOTAL];
  hbn_vflow_handle_t g_vflow_fd[PIPES_TOTAL] = {0};
  int64_t g_cam_fd[PIPES_TOTAL] = {-1};
  hbn_vnode_handle_t vse_node_handle[PIPES_TOTAL] = {0};

  std::vector<pipe_contex_t> pipe_contex;
  std::vector<std::shared_ptr<BuffQueueManage>> v_buff_que_manger_;
  std::shared_ptr<BuffQueueManage> combine_buff_que_manger_;
  std::vector<std::shared_ptr<FrameQueue>> v_frame_que_;
  std::vector<std::shared_ptr<BuffQueueManage>> v_sub_buff_que_manger_;
  std::shared_ptr<BuffQueueManage> sub_combine_buff_que_manger_;
  std::vector<std::shared_ptr<FrameQueue>> v_sub_frame_que_;
};

}  // namespace mipi_cam


#endif  // HOBOT_MIPI_CAP_IML_HPP_
