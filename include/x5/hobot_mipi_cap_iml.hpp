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

namespace mipi_cam {

#define PIPES_TOTAL 1


typedef struct pipe_contex_s {
	hbn_vflow_handle_t vflow_fd;
	hbn_vnode_handle_t vin_node_handle;
	hbn_vnode_handle_t isp_node_handle;
	hbn_vnode_handle_t vse_node_handle;
	hbn_vnode_handle_t gdc_node_handle;
	hbn_vnode_handle_t vpu_node_handle;
	camera_handle_t cam_fd;
	vp_sensor_config_t sensor_config;
  hb_mem_common_buf_t gdc_bin_buf;
  int gdc_bin_buf_is_valid;
  int gdc_init_valid;
  MIPI_CAP_INFO_ST *cap_info_;
}pipe_contex_t;

typedef struct video_buffer_s {
  uint64_t timestamp;
  uint32_t frame_id;
  int width;
  int height;
  int stride;
  uint32_t buff_size;
  uint32_t data_size;
  void* buff;
  ~video_buffer_s() {
    if (buff != NULL) {
      free(buff);
      buff = NULL;
    }
  }
} VideoBuffer_ST;


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
  int getFrame(std::string channel, int* nVOutW, int* nVOutH,
        void* buf, unsigned int bufsize, unsigned int*, uint64_t&, bool gray = false);


  // 获取cap的info信息；
  // 输入输出参数：MIPI_CAP_INFO_ST的结构信息。
  // 返回值：0，初始化成功；-1，初始化失败。
  int getCapInfo(MIPI_CAP_INFO_ST &info);

 protected:
  //遍历初始话的mipi host.
  void listMipiHost(std::vector<int> &mipi_hosts, std::vector<int> &started,
                    std::vector<int> &stoped);
  bool analysis_board_config();

  // 探测已经连接的sensor
  bool detectSensor(SENSOR_ID_T &sensor_info, int i2c_bus);

  int selectSensor(std::string &sensor, int &host, int &i2c_bus);

  void dualFrameTask();

  int getVnodeFrame(hbn_vnode_handle_t handle, int channel, int* width,
		int* height, int* stride, void* frame_buf, unsigned int bufsize, unsigned int* len,
        uint64_t *timestamp, uint32_t* frame_id, bool gray = false);

  int create_and_run_vflow(pipe_contex_t *pipe_contex);
  int creat_vse_node(pipe_contex_t *pipe_contex);
  int creat_isp_node(pipe_contex_t *pipe_contex);
  int creat_vin_node(pipe_contex_t *pipe_contex);
  int creat_gdc_node(pipe_contex_t *pipe_contex);
  int creat_camera_node(camera_config_t* camera_config,int64_t* cam_fd);
  int get_gdc_config(std::string gdc_bin_file, hb_mem_common_buf_t *bin_buf);

  bool m_inited_ = false;
  bool started_ = false;
  //x3_vin_info_t vin_info_;
  //x3_vps_infos_t vps_infos_;  // vps的配置，支持多个vps group
  int vin_enable_ = true;
  int vps_enable_ = true;
  MIPI_CAP_INFO_ST cap_info_;
  int entry_index_ = 0;
  int sensor_bus_ = 2;
  int pipeline_id_ = 0;
  std::vector<int> mipi_started_;
  std::vector<int> mipi_stoped_;
  std::map<int, BOARD_CONFIG_ST> board_config_m_;
  std::map<int, std::vector<std::string>> host_sensor_m_;
  //std::vector<hbn_cfg_t> hbn_cfg_;
  std::shared_ptr<std::thread> dual_frame_task_ = nullptr;

  std::mutex queue_mtx_;

  typedef struct hbn_cfg {
    vin_attr_t vin_attr; 
    isp_cfg_t isp_attr;
    vse_cfg_t vse_attr;
    codec_cfg_t codec_attr;
  } hbn_cfg_t;
  camera_config_t g_camera_config[PIPES_TOTAL];
  deserial_config_t g_deserial_config[PIPES_TOTAL];
  mipi_config_t g_mipi_config[PIPES_TOTAL];
  hbn_cfg_t g_hbn_cfg[PIPES_TOTAL];
  mipi_host_cfg_t g_mipi_host_cfg[PIPES_TOTAL];
  hbn_vflow_handle_t g_vflow_fd[PIPES_TOTAL] = {0};
  int64_t g_cam_fd[PIPES_TOTAL] = {-1};
  hbn_vnode_handle_t vse_node_handle[PIPES_TOTAL] = {0};

  std::vector<pipe_contex_t> pipe_contex;

  std::queue<std::shared_ptr<VideoBuffer_ST>> q_buff_empty_;
  std::queue<std::shared_ptr<VideoBuffer_ST>> q_left_buff_;
  std::queue<std::shared_ptr<VideoBuffer_ST>> q_right_buff_;
  std::vector<std::queue<std::shared_ptr<VideoBuffer_ST>>> q_v_buff_;
  std::queue<std::shared_ptr<VideoBuffer_ST>> q_combine_buff_;
  std::queue<std::shared_ptr<VideoBuffer_ST>> q_combine_buff_empty_;
};

}  // namespace mipi_cam


#endif  // HOBOT_MIPI_CAP_IML_HPP_
