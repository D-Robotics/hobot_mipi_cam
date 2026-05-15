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
#include "opencv2/opencv.hpp"
#include <stdint.h>
#include "hobot_mipi_cap.hpp"
#include "hobot_mipi_comm.hpp"
#include "hb_camera_interface.h"
#include "hbn_vpf_interface.h"
#include "vp_sensors.h"
#include "hb_gdc_cfg.h"
#include "gdc_cfg.h"

#include "hobot_mipi_calibration.hpp"

namespace mipi_cam {

#define PIPES_TOTAL 1


typedef enum {
	PIPELINE_SCENE_ISP_BYPASS    = 0,    /** 情景0(不需要ISP): Sensor 输出 YUV数据*/
	PIPELINE_SCENE_ISP_ONLY      = 1,    /** 情景1(ISP): 不需要运行YNR的Sensor, 或者宽和高 > 2048的sensor */
	PIPELINE_SCENE_ISP_YNR       = 2,    /** 情景2(ISP + YNR): 需要运行YNR的Sensor, 并且宽和高 <= 2048的sensor */
	PIPELINE_SCENE_MAX                   /**< 枚举边界，不可用作实际参数 */
} pipeline_usage_scene_type_t;

typedef struct {
	int isp_mode;
	int isp_hw_id;
	int isp_slot_id;

	int ynr_mode;
	int ynr_slot_id;

	int pym_slot_id;
	int pym_hw_id;
	int pym_mode;

	//PIPELINE_SCENE_ISP_BYPASS
	int is_online_vin_pym;

	//PIPELINE_SCENE_ISP_ONLY
	int is_online_vin_isp;
	int is_online_isp_pym;

	//PIPELINE_SCENE_ISP_YNR
	int is_online_isp_ynr;
	int is_online_ynr_pym;
}pipeline_channel_info_t;

typedef struct pipe_contex_s {
	hbn_vflow_handle_t vflow_fd;
	hbn_vnode_handle_t vin_node_handle;
	hbn_vnode_handle_t isp_node_handle;
  hbn_vnode_handle_t ynr_node_handle;
	hbn_vnode_handle_t pym_node_handle;
	hbn_vnode_handle_t gdc_node_handle;
  hbn_vnode_handle_t gdc_node_handle_r;
	hbn_vnode_handle_t vpu_node_handle;
  hbn_vnode_handle_t stream_handle;
	camera_handle_t cam_fd;
  deserial_handle_t des_fd;
	vp_sensor_config_t sensor_config;
  vp_csi_config_t csi_config;
  std::shared_ptr<GdcBinBuf_ST> gdc_bin;
  std::shared_ptr<GdcBinBuf_ST> gdc_bin_r;
  int gdc_init_valid;
  int gdc_init_valid_r;
  int stream_group;
  MIPI_CAP_INFO_ST *cap_info_;
  pipeline_channel_info_t pipe_info_;
  pipeline_usage_scene_type_t sensor_type_;
  int gsml_link_port_; // 0~3
  bool camera_bind_ = true;
}pipe_contex_t;

typedef struct {
  std::string sensor_type; //must
  std::string camera_mode; //must,"single"： 每个连接输出一个sensor的数据； "dual":每个连接输出两个sensor的数据。
  int dual_mode; //must
  int link_port; //must
  int mipi_rx;//must
  int dual_seq; //option
  bool valid_phy; //option
  int phy; //option
  bool valid_port2; //option
  int link_port2; //when dual mode is must
  int mipi_rx2; //when dual mode is must
  bool valid_phy2; //option
  int phy2; //option
} LINK_CONFIG_ST;

typedef struct {
  std::string deserial_name;
  std::vector<LINK_CONFIG_ST> link;
} GSML_CONFIG_ST;

typedef struct {
  deserial_config_t deserial_attr;
  deserial_handle_t des_fd;
  std::vector<std::shared_ptr<pipe_contex_t>> pipe;
} DESERIAL_CONTEX_ST;

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
  int mipi_init(MIPI_CAP_INFO_ST &info);
  int gsml_init(MIPI_CAP_INFO_ST &info);

  // 反初始化相关sensor的VIO pipeline ；
  // 返回值：0，反初始化成功；-1，反初始化失败。
  int deInit();

  // 启动相关sensor的VIO pipeline的码流；
  // 返回值：0，启动成功；-1，启动失败。
  int start();

  // 停止相关sensor的VIO pipeline的码流；
  // 返回值：0，停止成功；-1，停止失败。
  int stop();

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
  void sync_task();
  void sub_sync_task();
  bool isSynced(const std::vector<std::shared_ptr<VideoBuffer>> &frames, long long tolerance);
  int getVnodeFrame(hbn_vnode_handle_t handle, int channel, std::shared_ptr<VideoBuffer> buff_ptr);
  int getVnodeFrameGroup(hbn_vnode_handle_t handle, int channel, std::shared_ptr<VideoBuffer> buff_ptr);
  int create_and_run_vflow(std::shared_ptr<pipe_contex_t> pipe_contex);
  int create_and_run_vflow_step1(std::shared_ptr<pipe_contex_t> pipe_contex);
  int create_and_run_vflow_step2(std::shared_ptr<pipe_contex_t> pipe_contex);
  int create_pym_node(std::shared_ptr<pipe_contex_t> pipe_contex, int hw_id, int slot_id, int pym_mode);
  int create_isp_node(std::shared_ptr<pipe_contex_t> pipe_contex, int hw_id, int slot_id, int mode, int is_online);
  int create_ynr_node(std::shared_ptr<pipe_contex_t> pipe_contex, int slot_id, int work_mode);
  int create_vin_node(std::shared_ptr<pipe_contex_t> pipe_contex, int is_online, int link_port);
  int create_gdc_node(std::shared_ptr<pipe_contex_t> pipe_contex);
  int create_gdc_node_r(std::shared_ptr<pipe_contex_t> pipe_contex);
  int create_deserial_node(std::shared_ptr<pipe_contex_t> pipe_contex);
  int create_deserial_node(deserial_config_t *deserial_attr, deserial_handle_t &des_fd);
  int create_camera_node(std::shared_ptr<pipe_contex_t> pipe_contex, int link_port);


  int create_gsml_gdc_bin(std::shared_ptr<pipe_contex_t> pipe_contex);
  bool read_gsml_config(std::string gsml_cfg_file);

  void pipeline_connect_param_init(std::shared_ptr<pipe_contex_t> pipe_contex);

  // -----------------------------------------------------------------------------------------------------

  bool m_inited_ = false;
  bool started_ = false;
  bool combine_flag_ = false;
  int vin_enable_ = true;
  int vps_enable_ = true;
  int entry_index_ = 0;
  int sensor_bus_ = 2;
  int pipeline_id_ = 0;
  char cal_tpye_ = 0; //0x00:针孔标定；0x01：鱼眼标定。
  std::vector<int> mipi_started_;
  std::vector<int> mipi_stoped_;
  int pym_channel_ = 0;
  int vin_online_isp = 0;
  int isp_online_ynr = 1;
  std::map<int, BOARD_CONFIG_ST> board_config_m_;
  std::map<int, std::vector<std::string>> host_sensor_m_;
  std::vector<std::shared_ptr<std::thread>> task_;
  std::vector<GSML_CONFIG_ST> gsml_config_;
  int isp0_next_slot_id = 4;
  std::vector<sensor_msgs::msg::CameraInfo> cam_info_;
  std::vector<sensor_msgs::msg::CameraInfo> cal_cam_info_;
  std::vector<std::shared_ptr<GdcBinBuf_ST>> gdc_bin_buf_;

  std::vector<std::shared_ptr<GdcBinBuf_ST>> gdc_bin_buf_r_;
  std::vector<std::shared_ptr<Opt_Awb_Config>> awb_otp_data_;

  std::mutex queue_mtx_;

  typedef struct gdc_cfg {
    param_t gdc_param; 
    window_t  wnds;
    uint32_t wnd_num;
  } gdc_cfg_t;

  std::vector<std::shared_ptr<gdc_cfg_t>> v_gdc_cfg_;

  camera_config_t g_camera_config[PIPES_TOTAL];
  deserial_config_t g_deserial_config[PIPES_TOTAL];
  mipi_config_t g_mipi_config[PIPES_TOTAL];
  mipi_host_cfg_t g_mipi_host_cfg[PIPES_TOTAL];
  hbn_vflow_handle_t g_vflow_fd[PIPES_TOTAL] = {0};
  int64_t g_cam_fd[PIPES_TOTAL] = {-1};
  hbn_vnode_handle_t pym_node_handle[PIPES_TOTAL] = {0};
  std::vector<std::shared_ptr<pipe_contex_t>> pipe_contex;
  std::vector<std::shared_ptr<DESERIAL_CONTEX_ST>> deserial_contex;

};

}  // namespace mipi_cam


#endif  // HOBOT_MIPI_CAP_IML_HPP_
