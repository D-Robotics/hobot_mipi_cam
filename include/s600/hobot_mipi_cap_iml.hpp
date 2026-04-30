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
#include <stdint.h>
#include "hobot_mipi_cap.hpp"
#include "hobot_mipi_comm.hpp"
#include "hb_camera_interface.h"
#include "hbn_vpf_interface.h"
#include "vp_sensors.h"
#include "hb_gdc_cfg.h"
#include "gdc_cfg.h"
//#include "codec_cfg.h"

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

typedef struct eeprom_id {
  int i2c_bus;           // sensor挂在哪条总线上
  int i2c_dev_addr;      // sensor i2c设备地址
  int i2c_addr_width;    // 总线地址宽（1/2字节）
  int det_reg;           // 读取的寄存器地址
  int check_value;           // 读取的寄存器地址
  char device_name[25];  // sensor名字
} EEPROM_ID_T;


typedef struct eeprom_detect {
  int i2c_bus;           // sensor挂在哪条总线上
  int i2c_dev_addr;      // sensor i2c设备地址
  int i2c_addr_width;    // 总线地址宽（1/2字节）
  int det_reg;           // 读取的寄存器地址
  char check_str[10];           // 读取的寄存器地址
  char device_name[25];  // sensor名字
} EEPROM_DETECT_T;

#pragma pack(4)
typedef struct eeprom_drobot_head_st {
  char flag[8];
  char camType; //0x00:单目；0x01:双目
  char cal_tpye; //0x00:针孔标定；0x01:鱼眼标定
  char ver_main;
  char ver_min;
  char angle; //描述标定时是否旋转后再标定，旋转角度。0x00:表示不旋转，0x01:表示顺时针旋转90度,0x02:表示顺时针旋转180度,0x03:表示顺时针旋转270度,其他的数值无效，表示不旋转。
  char d_num; //D畸变参数的个数，鱼眼标定：k1,k2,k3,k4;针孔标定:k1,k2,p1,p2,k3,k4,k5,k6
  char res2;
  char check;
} EepromDrobotHead_ST;
#pragma pack()

#pragma pack(4)
typedef struct cal_dualcam_info_st {
  double fxl;        
  double fyl;    
  double cxl;    
  double cyl;        
  double k1l;        
  double k2l;
  double k3l;
  double k4l;
  double k5l;
  double k6l;
  double p1l;
  double p2l;
  double rmsl;
  double fxr;
  double fyr;
  double cxr;
  double cyr;
  double k1r;
  double k2r;
  double k3r;
  double k4r;
  double k5r;
  double k6r;
  double p1r;
  double p2r;
  double rmsr;
  double r11;
  double r12;
  double r13;
  double r21;
  double r22;
  double r23;
  double r31;
  double r32;
  double r33;
  double tx;
  double ty;
  double tz;
  double epilines;
  char h_v[4];
} CalDualCamInfo_ST;
#pragma pack()


#if 0
#pragma pack(4)
typedef struct cal_dual_M_D_st {
  int height;
  int width;
  float fx;
  float fy;
  float cx;
  float cy;
  float k1;
  float k2;
  float p1;
  float p2;
  float k3;
  float k4;
  float k5;
  float k6;
} CalDualMDInfo_ST;
#pragma pack()
#endif

#pragma pack(4)
typedef struct cal_dual_M_D_st {
  int width;
  int height;
  float fx;
  float fy;
  float cx;
  float cy;
  float d[8];//鱼眼:k1,k2,k3,k4;针孔:k1,k2,p1,p2,k3,k4,k5,k6
} CalDualMDInfo_ST;
#pragma pack()

#pragma pack(4)
typedef struct cal_dual_R_T_info_st {
  float r11;
  float r12;
  float r13;
  float r21;
  float r22;
  float r23;
  float r31;
  float r32;
  float r33;
  float tx;
  float ty;
  float tz;
} CalDualRTInfo_ST;
#pragma pack()

typedef struct {
  int link_id;
  std::string sensor_type; 
  std::string camera_mode; //"single"： 每个连接输出一个sensor的数据； "dual":每个连接输出两个sensor的数据。
  int dual_mode;
  int mipi_rx;
  bool valid_phy;
  int phy;
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

  int detectEeprom_lianhe(std::string &device, int &i2c_bus, uint16_t &i2c_addr);
  int detectEeprom_drobot(std::string &device, int &i2c_bus, uint16_t &i2c_addr);
  bool readEeprom16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, char* buf, int bufsize);
  bool getDualCamCalibrationFromEeprom();
  bool getDualCamCalibration_yugang(int i2c_bus, uint16_t i2c_addr);
  bool getDualCamCalibrationFromEeprom_230ai();
  void multiFrameTask();
  void sync_task();
  void sub_sync_task();
  bool isSynced(const std::vector<std::shared_ptr<VideoBuffer>> &frames, long long tolerance);
  int getVnodeFrame(hbn_vnode_handle_t handle, int channel, std::shared_ptr<VideoBuffer> buff_ptr);
  int getVnodeFrameGroup(hbn_vnode_handle_t handle, int channel, std::shared_ptr<VideoBuffer> buff_ptr);
  int create_and_run_vflow(std::shared_ptr<pipe_contex_t> pipe_contex);
  int create_pym_node(std::shared_ptr<pipe_contex_t> pipe_contex, int hw_id, int slot_id, int pym_mode);
  int create_isp_node(std::shared_ptr<pipe_contex_t> pipe_contex, int hw_id, int slot_id, int mode, int is_online);
  int create_ynr_node(std::shared_ptr<pipe_contex_t> pipe_contex, int slot_id, int work_mode);
  int create_vin_node(std::shared_ptr<pipe_contex_t> pipe_contex, int is_online, int link_port);
  int create_gdc_node(std::shared_ptr<pipe_contex_t> pipe_contex);
  int create_gdc_node_r(std::shared_ptr<pipe_contex_t> pipe_contex);
  int create_deserial_node(std::shared_ptr<pipe_contex_t> pipe_contex);
  int create_deserial_node(deserial_config_t *deserial_attr, deserial_handle_t &des_fd);
  int create_camera_node(std::shared_ptr<pipe_contex_t> pipe_contex, int link_port);
  std::shared_ptr<GdcBinBuf_ST> get_gdc_bin(std::string gdc_bin_file);
  std::vector<std::shared_ptr<GdcBinBuf_ST>> gen_gdc_bin_stereo(int gdc_width, int gdc_height,int out_width, int out_height,
		std::vector<sensor_msgs::msg::CameraInfo> &cam_info, std::vector<sensor_msgs::msg::CameraInfo> &cal_cam_info,
    double rotation = 0.0, double cal_rotate = 0.0);
  std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin(int gdc_width, int gdc_height,int out_width, int out_height,
       sensor_msgs::msg::CameraInfo *cam_info, sensor_msgs::msg::CameraInfo *cal_cam_info,
       double rotation = 0.0, double cal_rotate = 0.0);
  std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin_rotation(int gdc_width, int gdc_height,int out_width, int out_height, double rotation);
  std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin_json(std::string file);
  bool read_gsml_config(std::string gsml_cfg_file);

  void pipeline_connect_param_init(std::shared_ptr<pipe_contex_t> pipe_contex);

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
  std::vector<std::shared_ptr<BuffQueueManage>> v_buff_que_manger_;
  std::shared_ptr<BuffQueueManage> combine_buff_que_manger_;
  std::vector<std::shared_ptr<FrameQueue>> v_frame_que_;
  std::vector<std::shared_ptr<BuffQueueManage>> v_sub_buff_que_manger_;
  std::shared_ptr<BuffQueueManage> sub_combine_buff_que_manger_;
  std::vector<std::shared_ptr<FrameQueue>> v_sub_frame_que_;

};

}  // namespace mipi_cam


#endif  // HOBOT_MIPI_CAP_IML_HPP_
