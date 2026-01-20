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
#include "hb_camera_data_info.h"

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
	camera_handle_t cam_fd;
	vp_sensor_config_t sensor_config;
  vp_csi_config_t csi_config;
  std::shared_ptr<GdcBinBuf_ST> gdc_bin;
  std::shared_ptr<GdcBinBuf_ST> gdc_bin_r;
  int gdc_bin_buf_is_valid;
  int gdc_init_valid;
  int gdc_init_valid_r;
  MIPI_CAP_INFO_ST *cap_info_;
  //-----------awb 扩展 ------------------
  sensor_otp_t awb_otp_data_;
  // ------------------------------------------
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
  int width;
  int height;
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



// ----------------------------------    awb -------------------------------------------------

// typedef struct {
//     uint8_t  rg_ratio;       // 红通道 / 绿色通道 增益比例（对应rg_ratio_x）
//     uint8_t  bg_ratio;       // 蓝通道 / 绿色通道 增益比例（对应bg_ratio_x）
//     uint8_t  ex_ratio;       // 扩展通道 / 绿色通道 增益比例（对应ex_ratio_x，备用/特殊通道）
//     //uint8_t  zone_en;        // 该区域增益使能位（0：禁用该区域配置，1：启用）
// } AWB_Zone_Gain_Config;

//AWB配置参数
#pragma pack(4)
typedef struct cal_awb_st {
  int width;
  int height;
  uint8_t pattern;          // 0x0138: 拜耳阵列格式（0x00=RGGB, 0x01=GRBG等）
  uint16_t bls_r;
  uint16_t bls_gr;
  uint16_t bls_gb;
  uint16_t bls_b;
  uint8_t awb_ratio;        // 0x013D: AWB计算区域配置（默认使用中央区域）
  //AWB_Zone_Gain_Config   zone_gain[5];    // 5个区域的增益配置（对应ratio_1~5）
} CONFIG_AWB_ST;
#pragma pack()


#pragma pack(4)
typedef struct dual_cam_awb_st_l {
    unsigned short r_3100K;
    unsigned short gr_3100K;
    unsigned short gb_3100K;
    unsigned short b_3100K;
    unsigned short rg_ratio_3100K;
    unsigned short bg_ratio_3100K;
    unsigned short r_4000K;
    unsigned short gr_4000K;
    unsigned short gb_4000K;
    unsigned short b_4000K;
    unsigned short rg_ratio_4000K;
    unsigned short bg_ratio_4000K;
    unsigned short r_5800K;
    unsigned short gr_5800K;
    unsigned short gb_5800K;
    unsigned short b_5800K;
    unsigned short rg_ratio_5800K;
    unsigned short bg_ratio_5800K;
} DualCamAwbCalib_ST_L;
#pragma pack()

#pragma pack(4)
typedef struct dual_cam_awb_st_r {
    unsigned short r_3100K;
    unsigned short gr_3100K;
    unsigned short gb_3100K;
    unsigned short b_3100K;
    unsigned short rg_ratio_3100K;
    unsigned short bg_ratio_3100K;
    unsigned short r_4000K;
    unsigned short gr_4000K;
    unsigned short gb_4000K;
    unsigned short b_4000K;
    unsigned short rg_ratio_4000K;
    unsigned short bg_ratio_4000K;
    unsigned short r_5800K;
    unsigned short gr_5800K;
    unsigned short gb_5800K;
    unsigned short b_5800K;
    unsigned short rg_ratio_5800K;
    unsigned short bg_ratio_5800K;
} DualCamAwbCalib_ST_R;
#pragma pack()


#pragma pack(4)
typedef struct dual_cam_awb_st_golden {
    unsigned short r_3100K;
    unsigned short gr_3100K;
    unsigned short gb_3100K;
    unsigned short b_3100K;
    unsigned short rg_ratio_3100K;
    unsigned short bg_ratio_3100K;
    unsigned short r_4000K;
    unsigned short gr_4000K;
    unsigned short gb_4000K;
    unsigned short b_4000K;
    unsigned short rg_ratio_4000K;
    unsigned short bg_ratio_4000K;
    unsigned short r_5800K;
    unsigned short gr_5800K;
    unsigned short gb_5800K;
    unsigned short b_5800K;
    unsigned short rg_ratio_5800K;
    unsigned short bg_ratio_5800K;
} DualCamAwbCalib_ST_Golden;
#pragma pack()


// ---------------------------------------------  awb ------------------------------------------------------------------------

#pragma pack(4)
typedef struct cal_dual_w_h_st {
  int width;
  int height;
} CalDualWHInfo_d_ST;
#pragma pack()

#pragma pack(4)
typedef struct cal_dual_M_D_d_st {
  double fx;
  double fy;
  double cx;
  double cy;
  double d[8];//鱼眼:k1,k2,k3,k4;针孔:k1,k2,p1,p2,k3,k4,k5,k6
} CalDualMDInfo_d_ST;
#pragma pack()

#pragma pack(4)
typedef struct cal_dual_R_T_info_d_st {
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
} CalDualRTInfo_d_ST;
#pragma pack()

#pragma pack(4)
typedef struct imu_R_T_time_info_d_st {
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
  float timeshift;
  float reporject;
} ImuRTTimeInfo_d_ST;
#pragma pack()

#pragma pack(4)
typedef struct imu_mislign_st {
  float m00;
  float m01;
  float m02;
  float m10;
  float m11;
  float m12;
  float m20;
  float m21;
  float m22;
} ImuMislign_ST;
#pragma pack()

#pragma pack(4)
typedef struct imu_scale_st {
  float s0;
  float s1;
  float s2;
} ImuScale_ST;
#pragma pack()

#pragma pack(4)
typedef struct imu_bias_st {
  float b0;
  float b1;
  float b2;
} ImuBias_ST;
#pragma pack()

#pragma pack(4)
typedef struct imu_n_w_st {
  float n;
  float w;
} ImuNW_ST;
#pragma pack()


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
  bool getDualCamCalibration_union(int i2c_bus, uint16_t i2c_addr);
  bool getDualCamCalibration_abham(int i2c_bus, uint16_t i2c_addr);
  bool getDualCamCalibrationFromEeprom_230ai();

  void dualFrameTask();

  int getVnodeFrame(hbn_vnode_handle_t handle, int channel, int* width,
		int* height, int* stride, void* frame_buf, unsigned int bufsize, unsigned int* len,
        uint64_t *timestamp, uint32_t* frame_id, bool gray = false);

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
    double rotation = 0.0, double cal_rotate = 0.0);
  std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin(int gdc_width, int gdc_height,int out_width, int out_height,
       sensor_msgs::msg::CameraInfo *cam_info, sensor_msgs::msg::CameraInfo *cal_cam_info,
       double rotation = 0.0, double cal_rotate = 0.0);
  std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin_rotation(int gdc_width, int gdc_height,int out_width, int out_height, double rotation);
  std::shared_ptr<GdcBinBuf_ST> gen_gdc_bin_json(std::string file);

  //int config_awb_otp(int64_t cam_fd);       // 新增AWB配置函数声明
  int config_awb_otp(pipe_contex_t *pipe_contex); 

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
  char cal_tpye_ = 0; //0x00:针孔标定；0x01：鱼眼标定。
  std::string eeprom_name_ = "";
  std::vector<int> mipi_started_;
  std::vector<int> mipi_stoped_;
  std::map<int, BOARD_CONFIG_ST> board_config_m_;
  std::map<int, std::vector<std::string>> host_sensor_m_;
  //std::vector<hbn_cfg_t> hbn_cfg_;
  std::shared_ptr<std::thread> dual_frame_task_ = nullptr;

  std::vector<sensor_msgs::msg::CameraInfo> cam_info_;
  std::vector<sensor_msgs::msg::CameraInfo> cal_cam_info_;
  std::vector<std::shared_ptr<GdcBinBuf_ST>> gdc_bin_buf_;

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

  // ---------------------------------------- AWB参数存储变量

  sensor_otp_t left_awb_otp_data_;     // 左摄sensor_otp_t格式参数（用于后续AWB配置）
  sensor_otp_t right_awb_otp_data_;    // 右摄sensor_otp_t格式参数（用于后续AWB配置）
//------------------------------------------------------------------------------------------------------
  std::vector<std::shared_ptr<gdc_cfg_t>> v_gdc_cfg_;

  camera_config_t g_camera_config[PIPES_TOTAL];
  deserial_config_t g_deserial_config[PIPES_TOTAL];
  mipi_config_t g_mipi_config[PIPES_TOTAL];
  hbn_cfg_t g_hbn_cfg[PIPES_TOTAL];
  mipi_host_cfg_t g_mipi_host_cfg[PIPES_TOTAL];
  hbn_vflow_handle_t g_vflow_fd[PIPES_TOTAL] = {0};
  int64_t g_cam_fd[PIPES_TOTAL] = {-1};
  hbn_vnode_handle_t vse_node_handle[PIPES_TOTAL] = {0};

  std::vector<pipe_contex_t> pipe_contex;      //pipe_contex：管道上下文结构体，存储每个摄像头的传感器配置、CSI 配置、流水线句柄等核心信息

  std::queue<std::shared_ptr<VideoBuffer_ST>> q_buff_empty_;
  std::queue<std::shared_ptr<VideoBuffer_ST>> q_left_buff_;
  std::queue<std::shared_ptr<VideoBuffer_ST>> q_right_buff_;
  std::vector<std::queue<std::shared_ptr<VideoBuffer_ST>>> q_v_buff_;
  std::queue<std::shared_ptr<VideoBuffer_ST>> q_combine_buff_;
  std::queue<std::shared_ptr<VideoBuffer_ST>> q_combine_buff_empty_;
};

}  // namespace mipi_cam


#endif  // HOBOT_MIPI_CAP_IML_HPP_
