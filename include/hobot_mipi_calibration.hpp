#ifndef HOBOT_MIPI_CALIBRATION_HPP_
#define HOBOT_MIPI_CALIBRATION_HPP_


#include <memory>
#include <vector>
#include <cstdint>
#include <string>
#include <mutex>

#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cap.hpp"
#include "hobot_mipi_cam.hpp"


namespace mipi_cam {

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

#pragma pack(1)
typedef struct cal_dualcam_info_baolong_st
{
  unsigned char distortion_type;

  float k1l, k2l, k3l, k4l, k5l, k6l; 
  float p1l, p2l;                    

  float cxl, cyl;           
  float mean_errl, max_errl;
  float fxl, fyl;          

  float k1r, k2r, k3r, k4r, k5r, k6r; 
  float p1r, p2r;                  

  float cxr, cyr;        
  float mean_errr, max_errr; 
  float fxr, fyr;        

  float rot_x, rot_y, rot_z;
  float tx, ty, tz;      

  float measur_error;
} CalDualCamInfo_BaoLong_ST;
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

struct Imu_params
{
  ImuMislign_ST acc_mislign_;
  ImuMislign_ST gyro_mislign_;
  ImuScale_ST acc_scale_;
  ImuScale_ST gyro_scale_;
  ImuBias_ST acc_bias_;
  ImuBias_ST gyro_bias_;
  ImuNW_ST acc_n_w_;
  ImuNW_ST gyro_n_w_;
  ImuRTTimeInfo_d_ST r_t_info_;
};

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
  uint8_t awb_ratio;
} CONFIG_AWB_ST;
#pragma pack()

#pragma pack(4)
typedef struct dual_cam_awb_st_l_r {
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
} DualCamAwbCalib_ST_L_R;
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

#pragma pack(4)
typedef struct opt_awb_config
{
  CONFIG_AWB_ST awb_config_;
  DualCamAwbCalib_ST_L_R awb_info_l_r_;
  DualCamAwbCalib_ST_Golden awb_info_golden_;
} Opt_Awb_Config;
#pragma pack()

//单例类
class mipi_calibration {

  public: 

  static mipi_calibration & GetInstance() {
      static mipi_calibration instance;
      return instance;
  }

  struct CalibrationParams
  {
    std::string eeprom_name_;
    int i2c_bus;
    double cal_rotation_;
    std::vector<sensor_msgs::msg::CameraInfo> cam_info_;    
    std::vector<Imu_params> imu_info_;
    std::vector<std::shared_ptr<Opt_Awb_Config>> awb_otp_data_;
  };

  //返回结构体的const引用(零拷贝)
  std::vector<struct CalibrationParams>& getCalibrationParams() {
    //静态结构体，初始化一次
    return v_cal_params_;
  }

  bool getDualCamCalibrationFromEeprom_230ai(std::vector<sensor_msgs::msg::CameraInfo> &cam_info);  
  bool getDualCamCalibrationFromEeprom_baolong(std::vector<sensor_msgs::msg::CameraInfo> &cam_info);

  bool getCamCalibrationIml_single(sensor_msgs::msg::CameraInfo& cam_info, const std::string &file_path);
  bool getDualCamCalibrationIml(sensor_msgs::msg::CameraInfo &cam_info_l, sensor_msgs::msg::CameraInfo &cam_info_r, const std::string &file_path);
  
  // ===== 禁止拷贝/赋值（单例核心，单例唯一性）=====
  mipi_calibration(const mipi_calibration&) = delete;
  mipi_calibration & operator = (const mipi_calibration&) = delete;

  private:

  //私有构造函数——仅在首次获取实例时执行，完成Eeprom读取
  mipi_calibration()  : is_inited_(false) 
  {
      InitMipiCalibration();
  }

  ~mipi_calibration() = default;

  bool getCamCalibrationFromEeprom();
  bool getCamCalibration_yugang(int i2c_bus, uint16_t i2c_addr);
  bool getCamCalibration_union(int i2c_bus, uint16_t i2c_addr);
  bool getCamCalibration_abham(int i2c_bus, uint16_t i2c_addr);

  bool readEeprom16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, char* buf, int bufsize);
  int detectEeprom_drobot(int i2c_bus, std::string &device, uint16_t &i2c_addr);
  int detectEeprom_lianhe(std::string &device, int &i2c_bus, uint16_t &i2c_addr);
  std::vector<int> i2c_bus_detect();
  int detectEeprom_baolong(std::string &device, int &i2c_bus, uint16_t &i2c_addr);

  void InitMipiCalibration() {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_inited_) {
          return;
      }
      getCamCalibrationFromEeprom();
      is_inited_ = true;
  }
  std::vector<struct CalibrationParams> v_cal_params_;
  std::mutex mutex_;
  bool is_inited_;
  //---------------------------------------------------------------------------------
};
}
#endif