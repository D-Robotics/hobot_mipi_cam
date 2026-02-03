#ifndef HOBOT_MIPI_CALIBRATION_HPP_
#define HOBOT_MIPI_CALIBRATION_HPP_


#include <vector>
#include <cstdint>
#include <string>
#include <mutex>

#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cap.hpp"

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

//单例类
class mipi_calibration {

public: 

static mipi_calibration & GetInstance() {
    static mipi_calibration instance;
    return instance;
}

struct CalibrationParams
{
  const std::vector<sensor_msgs::msg::CameraInfo>& cam_info_;
  const MIPI_CAP_INFO_ST& cap_info_;
  const std::string& eeprom_name_;

  //禁止外部构造，仅类内初始化
  CalibrationParams() = delete;
  explicit CalibrationParams(mipi_calibration& parent) 
  :   cam_info_(parent.cam_info_),
      cap_info_(parent.cap_info_),
      eeprom_name_(parent.eeprom_name_) {}
};

//返回结构体的const引用(零拷贝)
const CalibrationParams& getCalibrationParams() const {
  //静态结构体，初始化一次
  static CalibrationParams params(*const_cast<mipi_calibration*>(this));
  return params;
}

bool getDualCamCalibrationFromEeprom_230ai(std::vector<sensor_msgs::msg::CameraInfo> &cam_info_);  

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

bool getDualCamCalibrationFromEeprom();
bool getDualCamCalibration_yugang(int i2c_bus, uint16_t i2c_addr);
bool getDualCamCalibration_union(int i2c_bus, uint16_t i2c_addr);
bool getDualCamCalibration_abham(int i2c_bus, uint16_t i2c_addr);

bool readEeprom16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, char* buf, int bufsize);
int detectEeprom_drobot(std::string &device, int &i2c_bus, uint16_t &i2c_addr);
int detectEeprom_lianhe(std::string &device, int &i2c_bus, uint16_t &i2c_addr);

void InitMipiCalibration() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_inited_) {
        return;
    }

    getDualCamCalibrationFromEeprom();

    is_inited_ = true;
}

std::string eeprom_name_ = "";
char cal_tpye_ = 0; //0x00:针孔标定；0x01：鱼眼标定。
MIPI_CAP_INFO_ST cap_info_;

std::vector<sensor_msgs::msg::CameraInfo> cam_info_;
std::mutex mutex_;
bool is_inited_;
};
}
#endif