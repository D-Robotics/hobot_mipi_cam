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

#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cap_iml.hpp"
#include "vio/hb_vio_interface.h"
#include "cam/hb_cam_interface.h"
#include "sensor_msgs/msg/camera_info.hpp"

#include <string>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <regex>

#include "x3_sdk_wrap.h"
#include "x3_vio_bind.h"
#include "x3_vio_vin.h"
#include "x3_vio_vp.h"
#include "x3_vio_vps.h"
#include "x3_preparam.h"

#include <rclcpp/rclcpp.hpp>
#include <json/json.h>

namespace mipi_cam {

int HobotMipiCapIml::initEnv() {
  std::vector<int> mipi_hosts;
  std::vector<int> mipi_bus;
  if (analysis_board_config ()) {
    if (board_config_m_.size() > 0) {
      for (auto board : board_config_m_) {
        mipi_hosts.push_back(board.first);
        mipi_bus.push_back(board.second.i2c_bus);
      }
    } else {
      mipi_hosts = {0,1,2,3};
    }
  } else {
    mipi_hosts = {0,1,2,3};
  }
  listMipiHost(mipi_hosts, mipi_started_, mipi_stoped_);
  if (mipi_started_.size() > 0) { //暂时不能同时启动多个mipi_cam进程
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "The device has already been started\n");
    return -1;
  }
  if (mipi_stoped_.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "There are no available host.\n");
    return -1;
  }
  if (board_config_m_.size() == 0) {
    if (mipi_started_.size() > 0) {
      return -1;
    }
  }

  if (mipi_started_.size() == 0) {
    // 系统镜像启动后进行初始化，暂时不做初始化，新硬件有差异在重载initEnv特殊初始化。
    std::vector<std::string> sys_cmds;
    std::string cmd_name;
    for (int num : mipi_stoped_) {
      cmd_name = "echo 1 > /sys/class/vps/mipi_host" + std::to_string(num) + "/param/snrclk_en > /dev/null";
      sys_cmds.push_back(cmd_name);
      cmd_name = "echo 24000000 > /sys/class/vps/mipi_host" + std::to_string(num) + "/param/snrclk_freq > /dev/null";
      sys_cmds.push_back(cmd_name);
      cmd_name = "echo 1 > /sys/class/vps/mipi_host" + std::to_string(num) + "/param/stop_check_instart";
      sys_cmds.push_back(cmd_name);
    }
    for (const auto& sys_cmd : sys_cmds) {
      RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "sys_cmd:%s\n",sys_cmd.c_str());
      system(sys_cmd.data());
    }
  }

  /* sdb 生态开发板  ，使能sensor       mclk, 否则i2c 通信不会成功的 */
  HB_MIPI_EnableSensorClock(0);
  HB_MIPI_EnableSensorClock(1);
  // HB_MIPI_EnableSensorClock(2); // 需要修改内核dts使能mipihost2的mclk

  return 0;
}

int HobotMipiCapIml::init(MIPI_CAP_INFO_ST &info) {
  int ret = 0;
  cap_info_ = info;

  entry_index_ = cap_info_.channel_;

  if (cap_info_.device_mode_.compare("dual") == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "X3 platform no suppot dual channle camera.\n");
    return -1;
  }

  if (selectSensor(cap_info_.sensor_type, entry_index_, sensor_bus_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "not select  sensor!\n");
    return -1;
  }
  int pipeline_id = 0;
  for (; pipeline_id < 8; pipeline_id++) {
    if (!checkPipelineOpened(pipeline_id)) {
      break;
    }
  }
  if (pipeline_id >= 8) {
    goto vp_err;
  }
  pipeline_id_ = pipeline_id;

  parseConfig(cap_info_.sensor_type, cap_info_.width, cap_info_.height, cap_info_.fps);

  ret = x3_vp_init();
  if (ret) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s]->hb_vp_init failed, ret: %d.\n", __func__, ret);
    goto vp_err;
  }
  // 初始化算法模块
  // 1. 初始化
  // vin，此处调用失败，大概率是因为sensor没有接，或者没有接好，或者sensor库用的版本不配套
  ret = x3_vin_init(&vin_info_);
  // {  // check分辨率
    // int sensor_w =vin_info.mipi_attr.mipi_host_cfg.width;
    // int sensor_h =vin_info.mipi_attr.mipi_host_cfg.height;
    // int input_w = m_oCamInfo.width;
    // int input_h = m_oCamInfo.height;
    // if (!checkParams(sensor_w, sensor_h, input_w, input_h)) {
    //  x3_vp_deinit();
    //  return -3;
    // }

  if (ret) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "x3_vin_init failed: %d!\n", ret);
    if (HB_ERR_VIN_SIF_INIT_FAIL == abs(ret)) {  //重复打开
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "Cannot open '%s'! You may have started mipi_cam repeatedly?",
        cap_info_.sensor_type);
    }
    x3_vp_deinit();
    return -2;
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_vin_init ok!\n");
  // 2. 初始化 vps，创建 group
  if (vps_enable_) {
    ret = x3_vps_init_wrap(&vps_infos_.m_vps_info[0]);
    if (ret) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "x3_vps_init failed = %d.\n", ret);
      goto vps_err;
    }
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_vps_init_wrap ok!\n");
  }
  // 4 vin bind vps
  if (vin_enable_ && vps_enable_) {
    ret = x3_vin_bind_vps(
        vin_info_.pipe_id,
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        vin_info_.vin_vps_mode);
    if (ret) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "[%s]x3_vin_bind_vps failed, %d.\n", __func__, ret);
      goto vps_bind_err;
    }
  }
  
  HB_ISP_GetSetInit();
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "x3 camera init success.\n");
  return ret;
vps_bind_err:
  if (vps_enable_) {
    x3_vps_uninit_wrap(&vps_infos_.m_vps_info[0]);
  }
vps_err:
  if (vin_enable_) {
    x3_vin_deinit(&vin_info_);
  }
vin_err:
  x3_vp_deinit();
vp_err:
  RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
    "[%s]->vp create err.\n", __func__);
  return -1;
}

int HobotMipiCapIml::deInit() {
  int i = 0;
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "x3_cam_deinit start.\n");
  if (vin_enable_ && vps_enable_) {
    x3_vin_unbind_vps(
        vin_info_.pipe_id,
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        vin_info_.vin_vps_mode);
  }
  if (vps_enable_) {
    for (i = 0; i < vps_infos_.m_group_num; i++)
      x3_vps_uninit_wrap(&vps_infos_.m_vps_info[i]);
  }
  if (vin_enable_) {
    x3_vin_deinit(&vin_info_);
  }
  x3_vp_deinit();
  HB_ISP_GetSetExit();
  return 0;
}

int HobotMipiCapIml::start() {
  int i = 0, ret = 0;
  // 使能 vps
  if (vps_enable_) {
    for (i = 0; i < vps_infos_.m_group_num; i++) {
      ret = x3_vps_start(
          vps_infos_.m_vps_info[i].m_vps_grp_id);
      if (ret) {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "x3_vps_start failed, %d", ret);
        return -3;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_vps_start ok!");
  }
  if (vin_enable_) {
    ret = x3_vin_start(&vin_info_);
    if (ret) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "x3_vin_start failed, %d", ret);
      return -3003;
    }
  }
  started_ = true;
  return 0;
}

int HobotMipiCapIml::stop() {
  int i = 0, ret = 0;
  if (!started_) {
     RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "x3 camera isn't started");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_mipi_cam_stop start.\n");
  if (vin_enable_) {
    x3_vin_stop(&vin_info_);
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_mipi_cam_stop groupNum=%d.\n",
             vps_infos_.m_group_num);
  // stop vps
  if (vps_enable_) {
    for (i = 0; i < vps_infos_.m_group_num; i++) {
      x3_vps_stop(vps_infos_.m_vps_info[i].m_vps_grp_id);
      if (ret) {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "x3_vps_stop failed, %d", ret);
        return -1;
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_mipi_cam_stop end.\n");
  return 0;
}

int HobotMipiCapIml::getFrame(std::string channel, int* nVOutW, int* nVOutH,
        void* frame_buf, unsigned int bufsize, unsigned int* len,
        uint64_t &timestamp, bool gray) {
  int size = -1, ret = 0;
  struct timeval select_timeout = {0};
  hb_vio_buffer_t vOut;
  if (!started_) {
     RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "x3 camera isn't started");
    return -1;
  }
  int i = 0;
  int stride = 0, width = 0, height = 0;
  int nTry = 0;
  do {
    ret = HB_VPS_GetChnFrame(
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        vps_channel_,
        &vOut,
        1000);
    if (ret < 0) {
      if (++nTry > 3) {
        if (ret == -268696577 || ret == -268696579 ||
            ret == -268696580) {  // group不存在或非法
           RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
            "Invalid groupID: %d! Please check if this group has been "
            "closed in unexpected places!",
            vps_infos_.m_vps_info[0].m_vps_grp_id);
          return -1;
        } else if (ret == -268696581 ||
                   -268696587 == ret) {  // 通道未使能或不存在
           RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
            "Invalid chn:%d in groupID: %d! Please check if this channel "
            "has been closed in unexpected places!",
            vps_channel_,
            vps_infos_.m_vps_info[0].m_vps_grp_id);
          return -1;
        }
        RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "[GetVpsFrame]->HB_VPS_GetChnFrame chn=%d,goupID=%d,error =%d "
          "!!!\n",
          vps_channel_,
          vps_infos_.m_vps_info[0].m_vps_grp_id,
          ret);
        return -1;
      }
      usleep(10 * 1000);
      continue;
    }
    timestamp = vOut.img_info.tv.tv_sec * 1e9 + vOut.img_info.tv.tv_usec * 1e3;
    size = vOut.img_addr.stride_size * vOut.img_addr.height;
    stride = vOut.img_addr.stride_size;
    width = vOut.img_addr.width;
    height = vOut.img_addr.height;
    *nVOutW = width;
    *nVOutH = height;
    if (gray == true) {
      *len = width * height;
    } else {
      *len = width * height * 3 / 2;
    }
    if (bufsize < *len) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "buf size(%d) < frame size(%d)", bufsize, *len);
      return -1;
    }
    ISP_AE_PARAM_S stAeParam;
    stAeParam.GainOpType = static_cast<ISP_OP_TYPE_E>(0);
    stAeParam.IntegrationOpType = static_cast<ISP_OP_TYPE_E>(0);
    ret = HB_ISP_GetAeParam(vin_info_.pipe_id, &stAeParam);
    if (ret == 0) {
      timestamp += stAeParam.u32IntegrationTime / 2 * 1e3;
    }
    if (stride == width) {
      memcpy(frame_buf, vOut.img_addr.addr[0], width * height);
      if (gray == false) {
        memcpy(frame_buf + width * height,
             vOut.img_addr.addr[1],
             width * height / 2);
      }
    } else {
      // jump over stride - width Y
      for (i = 0; i < height; i++) {
        memcpy(
            frame_buf + i * width, vOut.img_addr.addr[0] + i * stride, width);
      }
      // jump over stride - width UV
      if (gray == false) {
        for (i = 0; i < height / 2; i++) {
          memcpy(frame_buf + width * height + i * width,
                vOut.img_addr.addr[1] + i * stride,
                width);
        }
      }
    }
    HB_VPS_ReleaseChnFrame(
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        vps_channel_,
        &vOut);
    break;
  } while (1);
  return 0;
}

int HobotMipiCapIml::parseConfig(std::string sensor_name,
                                   int w, int h, int fps) {
  int ret = 0;
  if(strcasecmp(sensor_name.c_str(), "imx415") == 0) {
    imx415_linear_vin_param_init(&vin_info_);
  } else if(strcasecmp(sensor_name.c_str(), "f37") == 0) {
    f37_linear_vin_param_init(&vin_info_);
  } else if(strcasecmp(sensor_name.c_str(), "gc4663") == 0) {
    gc4663_linear_vin_param_init(&vin_info_);
  } else if(strcasecmp(sensor_name.c_str(), "imx586") == 0) {
    // enable, sdb3.0
    std::vector<std::string> sys_cmds{
        "echo 111 > /sys/class/gpio/export",
        "echo out > /sys/class/gpio/gpio111/direction",
        "echo 1 > /sys/class/gpio/gpio111/value",
    };
    for (const auto& sys_cmd : sys_cmds) {
      system(sys_cmd.data());
    }
    imx586_linear_vin_param_init(&vin_info_);
  } else if(strcasecmp(sensor_name.c_str(), "gc4c33") == 0) {
    gc4c33_linear_vin_param_init(&vin_info_);
  } else if(strcasecmp(sensor_name.c_str(), "imx219") == 0) {
    imx219_linear_vin_param_init(&vin_info_);
  } else if(strcasecmp(sensor_name.c_str(), "imx477") == 0) {
    imx477_linear_vin_param_init(&vin_info_);
  } else if(strcasecmp(sensor_name.c_str(), "ov5647") == 0) {
    ov5647_linear_vin_param_init(&vin_info_);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s]->sensor name not found(%s).\n", __func__, sensor_name.c_str());
    // m_oX3UsbCam.m_infos.m_vin_enable = 0;
    return -1;
  }

  vin_info_.pipe_id = pipeline_id_;
  vin_info_.dev_id = pipeline_id_;
  vin_info_.snsinfo.sensorInfo.bus_num = sensor_bus_;
  vin_info_.snsinfo.sensorInfo.entry_index = entry_index_;
  vin_info_.snsinfo.sensorInfo.dev_port = pipeline_id_;
    // 减少ddr带宽使用量
  vin_info_.vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;
  // 2. 根据vin中的分辨率配置vps
  int width = vin_info_.mipi_attr.mipi_host_cfg.width;
  int height = vin_info_.mipi_attr.mipi_host_cfg.height;
  int mipi_fps = vin_info_.mipi_attr.mipi_host_cfg.fps;
  vps_enable_ = 1;
  vps_infos_.m_group_num = 1;
  // 配置group的输入
  ret |= vps_grp_param_init(
      &vps_infos_.m_vps_info[0], width, height);
  // 以下是配置group的每一个通道的参数
  vps_infos_.m_vps_info[0].m_chn_num = 1;
  ret |= vps_chn_param_init(
      &vps_infos_.m_vps_info[0].m_vps_chn_attrs[0],
      vps_channel_, w, h, fps);
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
      "[%s]-> w:h=%d:%d ,fps=%d sucess.\n", __func__, w, h, fps);
  vps_infos_.m_vps_info[0].m_vps_grp_id = pipeline_id_;
  return 0;
}

bool HobotMipiCapIml::checkPipelineOpened(int pipeline_idx) {
  std::string cfg_info;
  // 若mipi camera已打开，sif_info返回“pipeid”
  std::ifstream sif_info("/sys/devices/platform/soc/a4001000.sif/cfg_info");
  if (sif_info.is_open()) {
    while (getline(sif_info, cfg_info)) {
      std::string pipe_line_info =
            "pipe " + std::to_string(pipeline_idx) + " not inited";
      if (!cfg_info.compare(pipe_line_info)) {
        RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
          "mipi camera the pipeline %d is idle.\n", pipeline_idx);
        return false;
      }
    }
  }
  return true;
}

int HobotMipiCapIml::getCapInfo(MIPI_CAP_INFO_ST &info) {
  info = cap_info_;
  return 0;
}

void HobotMipiCapIml::listMipiHost(std::vector<int> &mipi_hosts, 
    std::vector<int> &started, std::vector<int> &stoped) {
  std::vector<int> host;
  std::string board_type_str = "";
  for (int num : mipi_hosts) {
    std::string mipi_host = "/sys/class/vps/mipi_host" + std::to_string(num) + "/status/cfg";
    std::ifstream mipi_host_fd(mipi_host);
    board_type_str = "";
    if (mipi_host_fd.is_open()) {
      std::getline(mipi_host_fd, board_type_str);
      if (board_type_str == "not inited") {
        stoped.push_back(num);
      } else {
        started.push_back(num);
      }
      mipi_host_fd.close();
    }
  }
}

bool HobotMipiCapIml::detectSensor(SENSOR_ID_T &sensor_info, int i2c_bus) {
  char cmd[256];
  char result[1024];
  memset(cmd, '\0', sizeof(cmd));
  memset(result, '\0', sizeof(result));
  if (sensor_info.i2c_addr_width == I2C_ADDR_8) {
    sprintf(cmd, "i2ctransfer -y -f %d w1@0x%x 0x%x r1 2>&1",
            i2c_bus,
            sensor_info.i2c_dev_addr,
            sensor_info.det_reg);
  } else if (sensor_info.i2c_addr_width == I2C_ADDR_16) {
    sprintf(cmd,
            "i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1",
            i2c_bus,
            sensor_info.i2c_dev_addr,
            sensor_info.det_reg >> 8,
            sensor_info.det_reg & 0xFF);
  } else {
    return false;
  }
  exec_cmd_ex(cmd, result, sizeof(result));
  if (strstr(result, "Error") == NULL && strstr(result, "error") == NULL) {
    // 返回结果中不带Error, 说明sensor找到了
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
          "match sensor:%s\n", sensor_info.sensor_name);
    return true;
  }
  return false;
}


bool HobotMipiCapIml::analysis_board_config() {
  std::string board_type;
  bool auto_detect = false;
  std::ifstream som_name("/sys/class/socinfo/som_name");
  if (som_name.is_open()) {
    if (!getline(som_name, board_type)) {
      som_name.close();
      return false;
    }
  } else {
    return false;
  }

  std::ifstream board_config("/etc/board_config.json");
  if (!board_config.is_open()) {
    return false;
  }
  std::string  board_name = "board_" + board_type;
  Json::Value root;
  board_config >> root;
  std::string reset;
  int i2c_bus;
  int mipi_host;
  int gpio_num;
  int reset_level;
  std::regex regexPattern(R"((\d+):(\w+))");
  try {
    int camera_num = root[board_name]["camera_num"].asInt();
    for (int i = 0; i < camera_num; i++) {
      mipi_host = root[board_name]["cameras"][i]["mipi_host"].asInt();
      i2c_bus = root[board_name]["cameras"][i]["i2c_bus"].asInt();
      board_config_m_[mipi_host].mipi_host = mipi_host;
      board_config_m_[mipi_host].i2c_bus = i2c_bus;
      board_config_m_[mipi_host].reset_flag = false;
      if (root[board_name]["cameras"][i].isMember("reset")){
        reset = root[board_name]["cameras"][i]["reset"].asString();
        std::smatch matches;
        if (std::regex_search(reset, matches, regexPattern)) {
          board_config_m_[mipi_host].reset_gpio = std::stoi(matches[1]);
          if (matches[2] == "low") {
            board_config_m_[mipi_host].reset_level = 1;
          } else {
            board_config_m_[mipi_host].reset_level = 0;
          } 
          board_config_m_[mipi_host].reset_flag = true;
        } 
      }
    }
  }catch (std::runtime_error& e) {
    return false;
  }
  return true;
}

int HobotMipiCapIml::selectSensor(std::string &sensor, int &host, int &i2c_bus) {

  // mipi sensor的信息数组
  SENSOR_ID_T sensor_id_list[] = {
    {1, 0x40, I2C_ADDR_8, 0x0B, "F37"},        // F37
    {1, 0x1a, I2C_ADDR_16, 0x0000, "imx415"},  // imx415
    {1, 0x29, I2C_ADDR_16, 0x03f0, "GC4663"},  // GC4663
    {1, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {1, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {1, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {1, 0x1a, I2C_ADDR_16, 0x0000, "imx586"},  // imx586
    {1, 0x29, I2C_ADDR_16, 0x0000, "gc4c33"},  // gc4c33
  };
  std::vector<int> i2c_buss= {0,1,2,3,4,5,6};

  SENSOR_ID_T *sensor_ptr = nullptr;
  for (auto sensor_id : sensor_id_list) {
    if(strcasecmp(sensor_id.sensor_name, sensor.c_str()) == 0) {
      sensor_ptr = &sensor_id;
      break;
    }
  }
  bool sensor_flag = false;
  if (sensor_ptr) {
    if (board_config_m_.size() > 0) {
      for (auto board : board_config_m_) {
        std::vector<int>::iterator it = std::find(mipi_stoped_.begin(), mipi_stoped_.end(), board.second.mipi_host);
        if (it == mipi_stoped_.end()) {
           continue;
        }
        if (detectSensor(*sensor_ptr, board.second.i2c_bus)) {
          host = board.second.mipi_host;
          i2c_bus = board.second.i2c_bus;
          sensor_flag = true;
          return 0;
        }
      }
    } else {
      for (auto num : i2c_buss) {
        if (detectSensor(*sensor_ptr, num)) {
          // host = mipi_stoped_[0];
          i2c_bus = num;
          sensor_flag = true;
          return 0;
        }
      }
    }
  }
  if (board_config_m_.size() > 0) {
    for (auto board : board_config_m_) {
      if (board.second.mipi_host == host) {
        for (auto sensor_id : sensor_id_list) {
          if (detectSensor(sensor_id, board.second.i2c_bus)) {
            host = board.second.mipi_host;
            i2c_bus = board.second.i2c_bus;
            sensor = sensor_id.sensor_name;
            sensor_flag = true;
            return 0;
          }
        }
      }
    }
  }
  if (board_config_m_.size() > 0) {
    for (auto board : board_config_m_) {
      std::vector<int>::iterator it = std::find(mipi_stoped_.begin(), mipi_stoped_.end(), board.second.mipi_host);
      if (it == mipi_stoped_.end()) {
          continue;
      }
      for (auto sensor_id : sensor_id_list) {
        if (detectSensor(sensor_id, board.second.i2c_bus)) {
          host = board.second.mipi_host;
          i2c_bus = board.second.i2c_bus;
          sensor = sensor_id.sensor_name;
          sensor_flag = true;
          return 0;
        }
      }
    }
  }
  for (auto num : i2c_buss) {
    for (auto sensor_id : sensor_id_list) {
      if (detectSensor(sensor_id, num)) {
        // host = mipi_stoped_[0];
        i2c_bus = num;
        sensor = sensor_id.sensor_name;
        sensor_flag = true;
        return 0;
      }
    }
  }
  return -1;
}

}  // namespace mipi_cam
