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
#include "hobot_mipi_cap.hpp"
#include "hobot_mipi_comm.hpp"
#include "x3_sdk_wrap.h"

namespace mipi_cam {

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

  // 如果有 vps ，就 输出vps 的分层数据
  int getFrame(int nChnID, int* nVOutW, int* nVOutH,
        void* buf, unsigned int bufsize, unsigned int*, uint64_t&, bool gray = false);

  int parseConfig(std::string sensor_name, int w, int h, int fps);

  // 检测对应的pipeline是否已经打开；
  // 输入参数：pipeline_idx pipeline的group ID。
  // 返回值：true，已经打开；false，没有打开。
  bool checkPipelineOpened(int pipeline_idx);

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

  bool m_inited_ = false;
  bool started_ = false;
  x3_vin_info_t vin_info_;
  x3_vps_infos_t vps_infos_;  // vps的配置，支持多个vps group
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
};

}  // namespace mipi_cam


#endif  // HOBOT_MIPI_CAP_IML_HPP_
