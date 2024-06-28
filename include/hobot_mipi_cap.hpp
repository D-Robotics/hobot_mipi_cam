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

#ifndef HOBOT_MIPI_CAP_HPP_
#define HOBOT_MIPI_CAP_HPP_

#include <vector>
#include <string>
#include "hobot_mipi_comm.hpp"

namespace mipi_cam {

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

  // 如果有 vps ，就 输出vps 的分层数据 channel--"single":单sensor，"left": 双目的左sensor，"right":双目的右sensor，"combine"：左右sensor拼合的图像。
  virtual int getFrame(std::string channel, int* nVOutW, int* nVOutH,
      void* buf, unsigned int bufsize, unsigned int*, uint64_t&, bool gray = false) = 0;


  // 获取cap的info信息；
  // 输入输出参数：MIPI_CAP_INFO_ST的结构信息。
  // 返回值：0，初始化成功；-1，初始化失败。
  virtual int getCapInfo(MIPI_CAP_INFO_ST &info) = 0;
};

}  // namespace mipi_cam


#endif  // HOBOT_MIPI_CAP_HPP_
