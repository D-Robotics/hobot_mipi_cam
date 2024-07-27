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

#ifndef HOBOT_MIPI_FACTORY_HPP_
#define HOBOT_MIPI_FACTORY_HPP_

#include <memory>
#include <string>
#include "hobot_mipi_cap.hpp"

namespace mipi_cam {
  // 创建sensor数据捕抓对象HobotMipiCap,工厂模式。
  // dev_name--板级类型名称，如x3pi,x3sdb,rdkultra。
  std::shared_ptr<HobotMipiCap> createMipiCap(const std::string &dev_name);

  // 检测当前板子的类型，。
  // 返回值--板级类型名称，如x3pi,x3sdb,rdkultra。
  std::string getBoardType();

}  // namespace mipi_cam


#endif  // HOBOT_MIPI_FACTORY_HPP_
