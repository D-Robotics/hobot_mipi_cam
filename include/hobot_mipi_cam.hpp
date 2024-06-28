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

#ifndef MIPI_MIPI_CAM_HPP_
#define MIPI_MIPI_CAM_HPP_

#include "sensor_msgs/msg/camera_info.hpp"

#include <string>
#include <vector>
#include <sstream>
#include <memory>

#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cap.hpp"

namespace mipi_cam
{
class MipiCam
{
 public:
  MipiCam() {}
  virtual ~MipiCam() {}

  static std::shared_ptr<MipiCam> create_mipicam();

  // 初始化摄像机
  // 输入参数：para是node传入的参数，包括sensor类型、名称，图像的宽、高等等。
  // 返回值：0，初始化成功，-1，初始化失败。
  virtual int init(struct NodePara &para) = 0;

  // 反初始化摄像机；
  // 返回值：0，反初始化成功；-1，反初始化失败。
  virtual int deInit() = 0;

  // 启动摄像机的码流；
  // 返回值：0，启动成功；-1，启动失败。
  virtual int start() = 0;

  // 停止摄像机的码流；
  // 返回值：0，停止成功；-1，停止失败。
  virtual int stop() = 0;

  // grabs a new image from the camera
  virtual bool getImage(
    builtin_interfaces::msg::Time & stamp,
    std::string & encoding,
    uint32_t & height, uint32_t & width, uint32_t & step,
    std::vector<uint8_t> & data, std::string channel) = 0;

  // grabs a new hbmem's image hbmem from the camera
  virtual bool getImageMem(
    // uint64_t & stamp,
    builtin_interfaces::msg::Time & stamp,
    std::array<uint8_t, 12> & encoding,
    uint32_t & height, uint32_t & width, uint32_t & step,
    std::array<uint8_t, 6220800> & data, uint32_t & data_size, std::string channel) = 0;

  // gen camera calibration
  virtual bool getCamCalibration(sensor_msgs::msg::CameraInfo& cam_info,
               const std::string &file_path) = 0;

  virtual bool isCapturing() = 0;
};

}  // namespace mipi_cam

#endif  // MIPI_MIPI_CAM_HPP_
