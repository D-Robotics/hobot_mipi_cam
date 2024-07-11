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
#include "isp/hb_api_isp.h"

#include "sensor_msgs/msg/camera_info.hpp"

#include <string>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <regex>

#include <rclcpp/rclcpp.hpp>
#include <json/json.h>

namespace mipi_cam {

int HobotMipiCapIml::UpdateConfig(MIPI_CAP_INFO_ST &info) {
  RCLCPP_DEBUG(rclcpp::get_logger("mipi_cam"),
      "config_path : %s.\n", info.config_path.c_str());
  if ((info.sensor_type == "IMX219") || (info.sensor_type == "imx219")) {
    vio_cfg_file_ = info.config_path + "/imx219/vpm.json";
    cam_cfg_file_ = info.config_path + "/imx219/cam.json";
    mipi_cfg_file_ = info.config_path + "/imx219/mipi.json";
    cim_cfg_file_ = info.config_path + "/imx219/cim.json";
    cam_cfg_index_ = 0;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s]->sensor %s not found.\n", __func__, info.sensor_type.c_str());
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
      "vio_cfg_file_ : %s.\n", vio_cfg_file_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
      "cam_cfg_file_ : %s.\n", cam_cfg_file_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
      "pipeline_idx_ : %d.\n", pipeline_idx_);

  std::string pipeline_str = "pipeline" + std::to_string(pipeline_idx_);
  std::string port_str = "port_" + std::to_string(pipeline_idx_);
  std::string config_str = "config_" + std::to_string(cam_cfg_index_);
  {
    std::ifstream cam_file(cam_cfg_file_);
    if (!cam_file.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "[%s]->open file error (%s).\n", __func__, cam_cfg_file_.c_str());
      return -1;
    }
    Json::Value root;
    cam_file >> root;
    try {
      root[config_str]["port_mask"] = (1 << pipeline_idx_);
      root[config_str][port_str]["entry_num"] = entry_index_;
      root[config_str][port_str]["bus_num"] = sensor_bus_;
      // root[config_str][port_str]["dev_port"] = pipeline_idx_;
      // cim_cfg_file_ = root[config_str][port_str]["data_path"].asString();
      root[config_str][port_str]["data_path"] = cim_cfg_file_;
      root[config_str][port_str]["config_path"] = mipi_cfg_file_;
      cam_file.close();
      std::ofstream ofs(cam_cfg_file_);
      Json::StyledStreamWriter writer;
      writer.write(ofs, root);
      ofs.close();
    }catch (std::runtime_error& e) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "[%s]->update  (%s) config error.\n", __func__, cam_cfg_file_.c_str());
      cam_file.close();
      return -1;
    }
  }
  {
    std::ifstream cim_file(cim_cfg_file_);
    if (!cim_file.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "[%s]->open file error (%s).\n", __func__, cim_cfg_file_.c_str());
      return -1;
    }
    Json::Value root;
    cim_file >> root;
    try {
      if (!root.isMember(pipeline_str)) {
        std::string tmp_pipeline;
        for (int i = 0; i < 16; i++) {
          tmp_pipeline  = "pipeline" + std::to_string(i);
          if (root.isMember(tmp_pipeline)) {
            Json::Value oldValue = root[tmp_pipeline];
            root[pipeline_str] = oldValue;
            root.removeMember(tmp_pipeline);
            break;
          }
        }
      }
      if (entry_index_ < 2) {
        root[pipeline_str]["cim"]["input"]["mipi"]["enable"] = 1;
        root[pipeline_str]["cim_dma"]["input"]["mipi"]["enable"] = 0;
        root[pipeline_str]["cim"]["input"]["mipi"]["rx_index"] = entry_index_;
        root[pipeline_str]["cim"]["output"]["isp0"]["isp_ch0"] = 0;
        root[pipeline_str]["cim"]["output"]["isp0"]["isp_ch0"] = 0;
        root[pipeline_str]["cim"]["output"]["isp0"]["isp_ch0"] = 0;
        root[pipeline_str]["cim"]["output"]["isp0"]["isp_ch0"] = 0;
        int tmp_index = entry_index_ * 2;
        if (tmp_index == 0) {
          root[pipeline_str]["cim"]["output"]["isp0"]["isp_ch0"] = 1;
        } else if (tmp_index == 1) {
          root[pipeline_str]["cim"]["output"]["isp0"]["isp_ch1"] = 1;
        } else if (tmp_index == 2) {
          root[pipeline_str]["cim"]["output"]["isp0"]["isp_ch2"] = 1;
        } else if (tmp_index == 3) {
          root[pipeline_str]["cim"]["output"]["isp0"]["isp_ch3"] = 1;
        }
      } else {
        root[pipeline_str]["cim"]["input"]["mipi"]["enable"] = 0;
        root[pipeline_str]["cim_dma"]["input"]["mipi"]["enable"] = 1;
        root[pipeline_str]["cim_dma"]["input"]["mipi"]["rx_index"] = entry_index_;
      }
      cim_file.close();
      std::ofstream ofs(cim_cfg_file_);
      Json::StyledStreamWriter writer;
      writer.write(ofs, root);
      ofs.close();
    }catch (std::runtime_error& e) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "[%s]->update  (%s) config error.\n", __func__, cim_cfg_file_.c_str());
      cim_file.close();
      return -1;
    }
  }

  {
    std::ifstream vio_file(vio_cfg_file_);
    if (!vio_file.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "[%s]->open file error (%s).\n", __func__, vio_cfg_file_.c_str());
      return -1;
    }
    Json::Value root;
    vio_file >> root;
    int w, h;
    int pym_layer = -1;
    try {
      if (!root.isMember(pipeline_str)) {
        std::string tmp_pipeline;
        for (int i = 0; i < 16; i++) {
          tmp_pipeline  = "pipeline" + std::to_string(i);
          if (root.isMember(tmp_pipeline)) {
            Json::Value oldValue = root[tmp_pipeline];
            root[pipeline_str] = oldValue;
            root.removeMember(tmp_pipeline);
            break;
          }
        }
      }
      root[pipeline_str]["isp"]["ctx_id"] = (entry_index_ * 2);
      src_width_ = root[pipeline_str]["isp"]["width"].asInt();
      src_height_ = root[pipeline_str]["isp"]["height"].asInt();
      u_int32_t ds_roi_en = root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi_en"].asInt();
      bool ds_en = false;

  #if 0
      if ((info.width > src_width_) || (info.height > src_height_)) {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "[%s]-> w*h:%d*%d > src_w*src_h:%d*%d\n",
        __func__, info.width, info.height, src_width_, src_height_);
        return -1;
      } else if ((info.width == src_width_) && (info.height == src_height_)) {
      } else if (info.width >= src_width_ / 2) {
        ds_pym_layer_ = 0;
        use_ds_roi_ = true;
      } else if (info.width >= src_width_ / 4) {
        ds_pym_layer_ = 1;
        use_ds_roi_ = true;
      } else if (info.width >= src_width_ / 8) {
        ds_pym_layer_ = 2;
        use_ds_roi_ = true;
      } else if (info.width >= src_width_ / 16) {
        ds_pym_layer_ = 3;
        use_ds_roi_ = true;
      } else if (info.width >= src_width_ / 32) {
        ds_pym_layer_ = 4;
        use_ds_roi_ = true;
      }
      if (use_ds_roi_) {
        root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][ds_pym_layer_]["ds_roi_layer"] = 0;
        root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][ds_pym_layer_]["ds_roi_sel"] = 0;
        root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][ds_pym_layer_]["ds_roi_start_top"] = 0;
        root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][ds_pym_layer_]["ds_roi_start_left"] = 0;
        root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][ds_pym_layer_]["ds_roi_region_width"] = info.width;
        root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][ds_pym_layer_]["ds_roi_region_height"] = info.height;
        root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][ds_pym_layer_]["ds_roi_stride_y"] = info.width;
        root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][ds_pym_layer_]["ds_roi_stride_uv"] = info.width;
        root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][ds_pym_layer_]["ds_roi_out_width"] = info.width;
        root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][ds_pym_layer_]["ds_roi_out_height"] = info.height;
        root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi_en"] = (ds_roi_en | (1 << ds_pym_layer_));
      }
  #else
      if ((info.width > src_width_) || (info.height > src_height_)) {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "[%s]-> w*h:%d*%d > src_w*src_h:%d*%d\n",
        __func__, info.width, info.height, src_width_, src_height_);
        return -1;
      } else if ((info.width == src_width_) && (info.height == src_height_)) {
      } else {
        u_int32_t  tmp_width;
        u_int32_t  tmp_height;
        u_int32_t  roi_width;
        u_int32_t  roi_height;
        u_int32_t  ds_en = root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi_en"].asInt();;
        ds_pym_layer_ = 0;
        for (u_int32_t i = 0; i < 4; i++) {
          if (!(ds_en & (1 << i))) {
            continue;
          }
          tmp_width = root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][i]["ds_roi_region_width"].asInt();
          tmp_height = root[pipeline_str]["pym"]["pym_ctrl"]["ds_roi"][i]["ds_roi_region_height"].asInt();
          if ((0 == tmp_width) || (0 == tmp_height)) {
            break;
          } else if ((info.width == tmp_width) && (info.height == tmp_height)) {
            ds_pym_layer_ = i;
            use_ds_roi_ = true;
            roi_width = tmp_width;
            roi_height = tmp_height;
            break;
          } else if ((info.width <= tmp_width) && (info.height <= tmp_height)) {
            ds_pym_layer_ = i;
            use_ds_roi_ = true;
            roi_width = tmp_width;
            roi_height = tmp_height;
          } else if ((info.width > tmp_width) || (info.height > tmp_height)) {
            break;
          }
        }
        if (use_ds_roi_) {
          info.width = roi_width;
          info.height = roi_height;
        } else {
          info.width = src_width_;
          info.height = src_height_;
        }
      }
  #endif
      vio_file.close();
      std::ofstream ofs(vio_cfg_file_);
      Json::StyledStreamWriter writer;
      writer.write(ofs, root);
      ofs.close();
    }catch (std::runtime_error& e) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "[%s]->update json config error.\n", __func__);
      vio_file.close();
      return -1;
    }
    
  }

  return 0;
}

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
    std::vector<std::string> sys_cmds;
    std::string cmd_name;
    for (int num : mipi_stoped_) {
      cmd_name = "echo 1 > /sys/class/vps/mipi_host" + std::to_string(num) + "/param/snrclk_en";
      sys_cmds.push_back(cmd_name);
      cmd_name = "echo 24000000 > /sys/class/vps/mipi_host" + std::to_string(num) + "/param/snrclk_freq";
      sys_cmds.push_back(cmd_name);
      cmd_name = "echo 1 > /sys/class/vps/mipi_host" + std::to_string(num) + "/param/stop_check_instart";
      sys_cmds.push_back(cmd_name);
    }
    for (const auto& sys_cmd : sys_cmds) {
      system(sys_cmd.data());
    }
  }
  return 0;
}

int HobotMipiCapIml::init(MIPI_CAP_INFO_ST &info) {
  int ret = 0;
  cap_info_ = info;
  
  if (cap_info_.device_mode_.compare("dual") == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "RDK Ultra platform no suppot dual channle camera.\n");
    return -1;
  }


  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "Rdkultra init start.\n");

  entry_index_ = cap_info_.channel_;
  if (selectSensor(cap_info_.sensor_type, entry_index_, sensor_bus_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "not select  sensor!\n");
    return -1;
  }
  int pipeline_id = 0;
  for (; pipeline_id < 16; pipeline_id++) {
    if (!checkPipelineOpened(pipeline_id)) {
      break;
    }
  }
  if (pipeline_id >= 16) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "pipeline is over number 8\n");
    return -1;
  }
  pipeline_idx_ = pipeline_id;
  
  ret = UpdateConfig(cap_info_);
  if (ret) {
    return -1;
  }

  ret = hb_vio_cfg_check(vio_cfg_file_.c_str(), cam_cfg_file_.c_str(),
         cam_cfg_index_);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "vcs check cfg fail vpm:%s\n vin:%s\n index:%d,error:%d\n",
      cam_cfg_file_.c_str(), vio_cfg_file_.c_str(), cam_cfg_index_, ret);
    return -1;
   }

  ret = hb_vio_init(vio_cfg_file_.c_str());
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "hb_vio_init failed, %d", ret);
    return -1;
  } else {
    vio_inited_ = true;
  }
  ret = hb_cam_init(cam_cfg_index_, cam_cfg_file_.c_str());
  if (ret != 0) {
    hb_vio_deinit();
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "hb_cam_init failed, %d", ret);
    return -1;
  } else {
    cam_inited_ = true;
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "Rdkultra camera init success.\n");
  return ret;
}

int HobotMipiCapIml::deInit() {
  int i = 0;
  int ret;
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "cam_deinit start.\n");
  if (cam_inited_) {
    ret = hb_cam_deinit(cam_cfg_index_);
    if (ret < 0) {
     RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
       "hb_cam_deinit fail %d\n", ret);
    }
  }
  if (vio_inited_) {
    ret = hb_vio_deinit();
    if (ret < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "hb_vio_deinit fail %d\n", ret);
    }
  }
  return 0;
}

int HobotMipiCapIml::start() {
  int ret = 0;
  int port = 0;
  if (!cam_inited_) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "rdkultra camera isn't inited");
    return -1;
  }
  ret = hb_vio_start_pipeline(pipeline_idx_);  // 使能数据通路
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "%d hb_vio_start_pipeline pipeid:%d fail ret %d\n",
      __LINE__, pipeline_idx_, ret);
    return ret;
  }
  ret = hb_cam_start(pipeline_idx_);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "%d hb_cam_start pipeid:%d fail ret %d\n", __LINE__, port, ret);
    return ret;
  }
  hb_isp_dev_init();
  started_ = true;
  return 0;
}

int HobotMipiCapIml::stop() {
  int ret = 0;
  int port = 0;
  if (!started_) {
     RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "rdkultra camera isn't started");
    return -1;
  }
  ret = hb_cam_stop(pipeline_idx_);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "%d hb_cam_stop pipeid:%d ret %d\n", __LINE__, pipeline_idx_, ret);
    return ret;
  }
  ret = hb_vio_stop_pipeline(pipeline_idx_);  // 停止数据通道
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "%d hb_vio_stop_pipeline pipeid:%d ret %d\n",
      __LINE__, pipeline_idx_, ret);
    return ret;
  }
  return 0;
}

int HobotMipiCapIml::getFrame(std::string channel, int* nVOutW, int* nVOutH,
        void* frame_buf, unsigned int bufsize, unsigned int* len,
        uint64_t &timestamp, bool gray) {
  int size = -1, ret = 0;
  struct timeval select_timeout = {0};
  pym_buffer_v2_t pym_buf;
  hb_vio_buffer_t isp_buf;
  pym_buffer_common_t pym_common_buf;
  int pipe_id = 0;
  int i = 0;
  int stride = 0, width = 0, height = 0;
  if (!started_) {
     RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_cam"),
      "rdkultra camera isn't started");
    return -1;
  }
  do {
#if 1
    ret = hb_vio_get_data(pipeline_idx_, HB_VIO_PYM_DATA_V2, &pym_buf);
    if (ret < 0) {
      if (ret == -24) {
        RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_cam"),
          "hb_vio_get_data(pipe:%d) no available buf ret:%d\n",
          pipeline_idx_, ret);
      }
      else{
      }
      usleep(10 * 1000);
      continue;
    }
    timestamp = pym_buf.pym_img_info.tv.tv_sec * 1e9
              + pym_buf.pym_img_info.tv.tv_usec * 1e3;
    address_info_t *pym_addr;
    if (use_ds_roi_) {
      pym_addr = &pym_buf.ds_roi[ds_pym_layer_];
    } else {
      pym_addr = &pym_buf.src_out;
    }
    stride = pym_addr->stride_size;
    width = pym_addr->width;
    height = pym_addr->height;

    *nVOutW = width;
    *nVOutH = height;
    if (gray == true) {
      *len = width * height;
    } else {
      *len = width * height * 3 / 2;
    }
    if (bufsize < *len) {
      hb_vio_free_pymbuf(pipe_id, HB_VIO_PYM_DATA_V2, &pym_buf);
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "buf size(%d) < frame size(%d)", bufsize, *len);
      return -1;
    }
    if (stride == width) {
      memcpy(frame_buf, pym_addr->addr[0], width * height);
      if (gray == false) {
        memcpy(frame_buf + width * height,
             pym_addr->addr[1],
             width * height / 2);
      }
    } else {
      // jump over stride - width Y
      for (i = 0; i < height; i++) {
        memcpy(frame_buf + i * width,
          pym_addr->addr[0] + i * stride, width);
      }
      // jump over stride - width UV
      if (gray == false) {
        for (i = 0; i < height / 2; i++) {
          memcpy(frame_buf + width * height + i * width,
               pym_addr->addr[1] + i * stride,
               width);
        }
      }
    }
    hb_vio_free_pymbuf(pipeline_idx_, HB_VIO_PYM_DATA_V2, &pym_buf);
    break;
#elif 0
    ret = hb_vio_get_data(pipeline_idx_, HB_VIO_PYM_COMMON_DATA, &pym_common_buf);
    if (ret < 0) {
      if (ret == -24) {
        RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_cam"),
          "hb_vio_get_data(pipe:%d) no available buf ret:%d\n",
          pipeline_idx_, ret);
      } else{
      }
      usleep(10 * 1000);
      continue;
    }
    timestamp = pym_common_buf.pym_img_info.tv.tv_sec * 1e9
                + pym_common_buf.pym_img_info.tv.tv_usec * 1e3;
    address_info_t *pym_info = &(pym_common_buf.pym[0]);
    stride = pym_info->stride_size;
    width = pym_info->width;
    height = pym_info->height;
    *nVOutW = width;
    *nVOutH = height;
    if (gray == true) {
      *len = width * height;
    } else {
      *len = width * height * 3 / 2;
    }
    if (bufsize < *len) {
      hb_vio_free_pymbuf(pipe_id, HB_VIO_PYM_COMMON_DATA, &pym_common_buf);
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "buf size(%d) < frame size(%d)", bufsize, *len);
      return -1;
    }
    if (stride == width) {
      memcpy(frame_buf, pym_info->addr[0], width * height);
      if (gray == false) {
        memcpy(frame_buf + width * height,
             pym_info->addr[1],
             width * height / 2);
      }
    } else {
      // jump over stride - width Y
      for (i = 0; i < height; i++) {
        memcpy(frame_buf + i * width,
          pym_info->addr[0] + i * stride, width);
      }
      // jump over stride - width UV
      if (gray == false) {
        for (i = 0; i < height / 2; i++) {
          memcpy(frame_buf + width * height + i * width,
               pym_info->addr[1] + i * stride,
               width);
        }
      }
    }
    hb_vio_free_pymbuf(pipeline_idx_, HB_VIO_PYM_COMMON_DATA, &pym_common_buf);
    break;
#else
    ret = hb_vio_get_data(pipeline_idx_, HB_VIO_ISP_YUV_DATA, &isp_buf);
    if (ret < 0) {
      if (ret == -24) {
        RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_cam"),
          "hb_vio_get_data(pipe:%d) no available buf ret:%d\n",
          pipeline_idx_, ret);
      } else{
      }
      usleep(10 * 1000);
      continue;
    }
    timestamp = isp_buf.img_info.tv.tv_sec * 1e9 + isp_buf.img_info.tv.tv_usec * 1e3;
    stride = isp_buf.img_addr.stride_size;
    width = isp_buf.img_addr.width;
    height = isp_buf.img_addr.height;
    stride = width;
    *nVOutW = width;
    *nVOutH = height;
    if (gray == true) {
      *len = width * height;
    } else {
      *len = width * height * 3 / 2;
    }
    if (bufsize < *len) {
      hb_vio_free_pymbuf(pipe_id, HB_VIO_ISP_YUV_DATA, &isp_buf);
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "buf size(%d) < frame size(%d)", bufsize, *len);
      return -1;
    }
    if (stride == width) {
      memcpy(frame_buf, isp_buf.img_addr.addr[0], width * height);
      if (gray == false) {
        memcpy(frame_buf + width * height,
             isp_buf.img_addr.addr[1],
             width * height / 2);
      }
    } else {
      // jump over stride - width Y
      for (i = 0; i < height; i++) {
        memcpy(frame_buf + i * width,
          isp_buf.img_addr.addr[0] + i * stride, width);
      }
      // jump over stride - width UV
      if (gray == false) {
        for (i = 0; i < height / 2; i++) {
          memcpy(frame_buf + width * height + i * width,
               isp_buf.img_addr.addr[1] + i * stride,
               width);
        }
      }
    }
    hb_vio_free_pymbuf(pipeline_idx_, HB_VIO_ISP_YUV_DATA, &isp_buf);
    break;
#endif
  } while (1);
  return 0;
}

bool HobotMipiCapIml::checkPipelineOpened(int pipeline_idx) {
  std::string fps_info;
  std::string tmp_str = "cim pipe " + std::to_string(pipeline_idx);
  std::ifstream fps_file("/sys/devices/platform/soc/47010000.cam_sys/47060000.cim/fps");
  if (fps_file.is_open()) {
    while (std::getline(fps_file, fps_info)) {
      if(strncmp(fps_info.c_str(), tmp_str.c_str(), tmp_str.length()) == 0) {
        fps_file.close();
        return true;
      }
    }
  }
  fps_file.close();
  tmp_str = "cimdma pipe " + std::to_string(pipeline_idx);
  std::ifstream cim_dma_fps_file("/sys/devices/platform/soc/47010000.cam_sys/47180000.cim_dma/fps");
  if (cim_dma_fps_file.is_open()) {
    while (std::getline(cim_dma_fps_file, fps_info)) {
      if(strncmp(fps_info.c_str(), tmp_str.c_str(), tmp_str.length()) == 0) {
        cim_dma_fps_file.close();
        return true;
      }
    }
  }
  cim_dma_fps_file.close();
  return false;
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
  std::ifstream som_name("/sys/class/socinfo/board_id");
  if (som_name.is_open()) {
    if (!getline(som_name, board_type)) {
      som_name.close();
      return false;
    }
  } else {
    return false;
  }
  som_name.close();

  std::ifstream board_config("/etc/board_config.json");
  if (board_config.is_open() == false) {
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
     {5, 0x10, I2C_ADDR_16, 0x0000, "imx219"},
  };
  std::vector<int> i2c_buss= {1,2,4,5,6};

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
      if (mipi_started_.size() > 0) {
        return -1;
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

int HobotMipiCapImlRDKRdkultra::initEnv() {
  #if 0
  std::vector<int> mipi_hosts = {0,1};
  std::vector<int> mipi_started;
  std::vector<int> mipi_stoped;
  listMipiHost(mipi_hosts, mipi_started, mipi_stoped);
  if (mipi_stoped.size() == 0) {
    return -1;
  }
  std::vector<std::string> sys_cmds;
  for (int num : mipi_stoped) {
    switch (num) {
      case 0:
        sys_cmds.push_back("echo 293 > /sys/class/gpio/export");
        sys_cmds.push_back("echo out > /sys/class/gpio/gpio293/direction");
        sys_cmds.push_back("echo 1 > /sys/class/gpio/gpio293/value");
        sys_cmds.push_back("echo 293 > /sys/class/gpio/unexport");
        sys_cmds.push_back("echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart");
        break;
      case 1:
        sys_cmds.push_back("echo 290 > /sys/class/gpio/export");
        sys_cmds.push_back("echo out > /sys/class/gpio/gpio290/direction");
        sys_cmds.push_back("echo 0 > /sys/class/gpio/gpio290/value");
        sys_cmds.push_back("sleep 0.5");
        sys_cmds.push_back("echo 1 > /sys/class/gpio/gpio290/value");
        sys_cmds.push_back("echo 290 > /sys/class/gpio/unexport");
        sys_cmds.push_back("echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart");
        break;
      default:
        break;
    }
  }
  for (const auto& sys_cmd : sys_cmds) {
    system(sys_cmd.data());
  }
  #else 
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
  if ((mipi_stoped_.size() == 0))  {
    return -1;
  }
  std::vector<std::string> sys_cmds;
  std::string cmd_name;
  #if 0
  for (auto board : board_config_m_) {
    if (board.second.reset_flag) {
      cmd_name = "echo " + std::to_string(board.second.reset_gpio) + " > /sys/class/gpio/export";
      sys_cmds.push_back(cmd_name);
      cmd_name = "echo out > /sys/class/gpio/gpio" + std::to_string(board.second.reset_gpio) + "/direction";
      sys_cmds.push_back(cmd_name);
      cmd_name = "echo " + std::to_string(board.second.reset_level) + " > /sys/class/gpio/gpio" + std::to_string(board.second.reset_gpio) + "/value";
      sys_cmds.push_back(cmd_name);
      cmd_name = "echo " + std::to_string(board.second.reset_gpio) + " > /sys/class/gpio/unexport";
      sys_cmds.push_back(cmd_name);
    }
  }
#endif
  for (int num : mipi_stoped_) {
    cmd_name = "echo 1 > /sys/class/vps/mipi_host" + std::to_string(num) + "/param/snrclk_en";
    sys_cmds.push_back(cmd_name);
    cmd_name = "echo 24000000 > /sys/class/vps/mipi_host" + std::to_string(num) + "/param/snrclk_freq";
    sys_cmds.push_back(cmd_name);
    cmd_name = "echo 1 > /sys/class/vps/mipi_host" + std::to_string(num) + "/param/stop_check_instart";
    sys_cmds.push_back(cmd_name);
  }
  for (const auto& sys_cmd : sys_cmds) {
    system(sys_cmd.data());
  }
  #endif
  return 0;
}

}  // namespace mipi_cam
