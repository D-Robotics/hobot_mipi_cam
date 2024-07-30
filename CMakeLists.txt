# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0077 NEW)
project(mipi_cam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-w)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## pkg-config libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rclcpp_components REQUIRED)

#set(PREFIX_PATH x3)

if(PLATFORM_X3)
  message("build platform X3")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
  set(SRC_PLATFORM_FILE 
    src/x3/x3_sdk_swap/src/x3_sdk_wrap.cpp
    src/x3/x3_sdk_swap/src/x3_vio_bind.cpp
    src/x3/x3_sdk_swap/src/x3_vio_vin.cpp
    src/x3/x3_sdk_swap/src/x3_vio_vps.cpp
    src/x3/x3_sdk_swap/src/x3_vio_vp.cpp
    src/x3/sensor/sensor_f37_config.c
    src/x3/sensor/sensor_gc4c33_config.c
    src/x3/sensor/sensor_gc4663_config.c
    src/x3/sensor/sensor_imx415_config.c
    src/x3/sensor/sensor_imx586_config.c
    src/x3/sensor/sensor_imx219_config.c
    src/x3/sensor/sensor_imx477_config.c
    src/x3/sensor/sensor_ov5647_config.c
    src/x3/sensor/x3_preparam.c
  )
  include_directories(include 
    src/x3/x3_sdk_swap/include
    src/x3/sensor
  )
elseif(PLATFORM_Rdkultra)
  message("build platform rdkultra")
  add_definitions(-DPLATFORM_Rdkultra)
  set(PREFIX_PATH rdkultra)
  include_directories(${CMAKE_SYSROOT}/usr/include/cam
    ${CMAKE_SYSROOT}/usr/include/isp
    ${CMAKE_SYSROOT}/usr/include/vps/vpm
  )
elseif(PLATFORM_X5)
  message("build platform X5")
  add_definitions(-DPLATFORM_X5)
  set(PREFIX_PATH x5)
  set(SRC_PLATFORM_FILE 
    src/x5/sensor/f37_linear_1920x1080_raw10_30fps_1lane.c
    src/x5/sensor/irs2875_linear_208x1413_raw12_15fps_2lane.c
    src/x5/sensor/ov5640_linear_1920x1080_raw10_30fps_2lane.c
    src/x5/sensor/sc035hgs_linear_640x480_raw10_30fps_1lane.c
    src/x5/sensor/sc132gs_linear_896x896_raw10_10fps_2lane.c
    src/x5/sensor/sc132gs_linear_1088x1280_raw10_10fps_2lane.c
    src/x5/sensor/sc230ai_linear_1920x1080_raw10_10fps_1lane.c
    src/x5/sensor/sc230ai_linear_1920x1080_raw10_30fps_1lane.c
    src/x5/sensor/sc1330t_linear_1280x960_raw10_30fps_1lane.c
    src/x5/sensor/vp_sensors.c
  )
  include_directories(include 
    src/x5/sensor
  )
else()
  message("invalid platform, build platform X3 default")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
endif()

message("PREFIX_PATH: ${PREFIX_PATH}")

include_directories(include
  include/${PREFIX_PATH}
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_SYSROOT}/usr/
  ${CMAKE_SYSROOT}/usr/include
  ${CMAKE_SYSROOT}/usr/include/opencv/
  deps/jsoncpp/include/
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib
  ${CMAKE_SYSROOT}/usr/lib/hbmedia
  ${CMAKE_SYSROOT}/usr/lib/system/lib
  deps/jsoncpp/lib
)

set(BASE_LIBRARIES pthread dl)
if(PLATFORM_X3)
  set(X3_HAPI_LIB isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson diag gdcbin  jsoncpp)
elseif(PLATFORM_Rdkultra)
  #set(X3_HAPI_LIB jsoncpp hb_api_isp vpm vio_client)
  set(X3_HAPI_LIB jsoncpp hb_api_isp vio)
elseif(PLATFORM_X5)
  set(X3_HAPI_LIB jsoncpp multimedia hbmem cam vpf  Nano2D Nano2Dutil opencv_world)
else()
  set(BASE_LIBRARIES pthread vio cam dl)
  set(X3_HAPI_LIB isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson diag gdcbin)
endif()

# build component
add_library(${PROJECT_NAME}_plugin SHARED
  src/camera.cpp
  src/hobot_mipi_node.cpp
  src/hobot_mipi_cam.cpp
  src/${PREFIX_PATH}/hobot_mipi_cap_iml.cpp
  src/${PREFIX_PATH}/hobot_mipi_factory.cpp
  ${SRC_PLATFORM_FILE}
)
target_link_libraries(${PROJECT_NAME}_plugin
  ${BASE_LIBRARIES} ${X3_HAPI_LIB} yaml-cpp
)
ament_target_dependencies(
  ${PROJECT_NAME}_plugin
  rclcpp
  rclcpp_components
  sensor_msgs
  yaml_cpp_vendor
  hbm_img_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_plugin
  PLUGIN "mipi_cam::MipiCamNode"
  EXECUTABLE ${PROJECT_NAME}
)
set(node_plugins "${node_plugins}mipi_cam::MipiCamNode;$<TARGET_FILE:${PROJECT_NAME}_plugin>\n")

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_plugin)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}_plugin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(PLATFORM_X3)
  install(DIRECTORY
    ${PROJECT_SOURCE_DIR}/config/x3/
    DESTINATION lib/${PROJECT_NAME}/config/
  )
elseif(PLATFORM_Rdkultra)
  install(DIRECTORY
    ${PROJECT_SOURCE_DIR}/config/rdkultra/
    DESTINATION lib/${PROJECT_NAME}/config/
  )
elseif(PLATFORM_X5)
  install(DIRECTORY
    ${PROJECT_SOURCE_DIR}/config/x5/
    DESTINATION lib/${PROJECT_NAME}/config/
  )
endif()


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
