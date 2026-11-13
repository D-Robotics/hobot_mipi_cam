cmake_minimum_required(VERSION 3.10)
project(mipi_cam_eval)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# 可选：hbmem 零拷贝消息（板端全量构建可用；缺失时 hbmem 订阅不编译）
find_package(hbm_img_msgs QUIET)

# 主包共享 include（埋点统一接口头 hobot_mipi_eval_log.hpp）与包自身 include
set(MIPI_CAM_EVAL_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_library(recorder_lib src/recorder_node.cpp)
target_include_directories(recorder_lib PUBLIC ${MIPI_CAM_EVAL_INCLUDES})
ament_target_dependencies(recorder_lib rclcpp sensor_msgs)
if(hbm_img_msgs_FOUND)
  ament_target_dependencies(recorder_lib hbm_img_msgs)
  target_compile_definitions(recorder_lib PUBLIC EVAL_WITH_HBMEM)
  message(STATUS "mipi_cam_eval: hbm_img_msgs found, hbmem topics enabled")
else()
  message(STATUS "mipi_cam_eval: hbm_img_msgs not found, hbmem topics disabled")
endif()

add_executable(recorder src/recorder_main.cpp)
target_link_libraries(recorder recorder_lib)
ament_target_dependencies(recorder rclcpp)

install(TARGETS recorder
  DESTINATION lib/${PROJECT_NAME})

# 评测 launch / 用例（产品 launch 不改）
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY cases DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml")
# run_case.sh 只在仓库内运行（依赖 eval/cases 与仓库 install/setup.bash），不安装到 share

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    # 缝3：埋点输出契约（纯函数，无 ROS 依赖）
    ament_add_gtest(eval_log_contract_test test/eval_log_contract_test.cpp)
    target_include_directories(eval_log_contract_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    # 缝2：recorder（ROS2 topic 边界，合成发布者灌消息）
    ament_add_gtest(recorder_test test/recorder_test.cpp)
    target_link_libraries(recorder_test recorder_lib)
  else()
    message(STATUS "mipi_cam_eval: ament_cmake_gtest not found, tests skipped")
  endif()
endif()

ament_package()
