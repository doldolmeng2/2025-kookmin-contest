cmake_minimum_required(VERSION 3.8)
project(lane_detection)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# ---------- Libraries ----------

# preprocess (영상 전처리)
add_library(preprocess
  src/preprocess.cpp
)
target_include_directories(preprocess PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(preprocess
  ${OpenCV_LIBS}
)

# config_io (JSON 로드/검증)
add_library(config_io
  src/config_io.cpp
)
target_include_directories(config_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(config_io
  ament_index_cpp
)
target_link_libraries(config_io
  nlohmann_json::nlohmann_json
)

# offset_calculate (라인/오프셋 계산)
add_library(lane_offset
  src/offset_calculate.cpp
)
target_include_directories(lane_offset PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# OpenCV는 ament 패키지가 아니므로 target_link_libraries로 연결
target_link_libraries(lane_offset
  ${OpenCV_LIBS}
)

# ---------- Executables ----------

# mask_tuner
add_executable(mask_tuner
  src/mask_tuner.cpp
)
target_include_directories(mask_tuner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mask_tuner
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(mask_tuner
  ${OpenCV_LIBS}
  preprocess
  config_io
  lane_offset
)

# lane_detection node
add_executable(lane_detection_node
  src/lane_detection.cpp
)
target_include_directories(lane_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lane_detection_node
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
)
target_link_libraries(lane_detection_node
  ${OpenCV_LIBS}
  preprocess
  config_io
  lane_offset    # <-- 링크 추가!
)

# ---------- Install ----------

install(TARGETS
  preprocess
  config_io
  lane_offset       # <-- 설치 추가!
  mask_tuner
  lane_detection_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# 기본 설정 파일 배포 (lane_config.json)
install(FILES lane_config.json DESTINATION share/${PROJECT_NAME}/config)

# ---------- Testing ----------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
