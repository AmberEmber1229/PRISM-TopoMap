cmake_minimum_required(VERSION 3.0.2)
project(prism_topomap)

# 使用 C++17 标准 (Eigen, structured bindings 等)
add_compile_options(-std=c++17 -O2)

###############################################################################
# 查找依赖 (新增了 roslib)
###############################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
  cv_bridge
  message_generation
  toposlam_msgs
  roslib  # <--- 解决 ros::package::getPath 报错
)

# Eigen3
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    # 把 OpenMP 的编译参数加到全局
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
# FAISS — 因为我们在系统级安装了，直接用 REQUIRED 强制查找，最稳妥
find_package(faiss REQUIRED)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# nlohmann_json — 用于图的 JSON 序列化
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  message(STATUS "nlohmann_json not found via cmake, assuming header-only in system include path")
endif()

###############################################################################
# 自定义 Service
###############################################################################
add_service_files(
  FILES
  GetDescriptor.srv
  GridRegistration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###############################################################################
# catkin 包配置 (新增了 roslib)
###############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES prism_topomap_lib
  CATKIN_DEPENDS
    roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs
    visualization_msgs tf cv_bridge message_runtime toposlam_msgs roslib
  DEPENDS EIGEN3 OpenCV
)

###############################################################################
# 头文件路径
###############################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

###############################################################################
# 构建静态库 (所有核心模块)
###############################################################################
add_library(prism_topomap_lib
  src/utils.cpp
  src/local_grid.cpp
  src/inference_client.cpp
  src/topological_graph.cpp
  src/localizer.cpp
  src/topo_slam_model.cpp
  src/results_publisher.cpp
)
add_dependencies(prism_topomap_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
# 强制把 faiss 链接进你的底层库
target_link_libraries(prism_topomap_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  faiss 
  Eigen3::Eigen
)

###############################################################################
# 构建可执行文件 (C++ 主节点)
###############################################################################
add_executable(prism_topomap_cpp_node
  src/prism_topomap_node.cpp
)
add_dependencies(prism_topomap_cpp_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
# 强制把 faiss 也链接进你的节点程序
target_link_libraries(prism_topomap_cpp_node
  prism_topomap_lib
  ${catkin_LIBRARIES}
  faiss 
)

###############################################################################
# 安装
###############################################################################
install(TARGETS prism_topomap_lib prism_topomap_cpp_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Python 脚本安装
catkin_install_python(PROGRAMS
  scripts/inference_service_node.py
  scripts/tf_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)