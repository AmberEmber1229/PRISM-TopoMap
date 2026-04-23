cmake_minimum_required(VERSION 3.0.2)
project(prism_topomap)

# 使用 C++17 标准 (Eigen, structured bindings 等)
add_compile_options(-std=c++17 -O2)

###############################################################################
# 查找依赖
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
  pcl_conversions
  pcl_ros
)

# Eigen3
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common filters segmentation io)
add_definitions(${PCL_DEFINITIONS})

# OpenMP (required by FAISS)
find_package(OpenMP REQUIRED)

# FAISS — 通过 cmake、pkg-config 或 find_library 查找
find_package(faiss QUIET)
if(faiss_FOUND)
  # Modern FAISS cmake exports a target
  if(TARGET faiss::faiss)
    set(FAISS_LIBRARIES faiss::faiss)
  elseif(TARGET faiss)
    set(FAISS_LIBRARIES faiss)
  endif()
  message(STATUS "FAISS found via cmake: ${FAISS_LIBRARIES}")
else()
  # Try pkg-config
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(FAISS QUIET faiss)
  endif()
  if(NOT FAISS_FOUND)
    # Manual search using find_library
    find_library(FAISS_LIB
      NAMES faiss
      PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
      NO_DEFAULT_PATH
    )
    if(NOT FAISS_LIB)
      find_library(FAISS_LIB NAMES faiss)
    endif()
    if(FAISS_LIB)
      set(FAISS_LIBRARIES ${FAISS_LIB})
      set(FAISS_INCLUDE_DIRS "/usr/local/include")
      message(STATUS "FAISS found via find_library: ${FAISS_LIB}")
    else()
      message(FATAL_ERROR "FAISS library not found! Install via: apt install libfaiss-dev, or build from source.")
    endif()
  endif()
endif()

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# nlohmann_json — 用于图的 JSON 序列化
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  # 如果没有 cmake 模块, 假设头文件在标准路径中
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
# catkin 包配置
###############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES prism_topomap_lib
  CATKIN_DEPENDS
    roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs
    visualization_msgs tf cv_bridge message_runtime toposlam_msgs
    pcl_conversions pcl_ros
  DEPENDS EIGEN3 OpenCV PCL
)

###############################################################################
# 头文件路径
###############################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${FAISS_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
link_directories(${PCL_LIBRARY_DIRS})

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
target_link_libraries(prism_topomap_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${FAISS_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
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
target_link_libraries(prism_topomap_cpp_node
  prism_topomap_lib
  ${catkin_LIBRARIES}
  ${FAISS_LIBRARIES}
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
