cmake_minimum_required(VERSION 3.0.2)
project(lidar_clustering)

# 1. 设置 C++ 标准为 14 (PCL 1.8 建议 C++11 或 C++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. 开启 PCL_NO_PRECOMPILE 宏，防止 PointXYZI 报链接错误
add_definitions(-DPCL_NO_PRECOMPILE)

# 3. 指定寻找 PCL 1.8
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters segmentation sample_consensus kdtree search)

# 包含 PCL 的头文件和库
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
)

find_package(PCL REQUIRED)

# 寻找 yaml-cpp 库
find_package(yaml-cpp REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(cluster_node src/cluster_node.cpp)

# 链接 catkin, PCL 和 yaml-cpp
target_link_libraries(cluster_node 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  yaml-cpp
)

install(TARGETS cluster_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ========================================================
# 安装 launch 和 params 文件夹到 share 目录
# ========================================================
install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)