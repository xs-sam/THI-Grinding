cmake_minimum_required(VERSION 3.0.2)
project(pc_merge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

#寻找PCL 和 VTK
find_package(PCL 1.13 REQUIRED)
find_package(VTK 8.2 REQUIRED)

#添加srv自定义服务文件
add_service_files(
    FILES
    mergePc.srv
    )
#添加依赖的消息类型
generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
    sensor_msgs
    )

#声明联合编译为ros包 不然roslaunch和rosrun调不起
catkin_package(CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    message_runtime
    )

set(SOURCES
    src/main.cpp
)

#加载头文件到环境中
include_directories(
    /usr/include/eigen3
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    )

link_directories(
)

#源码编译为
add_executable(pc_merge ${SOURCES})
#优先编译srv和msg文件
add_dependencies(pc_merge ${PROJECT_NAME}_generate_messages_cpp)
#链接所有动态链接库
target_link_libraries(pc_merge ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
