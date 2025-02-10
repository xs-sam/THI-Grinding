cmake_minimum_required(VERSION 3.0.2)
project(thi_vision)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  tf
)

find_package(PCL 1.13 REQUIRED)
find_package(VTK 8.2 REQUIRED)

#添加msg自定义消息文件
add_message_files(
    FILES
    position.msg
    orientation.msg
    pose.msg
    track.msg
    pcArea.msg
    )

#添加srv自定义服务文件
add_service_files(
  FILES
  visionTracks.srv
 )

#添加依赖的消息类型
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
 )

#声明联合编译为ros包 不然roslaunch和rosrun调不起
catkin_package(CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    message_runtime
    )

 #########################################  源码、ui文件和头文件  ################################################
 set(SOURCES
     src/main.cpp
     src/thivision.cpp
 )

 set(HEADERS
     include/thivision.h
 )

 #加载头文件到环境中
 include_directories(
     include
     ${catkin_INCLUDE_DIRS}
     ${PCL_INCLUDE_DIRS}
     ${VTK_INCLUDE_DIRS}
 )

 #源码编译为
 add_executable(thi_vision ${SOURCES} ${HEADERS}
 )
 #优先编译srv和msg文件
 add_dependencies(thi_vision ${PROJECT_NAME}_generate_messages_cpp)
 #链接所有动态链接库
 target_link_libraries(thi_vision  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
