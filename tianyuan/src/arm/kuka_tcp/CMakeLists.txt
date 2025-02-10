cmake_minimum_required(VERSION 3.0.2)
project(kuka_tcp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

#添加msg自定义消息文件
add_message_files(
    FILES
    kukaPoint.msg
 )

#添加srv自定义服务文件
add_service_files(
  FILES
  kukaTrack.srv
 )

#添加依赖的消息类型
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
 )

catkin_package(CATKIN_DEPENDS  std_msgs geometry_msgs message_runtime)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#源码编译为
add_executable(kuka_tcp src/main.cpp src/xmltcp.cpp src/tinyxml2.cpp src/kukatcp.cpp)

#优先编译srv和msg文件
add_dependencies(kuka_tcp ${PROJECT_NAME}_generate_messages_cpp)

#链接所有动态链接库
target_link_libraries(kuka_tcp ${catkin_LIBRARIES} pthread)
