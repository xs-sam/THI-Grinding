cmake_minimum_required(VERSION 3.0.2)
project(kuka_create_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)


#编译包依赖另一个包的消息类型 加上包依赖
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#源码编译为
add_executable(kuka_create_tf src/main.cpp)

#链接所有动态链接库
target_link_libraries(kuka_create_tf ${catkin_LIBRARIES})
