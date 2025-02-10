cmake_minimum_required(VERSION 3.0.2)
project(tcp_test)

find_package(catkin REQUIRED COMPONENTS
  kuka_tcp
  roscpp
)


#编译包依赖另一个包的消息类型 加上包依赖
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#源码编译为
add_executable(tcp_test src/main.cpp)

#链接所有动态链接库
target_link_libraries(tcp_test ${catkin_LIBRARIES})
