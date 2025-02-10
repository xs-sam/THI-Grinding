cmake_minimum_required(VERSION 3.0.2)
project(test)
find_package(PCL 1.13 REQUIRED)
find_package(VTK 8.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  thi_vision
)


#编译包依赖另一个包的消息类型 加上包依赖
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)


#源码编译为
add_executable(test_node src/main.cpp)

#链接所有动态链接库
target_link_libraries(test_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
