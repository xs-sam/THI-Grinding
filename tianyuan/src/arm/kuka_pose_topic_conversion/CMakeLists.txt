cmake_minimum_required(VERSION 3.0.2)
project(kuka_pose_topic_conversion)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  kuka_tcp
  tf
)

#编译包依赖另一个包的消息类型 加上包依赖
catkin_package(CATKIN_DEPENDS kuka_tcp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#源码编译为
add_executable(kuka_pose_topic_conversion src/main.cpp)
add_executable(toolpose src/toolpose.cpp)

#添加其他依赖消息
add_dependencies(kuka_pose_topic_conversion ${catkin_EXPORTED_TARGETS})
add_dependencies(toolpose ${catkin_EXPORTED_TARGETS})

#链接所有动态链接库
target_link_libraries(kuka_pose_topic_conversion ${catkin_LIBRARIES})
target_link_libraries(toolpose ${catkin_LIBRARIES})
