cmake_minimum_required(VERSION 3.0.2)
project(node_manager)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roslaunch
  rospy
  std_msgs
)

#添加srv自定义服务文件
add_service_files(
  FILES
  nodeInfo.srv
 )

#添加依赖的消息类型
generate_messages(
  DEPENDENCIES
  std_msgs
 )

#声明联合编译为ros包 不然roslaunch和rosrun调不起
catkin_package(CATKIN_DEPENDS
    std_msgs
    message_runtime
    )
    
#添加运行脚本  
 catkin_install_python(PROGRAMS
   scripts/node_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
    
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


