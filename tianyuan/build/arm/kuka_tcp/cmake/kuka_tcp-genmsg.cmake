# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "kuka_tcp: 1 messages, 1 services")

set(MSG_I_FLAGS "-Ikuka_tcp:/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(kuka_tcp_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg" NAME_WE)
add_custom_target(_kuka_tcp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kuka_tcp" "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg" ""
)

get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv" NAME_WE)
add_custom_target(_kuka_tcp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kuka_tcp" "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv" "kuka_tcp/kukaPoint"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(kuka_tcp
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kuka_tcp
)

### Generating Services
_generate_srv_cpp(kuka_tcp
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kuka_tcp
)

### Generating Module File
_generate_module_cpp(kuka_tcp
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kuka_tcp
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(kuka_tcp_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(kuka_tcp_generate_messages kuka_tcp_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg" NAME_WE)
add_dependencies(kuka_tcp_generate_messages_cpp _kuka_tcp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv" NAME_WE)
add_dependencies(kuka_tcp_generate_messages_cpp _kuka_tcp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kuka_tcp_gencpp)
add_dependencies(kuka_tcp_gencpp kuka_tcp_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kuka_tcp_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(kuka_tcp
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kuka_tcp
)

### Generating Services
_generate_srv_eus(kuka_tcp
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kuka_tcp
)

### Generating Module File
_generate_module_eus(kuka_tcp
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kuka_tcp
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(kuka_tcp_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(kuka_tcp_generate_messages kuka_tcp_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg" NAME_WE)
add_dependencies(kuka_tcp_generate_messages_eus _kuka_tcp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv" NAME_WE)
add_dependencies(kuka_tcp_generate_messages_eus _kuka_tcp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kuka_tcp_geneus)
add_dependencies(kuka_tcp_geneus kuka_tcp_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kuka_tcp_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(kuka_tcp
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kuka_tcp
)

### Generating Services
_generate_srv_lisp(kuka_tcp
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kuka_tcp
)

### Generating Module File
_generate_module_lisp(kuka_tcp
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kuka_tcp
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(kuka_tcp_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(kuka_tcp_generate_messages kuka_tcp_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg" NAME_WE)
add_dependencies(kuka_tcp_generate_messages_lisp _kuka_tcp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv" NAME_WE)
add_dependencies(kuka_tcp_generate_messages_lisp _kuka_tcp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kuka_tcp_genlisp)
add_dependencies(kuka_tcp_genlisp kuka_tcp_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kuka_tcp_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(kuka_tcp
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kuka_tcp
)

### Generating Services
_generate_srv_nodejs(kuka_tcp
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kuka_tcp
)

### Generating Module File
_generate_module_nodejs(kuka_tcp
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kuka_tcp
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(kuka_tcp_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(kuka_tcp_generate_messages kuka_tcp_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg" NAME_WE)
add_dependencies(kuka_tcp_generate_messages_nodejs _kuka_tcp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv" NAME_WE)
add_dependencies(kuka_tcp_generate_messages_nodejs _kuka_tcp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kuka_tcp_gennodejs)
add_dependencies(kuka_tcp_gennodejs kuka_tcp_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kuka_tcp_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(kuka_tcp
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_tcp
)

### Generating Services
_generate_srv_py(kuka_tcp
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_tcp
)

### Generating Module File
_generate_module_py(kuka_tcp
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_tcp
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(kuka_tcp_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(kuka_tcp_generate_messages kuka_tcp_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/msg/kukaPoint.msg" NAME_WE)
add_dependencies(kuka_tcp_generate_messages_py _kuka_tcp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/arm/kuka_tcp/srv/kukaTrack.srv" NAME_WE)
add_dependencies(kuka_tcp_generate_messages_py _kuka_tcp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kuka_tcp_genpy)
add_dependencies(kuka_tcp_genpy kuka_tcp_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kuka_tcp_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kuka_tcp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kuka_tcp
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(kuka_tcp_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(kuka_tcp_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kuka_tcp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kuka_tcp
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(kuka_tcp_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(kuka_tcp_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kuka_tcp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kuka_tcp
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(kuka_tcp_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(kuka_tcp_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kuka_tcp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kuka_tcp
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(kuka_tcp_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(kuka_tcp_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_tcp)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_tcp\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_tcp
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(kuka_tcp_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(kuka_tcp_generate_messages_py geometry_msgs_generate_messages_py)
endif()
