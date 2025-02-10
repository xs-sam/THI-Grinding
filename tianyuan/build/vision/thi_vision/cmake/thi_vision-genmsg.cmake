# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "thi_vision: 5 messages, 1 services")

set(MSG_I_FLAGS "-Ithi_vision:/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(thi_vision_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg" NAME_WE)
add_custom_target(_thi_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "thi_vision" "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg" ""
)

get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg" NAME_WE)
add_custom_target(_thi_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "thi_vision" "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg" ""
)

get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg" NAME_WE)
add_custom_target(_thi_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "thi_vision" "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg" "thi_vision/orientation:thi_vision/position"
)

get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg" NAME_WE)
add_custom_target(_thi_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "thi_vision" "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg" "thi_vision/orientation:thi_vision/position:thi_vision/pose"
)

get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg" NAME_WE)
add_custom_target(_thi_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "thi_vision" "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg" "thi_vision/position"
)

get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv" NAME_WE)
add_custom_target(_thi_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "thi_vision" "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv" "thi_vision/position:thi_vision/orientation:thi_vision/track:thi_vision/pose:thi_vision/pcArea"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thi_vision
)
_generate_msg_cpp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thi_vision
)
_generate_msg_cpp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thi_vision
)
_generate_msg_cpp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thi_vision
)
_generate_msg_cpp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thi_vision
)

### Generating Services
_generate_srv_cpp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thi_vision
)

### Generating Module File
_generate_module_cpp(thi_vision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thi_vision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(thi_vision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(thi_vision_generate_messages thi_vision_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_cpp _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_cpp _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_cpp _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_cpp _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_cpp _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv" NAME_WE)
add_dependencies(thi_vision_generate_messages_cpp _thi_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thi_vision_gencpp)
add_dependencies(thi_vision_gencpp thi_vision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thi_vision_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thi_vision
)
_generate_msg_eus(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thi_vision
)
_generate_msg_eus(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thi_vision
)
_generate_msg_eus(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thi_vision
)
_generate_msg_eus(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thi_vision
)

### Generating Services
_generate_srv_eus(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thi_vision
)

### Generating Module File
_generate_module_eus(thi_vision
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thi_vision
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(thi_vision_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(thi_vision_generate_messages thi_vision_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_eus _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_eus _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_eus _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_eus _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_eus _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv" NAME_WE)
add_dependencies(thi_vision_generate_messages_eus _thi_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thi_vision_geneus)
add_dependencies(thi_vision_geneus thi_vision_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thi_vision_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thi_vision
)
_generate_msg_lisp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thi_vision
)
_generate_msg_lisp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thi_vision
)
_generate_msg_lisp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thi_vision
)
_generate_msg_lisp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thi_vision
)

### Generating Services
_generate_srv_lisp(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thi_vision
)

### Generating Module File
_generate_module_lisp(thi_vision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thi_vision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(thi_vision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(thi_vision_generate_messages thi_vision_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_lisp _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_lisp _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_lisp _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_lisp _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_lisp _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv" NAME_WE)
add_dependencies(thi_vision_generate_messages_lisp _thi_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thi_vision_genlisp)
add_dependencies(thi_vision_genlisp thi_vision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thi_vision_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thi_vision
)
_generate_msg_nodejs(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thi_vision
)
_generate_msg_nodejs(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thi_vision
)
_generate_msg_nodejs(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thi_vision
)
_generate_msg_nodejs(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thi_vision
)

### Generating Services
_generate_srv_nodejs(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thi_vision
)

### Generating Module File
_generate_module_nodejs(thi_vision
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thi_vision
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(thi_vision_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(thi_vision_generate_messages thi_vision_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_nodejs _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_nodejs _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_nodejs _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_nodejs _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_nodejs _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv" NAME_WE)
add_dependencies(thi_vision_generate_messages_nodejs _thi_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thi_vision_gennodejs)
add_dependencies(thi_vision_gennodejs thi_vision_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thi_vision_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thi_vision
)
_generate_msg_py(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thi_vision
)
_generate_msg_py(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thi_vision
)
_generate_msg_py(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thi_vision
)
_generate_msg_py(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thi_vision
)

### Generating Services
_generate_srv_py(thi_vision
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv"
  "${MSG_I_FLAGS}"
  "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg;/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thi_vision
)

### Generating Module File
_generate_module_py(thi_vision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thi_vision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(thi_vision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(thi_vision_generate_messages thi_vision_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/position.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_py _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/orientation.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_py _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pose.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_py _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/track.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_py _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/msg/pcArea.msg" NAME_WE)
add_dependencies(thi_vision_generate_messages_py _thi_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xs/sifc/project/ros/workProject/tianyuan/src/vision/thi_vision/srv/visionTracks.srv" NAME_WE)
add_dependencies(thi_vision_generate_messages_py _thi_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thi_vision_genpy)
add_dependencies(thi_vision_genpy thi_vision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thi_vision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thi_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thi_vision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(thi_vision_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(thi_vision_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thi_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/thi_vision
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(thi_vision_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(thi_vision_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thi_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thi_vision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(thi_vision_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(thi_vision_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thi_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/thi_vision
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(thi_vision_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(thi_vision_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thi_vision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thi_vision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thi_vision
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(thi_vision_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(thi_vision_generate_messages_py geometry_msgs_generate_messages_py)
endif()
