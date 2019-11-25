# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lidar_camera_calibration: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ilidar_camera_calibration:/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lidar_camera_calibration_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg" NAME_WE)
add_custom_target(_lidar_camera_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_camera_calibration" "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg" "std_msgs/Float32MultiArray:std_msgs/MultiArrayDimension:std_msgs/Header:std_msgs/MultiArrayLayout"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lidar_camera_calibration
  "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_camera_calibration
)

### Generating Services

### Generating Module File
_generate_module_cpp(lidar_camera_calibration
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_camera_calibration
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lidar_camera_calibration_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lidar_camera_calibration_generate_messages lidar_camera_calibration_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg" NAME_WE)
add_dependencies(lidar_camera_calibration_generate_messages_cpp _lidar_camera_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_camera_calibration_gencpp)
add_dependencies(lidar_camera_calibration_gencpp lidar_camera_calibration_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_camera_calibration_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(lidar_camera_calibration
  "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_camera_calibration
)

### Generating Services

### Generating Module File
_generate_module_eus(lidar_camera_calibration
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_camera_calibration
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(lidar_camera_calibration_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(lidar_camera_calibration_generate_messages lidar_camera_calibration_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg" NAME_WE)
add_dependencies(lidar_camera_calibration_generate_messages_eus _lidar_camera_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_camera_calibration_geneus)
add_dependencies(lidar_camera_calibration_geneus lidar_camera_calibration_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_camera_calibration_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lidar_camera_calibration
  "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_camera_calibration
)

### Generating Services

### Generating Module File
_generate_module_lisp(lidar_camera_calibration
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_camera_calibration
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lidar_camera_calibration_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lidar_camera_calibration_generate_messages lidar_camera_calibration_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg" NAME_WE)
add_dependencies(lidar_camera_calibration_generate_messages_lisp _lidar_camera_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_camera_calibration_genlisp)
add_dependencies(lidar_camera_calibration_genlisp lidar_camera_calibration_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_camera_calibration_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(lidar_camera_calibration
  "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_camera_calibration
)

### Generating Services

### Generating Module File
_generate_module_nodejs(lidar_camera_calibration
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_camera_calibration
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(lidar_camera_calibration_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(lidar_camera_calibration_generate_messages lidar_camera_calibration_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg" NAME_WE)
add_dependencies(lidar_camera_calibration_generate_messages_nodejs _lidar_camera_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_camera_calibration_gennodejs)
add_dependencies(lidar_camera_calibration_gennodejs lidar_camera_calibration_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_camera_calibration_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lidar_camera_calibration
  "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_camera_calibration
)

### Generating Services

### Generating Module File
_generate_module_py(lidar_camera_calibration
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_camera_calibration
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lidar_camera_calibration_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lidar_camera_calibration_generate_messages lidar_camera_calibration_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/taolihao/lidar_camera_tlhLine/src/lidar_camera_calibration-master/msg/marker_6dof.msg" NAME_WE)
add_dependencies(lidar_camera_calibration_generate_messages_py _lidar_camera_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_camera_calibration_genpy)
add_dependencies(lidar_camera_calibration_genpy lidar_camera_calibration_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_camera_calibration_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_camera_calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_camera_calibration
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(lidar_camera_calibration_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_camera_calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_camera_calibration
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(lidar_camera_calibration_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_camera_calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_camera_calibration
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(lidar_camera_calibration_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_camera_calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_camera_calibration
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(lidar_camera_calibration_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_camera_calibration)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_camera_calibration\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_camera_calibration
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(lidar_camera_calibration_generate_messages_py std_msgs_generate_messages_py)
endif()
