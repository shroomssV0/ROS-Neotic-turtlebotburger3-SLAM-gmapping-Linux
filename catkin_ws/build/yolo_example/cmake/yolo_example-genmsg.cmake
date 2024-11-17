# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "yolo_example: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iyolo_example:/home/ros_ws/catkin_ws/src/yolo_example/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(yolo_example_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg" NAME_WE)
add_custom_target(_yolo_example_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "yolo_example" "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg" "yolo_example/BoundingBox"
)

get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg" NAME_WE)
add_custom_target(_yolo_example_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "yolo_example" "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(yolo_example
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yolo_example
)
_generate_msg_cpp(yolo_example
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yolo_example
)

### Generating Services

### Generating Module File
_generate_module_cpp(yolo_example
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yolo_example
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(yolo_example_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(yolo_example_generate_messages yolo_example_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg" NAME_WE)
add_dependencies(yolo_example_generate_messages_cpp _yolo_example_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg" NAME_WE)
add_dependencies(yolo_example_generate_messages_cpp _yolo_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yolo_example_gencpp)
add_dependencies(yolo_example_gencpp yolo_example_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yolo_example_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(yolo_example
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yolo_example
)
_generate_msg_eus(yolo_example
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yolo_example
)

### Generating Services

### Generating Module File
_generate_module_eus(yolo_example
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yolo_example
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(yolo_example_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(yolo_example_generate_messages yolo_example_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg" NAME_WE)
add_dependencies(yolo_example_generate_messages_eus _yolo_example_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg" NAME_WE)
add_dependencies(yolo_example_generate_messages_eus _yolo_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yolo_example_geneus)
add_dependencies(yolo_example_geneus yolo_example_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yolo_example_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(yolo_example
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yolo_example
)
_generate_msg_lisp(yolo_example
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yolo_example
)

### Generating Services

### Generating Module File
_generate_module_lisp(yolo_example
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yolo_example
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(yolo_example_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(yolo_example_generate_messages yolo_example_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg" NAME_WE)
add_dependencies(yolo_example_generate_messages_lisp _yolo_example_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg" NAME_WE)
add_dependencies(yolo_example_generate_messages_lisp _yolo_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yolo_example_genlisp)
add_dependencies(yolo_example_genlisp yolo_example_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yolo_example_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(yolo_example
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yolo_example
)
_generate_msg_nodejs(yolo_example
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yolo_example
)

### Generating Services

### Generating Module File
_generate_module_nodejs(yolo_example
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yolo_example
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(yolo_example_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(yolo_example_generate_messages yolo_example_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg" NAME_WE)
add_dependencies(yolo_example_generate_messages_nodejs _yolo_example_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg" NAME_WE)
add_dependencies(yolo_example_generate_messages_nodejs _yolo_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yolo_example_gennodejs)
add_dependencies(yolo_example_gennodejs yolo_example_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yolo_example_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(yolo_example
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_example
)
_generate_msg_py(yolo_example
  "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_example
)

### Generating Services

### Generating Module File
_generate_module_py(yolo_example
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_example
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(yolo_example_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(yolo_example_generate_messages yolo_example_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/DetectedObject.msg" NAME_WE)
add_dependencies(yolo_example_generate_messages_py _yolo_example_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros_ws/catkin_ws/src/yolo_example/msg/BoundingBox.msg" NAME_WE)
add_dependencies(yolo_example_generate_messages_py _yolo_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yolo_example_genpy)
add_dependencies(yolo_example_genpy yolo_example_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yolo_example_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yolo_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yolo_example
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(yolo_example_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yolo_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yolo_example
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(yolo_example_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yolo_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yolo_example
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(yolo_example_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yolo_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yolo_example
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(yolo_example_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_example)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_example\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_example
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(yolo_example_generate_messages_py std_msgs_generate_messages_py)
endif()
