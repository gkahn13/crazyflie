# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "crazyflie: 3 messages, 0 services")

set(MSG_I_FLAGS "-Icrazyflie:/home/katiekang/catkin_ws/src/crazyflie/msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(crazyflie_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFMotion.msg" NAME_WE)
add_custom_target(_crazyflie_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "crazyflie" "/home/katiekang/catkin_ws/src/crazyflie/msg/CFMotion.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFCommand.msg" NAME_WE)
add_custom_target(_crazyflie_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "crazyflie" "/home/katiekang/catkin_ws/src/crazyflie/msg/CFCommand.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFData.msg" NAME_WE)
add_custom_target(_crazyflie_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "crazyflie" "/home/katiekang/catkin_ws/src/crazyflie/msg/CFData.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(crazyflie
  "/home/katiekang/catkin_ws/src/crazyflie/msg/CFMotion.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crazyflie
)
_generate_msg_cpp(crazyflie
  "/home/katiekang/catkin_ws/src/crazyflie/msg/CFCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crazyflie
)
_generate_msg_cpp(crazyflie
  "/home/katiekang/catkin_ws/src/crazyflie/msg/CFData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crazyflie
)

### Generating Services

### Generating Module File
_generate_module_cpp(crazyflie
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crazyflie
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(crazyflie_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(crazyflie_generate_messages crazyflie_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFMotion.msg" NAME_WE)
add_dependencies(crazyflie_generate_messages_cpp _crazyflie_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFCommand.msg" NAME_WE)
add_dependencies(crazyflie_generate_messages_cpp _crazyflie_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFData.msg" NAME_WE)
add_dependencies(crazyflie_generate_messages_cpp _crazyflie_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(crazyflie_gencpp)
add_dependencies(crazyflie_gencpp crazyflie_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS crazyflie_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(crazyflie
  "/home/katiekang/catkin_ws/src/crazyflie/msg/CFMotion.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crazyflie
)
_generate_msg_lisp(crazyflie
  "/home/katiekang/catkin_ws/src/crazyflie/msg/CFCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crazyflie
)
_generate_msg_lisp(crazyflie
  "/home/katiekang/catkin_ws/src/crazyflie/msg/CFData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crazyflie
)

### Generating Services

### Generating Module File
_generate_module_lisp(crazyflie
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crazyflie
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(crazyflie_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(crazyflie_generate_messages crazyflie_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFMotion.msg" NAME_WE)
add_dependencies(crazyflie_generate_messages_lisp _crazyflie_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFCommand.msg" NAME_WE)
add_dependencies(crazyflie_generate_messages_lisp _crazyflie_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFData.msg" NAME_WE)
add_dependencies(crazyflie_generate_messages_lisp _crazyflie_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(crazyflie_genlisp)
add_dependencies(crazyflie_genlisp crazyflie_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS crazyflie_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(crazyflie
  "/home/katiekang/catkin_ws/src/crazyflie/msg/CFMotion.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazyflie
)
_generate_msg_py(crazyflie
  "/home/katiekang/catkin_ws/src/crazyflie/msg/CFCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazyflie
)
_generate_msg_py(crazyflie
  "/home/katiekang/catkin_ws/src/crazyflie/msg/CFData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazyflie
)

### Generating Services

### Generating Module File
_generate_module_py(crazyflie
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazyflie
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(crazyflie_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(crazyflie_generate_messages crazyflie_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFMotion.msg" NAME_WE)
add_dependencies(crazyflie_generate_messages_py _crazyflie_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFCommand.msg" NAME_WE)
add_dependencies(crazyflie_generate_messages_py _crazyflie_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/katiekang/catkin_ws/src/crazyflie/msg/CFData.msg" NAME_WE)
add_dependencies(crazyflie_generate_messages_py _crazyflie_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(crazyflie_genpy)
add_dependencies(crazyflie_genpy crazyflie_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS crazyflie_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crazyflie)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/crazyflie
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(crazyflie_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(crazyflie_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crazyflie)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/crazyflie
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(crazyflie_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(crazyflie_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazyflie)
  install(CODE "execute_process(COMMAND \"/home/katiekang/py3/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazyflie\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazyflie
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazyflie
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/crazyflie/.+/__init__.pyc?$"
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(crazyflie_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(crazyflie_generate_messages_py std_msgs_generate_messages_py)
endif()
