# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "project_isds: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iproject_isds:/home/kimsngi/project_ws/src/project_isds/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(project_isds_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_project_isds_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "project_isds" "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv" ""
)

get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/msg/student.msg" NAME_WE)
add_custom_target(_project_isds_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "project_isds" "/home/kimsngi/project_ws/src/project_isds/msg/student.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(project_isds
  "/home/kimsngi/project_ws/src/project_isds/msg/student.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/project_isds
)

### Generating Services
_generate_srv_cpp(project_isds
  "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/project_isds
)

### Generating Module File
_generate_module_cpp(project_isds
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/project_isds
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(project_isds_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(project_isds_generate_messages project_isds_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(project_isds_generate_messages_cpp _project_isds_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/msg/student.msg" NAME_WE)
add_dependencies(project_isds_generate_messages_cpp _project_isds_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(project_isds_gencpp)
add_dependencies(project_isds_gencpp project_isds_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS project_isds_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(project_isds
  "/home/kimsngi/project_ws/src/project_isds/msg/student.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/project_isds
)

### Generating Services
_generate_srv_eus(project_isds
  "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/project_isds
)

### Generating Module File
_generate_module_eus(project_isds
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/project_isds
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(project_isds_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(project_isds_generate_messages project_isds_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(project_isds_generate_messages_eus _project_isds_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/msg/student.msg" NAME_WE)
add_dependencies(project_isds_generate_messages_eus _project_isds_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(project_isds_geneus)
add_dependencies(project_isds_geneus project_isds_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS project_isds_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(project_isds
  "/home/kimsngi/project_ws/src/project_isds/msg/student.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/project_isds
)

### Generating Services
_generate_srv_lisp(project_isds
  "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/project_isds
)

### Generating Module File
_generate_module_lisp(project_isds
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/project_isds
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(project_isds_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(project_isds_generate_messages project_isds_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(project_isds_generate_messages_lisp _project_isds_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/msg/student.msg" NAME_WE)
add_dependencies(project_isds_generate_messages_lisp _project_isds_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(project_isds_genlisp)
add_dependencies(project_isds_genlisp project_isds_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS project_isds_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(project_isds
  "/home/kimsngi/project_ws/src/project_isds/msg/student.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/project_isds
)

### Generating Services
_generate_srv_nodejs(project_isds
  "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/project_isds
)

### Generating Module File
_generate_module_nodejs(project_isds
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/project_isds
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(project_isds_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(project_isds_generate_messages project_isds_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(project_isds_generate_messages_nodejs _project_isds_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/msg/student.msg" NAME_WE)
add_dependencies(project_isds_generate_messages_nodejs _project_isds_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(project_isds_gennodejs)
add_dependencies(project_isds_gennodejs project_isds_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS project_isds_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(project_isds
  "/home/kimsngi/project_ws/src/project_isds/msg/student.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/project_isds
)

### Generating Services
_generate_srv_py(project_isds
  "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/project_isds
)

### Generating Module File
_generate_module_py(project_isds
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/project_isds
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(project_isds_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(project_isds_generate_messages project_isds_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(project_isds_generate_messages_py _project_isds_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kimsngi/project_ws/src/project_isds/msg/student.msg" NAME_WE)
add_dependencies(project_isds_generate_messages_py _project_isds_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(project_isds_genpy)
add_dependencies(project_isds_genpy project_isds_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS project_isds_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/project_isds)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/project_isds
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(project_isds_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/project_isds)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/project_isds
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(project_isds_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/project_isds)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/project_isds
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(project_isds_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/project_isds)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/project_isds
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(project_isds_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/project_isds)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/project_isds\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/project_isds
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(project_isds_generate_messages_py std_msgs_generate_messages_py)
endif()