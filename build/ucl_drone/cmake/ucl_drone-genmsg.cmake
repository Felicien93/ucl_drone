# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ucl_drone: 9 messages, 4 services")

set(MSG_I_FLAGS "-Iucl_drone:/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Iardrone_autonomy:/home/felicien/Desktop/UCL_drones/shared-ros-workspace/src/ardrone_autonomy/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ucl_drone_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/Projection.srv" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/Projection.srv" ""
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRole.msg" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRole.msg" ""
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/PoseRef.msg" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/PoseRef.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/SetPoseRef.srv" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/SetPoseRef.srv" ""
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/cellUpdate.msg" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/cellUpdate.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRoles.msg" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRoles.msg" "ucl_drone/DroneRole"
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/KeyPoint.msg" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/KeyPoint.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/MapChunk.srv" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/MapChunk.srv" ""
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/ProcessedImageMsg.msg" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/ProcessedImageMsg.msg" "sensor_msgs/Image:ucl_drone/Pose3D:std_msgs/Header:ucl_drone/KeyPoint:geometry_msgs/Point"
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/StrategyMsg.msg" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/StrategyMsg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/FeaturesArray.srv" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/FeaturesArray.srv" ""
)

get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/TargetDetected.msg" NAME_WE)
add_custom_target(_ucl_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucl_drone" "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/TargetDetected.msg" "ardrone_autonomy/Navdata:ucl_drone/Pose3D:std_msgs/Header:geometry_msgs/Point"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRole.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)
_generate_msg_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/PoseRef.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)
_generate_msg_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/cellUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)
_generate_msg_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRoles.msg"
  "${MSG_I_FLAGS}"
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRole.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)
_generate_msg_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)
_generate_msg_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)
_generate_msg_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/ProcessedImageMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg;/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/KeyPoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)
_generate_msg_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/StrategyMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)
_generate_msg_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/TargetDetected.msg"
  "${MSG_I_FLAGS}"
  "/home/felicien/Desktop/UCL_drones/shared-ros-workspace/src/ardrone_autonomy/msg/Navdata.msg;/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)

### Generating Services
_generate_srv_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/SetPoseRef.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)
_generate_srv_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/Projection.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)
_generate_srv_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/MapChunk.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)
_generate_srv_cpp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/FeaturesArray.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
)

### Generating Module File
_generate_module_cpp(ucl_drone
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ucl_drone_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ucl_drone_generate_messages ucl_drone_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/Projection.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRole.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/PoseRef.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/SetPoseRef.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/cellUpdate.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRoles.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/KeyPoint.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/MapChunk.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/ProcessedImageMsg.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/StrategyMsg.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/FeaturesArray.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/TargetDetected.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_cpp _ucl_drone_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucl_drone_gencpp)
add_dependencies(ucl_drone_gencpp ucl_drone_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucl_drone_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRole.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)
_generate_msg_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/PoseRef.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)
_generate_msg_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/cellUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)
_generate_msg_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRoles.msg"
  "${MSG_I_FLAGS}"
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRole.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)
_generate_msg_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)
_generate_msg_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)
_generate_msg_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/ProcessedImageMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg;/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/KeyPoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)
_generate_msg_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/StrategyMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)
_generate_msg_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/TargetDetected.msg"
  "${MSG_I_FLAGS}"
  "/home/felicien/Desktop/UCL_drones/shared-ros-workspace/src/ardrone_autonomy/msg/Navdata.msg;/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)

### Generating Services
_generate_srv_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/SetPoseRef.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)
_generate_srv_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/Projection.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)
_generate_srv_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/MapChunk.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)
_generate_srv_lisp(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/FeaturesArray.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
)

### Generating Module File
_generate_module_lisp(ucl_drone
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ucl_drone_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ucl_drone_generate_messages ucl_drone_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/Projection.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRole.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/PoseRef.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/SetPoseRef.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/cellUpdate.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRoles.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/KeyPoint.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/MapChunk.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/ProcessedImageMsg.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/StrategyMsg.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/FeaturesArray.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/TargetDetected.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_lisp _ucl_drone_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucl_drone_genlisp)
add_dependencies(ucl_drone_genlisp ucl_drone_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucl_drone_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRole.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)
_generate_msg_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/PoseRef.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)
_generate_msg_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/cellUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)
_generate_msg_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRoles.msg"
  "${MSG_I_FLAGS}"
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRole.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)
_generate_msg_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)
_generate_msg_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)
_generate_msg_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/ProcessedImageMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg;/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/KeyPoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)
_generate_msg_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/StrategyMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)
_generate_msg_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/TargetDetected.msg"
  "${MSG_I_FLAGS}"
  "/home/felicien/Desktop/UCL_drones/shared-ros-workspace/src/ardrone_autonomy/msg/Navdata.msg;/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)

### Generating Services
_generate_srv_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/SetPoseRef.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)
_generate_srv_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/Projection.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)
_generate_srv_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/MapChunk.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)
_generate_srv_py(ucl_drone
  "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/FeaturesArray.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
)

### Generating Module File
_generate_module_py(ucl_drone
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ucl_drone_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ucl_drone_generate_messages ucl_drone_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/Projection.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRole.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/PoseRef.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/SetPoseRef.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/cellUpdate.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/DroneRoles.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/KeyPoint.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/MapChunk.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/Pose3D.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/ProcessedImageMsg.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/StrategyMsg.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/srv/FeaturesArray.srv" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felicien/Desktop/origin2/ucl_drone/src/ucl_drone/msg/TargetDetected.msg" NAME_WE)
add_dependencies(ucl_drone_generate_messages_py _ucl_drone_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucl_drone_genpy)
add_dependencies(ucl_drone_genpy ucl_drone_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucl_drone_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucl_drone
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(ucl_drone_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(ucl_drone_generate_messages_cpp geometry_msgs_generate_messages_cpp)
add_dependencies(ucl_drone_generate_messages_cpp sensor_msgs_generate_messages_cpp)
add_dependencies(ucl_drone_generate_messages_cpp ardrone_autonomy_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucl_drone
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(ucl_drone_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(ucl_drone_generate_messages_lisp geometry_msgs_generate_messages_lisp)
add_dependencies(ucl_drone_generate_messages_lisp sensor_msgs_generate_messages_lisp)
add_dependencies(ucl_drone_generate_messages_lisp ardrone_autonomy_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucl_drone
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(ucl_drone_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(ucl_drone_generate_messages_py geometry_msgs_generate_messages_py)
add_dependencies(ucl_drone_generate_messages_py sensor_msgs_generate_messages_py)
add_dependencies(ucl_drone_generate_messages_py ardrone_autonomy_generate_messages_py)
