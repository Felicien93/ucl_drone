execute_process(COMMAND "/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/build/ucl_drone_gui/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/build/ucl_drone_gui/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
