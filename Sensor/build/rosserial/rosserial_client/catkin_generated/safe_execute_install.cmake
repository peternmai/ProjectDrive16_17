execute_process(COMMAND "/home/ubuntu/ProjectDrive16_17/Sensor/build/rosserial/rosserial_client/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ubuntu/ProjectDrive16_17/Sensor/build/rosserial/rosserial_client/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
