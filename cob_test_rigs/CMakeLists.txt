cmake_minimum_required(VERSION 2.8.3)
project(cob_test_rigs)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(PROGRAMS
  launch/rqt_plot.sh
  scripts/test_fdm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch robots
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

### TEST ###
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  # components
  set(component_list
    arm_pg70
    arm
    fdm
    head3
    pg70
    sensorring
    torso2
    torso3
  )
  foreach(component ${component_list})
    message("testing for component: ${component}")
    roslaunch_add_file_check(launch/${component}.launch)
  endforeach()

  # components
  set(component_list
    single_elmo
    single_schunk
  )
  foreach(component ${component_list})
    message("testing for component: ${component}")
    roslaunch_add_file_check(launch/${component}.launch can_device:=can0 can_id:=0)
  endforeach()
endif()
