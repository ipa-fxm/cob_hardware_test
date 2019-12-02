cmake_minimum_required(VERSION 2.8.3)
project(cob_devices_test)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(PROGRAMS
  scripts/test_light.py
  scripts/udev_joy.sh
  scripts/udev_s300.sh
  scripts/plot_thermal_test.py
  scripts/run_thermal_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

### TEST ###
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(launch/base.launch flexisoft_host:=test)
  roslaunch_add_file_check(launch/head.launch)
  roslaunch_add_file_check(launch/torso.launch)
endif()
