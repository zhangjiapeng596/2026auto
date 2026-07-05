cmake_minimum_required(VERSION 2.8.3)
project(common)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/config_loader.py
  scripts/mission_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
