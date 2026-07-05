cmake_minimum_required(VERSION 2.8.3)
project(mission_manager)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/mission_state_machine.py
  scripts/safety_monitor.py
  scripts/mock_vlm.py
  scripts/sim_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
