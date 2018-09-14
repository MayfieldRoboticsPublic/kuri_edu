cmake_minimum_required(VERSION 2.8.3)
project(kuri_edu)

find_package(catkin)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/simple_kuri
  scripts/safety_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)
endif()
