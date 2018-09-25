cmake_minimum_required(VERSION 2.8.3)
project(kuri_edu)

find_package(catkin)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/cap_touch_chest_light
  scripts/chest_light_controller
  scripts/head_controller
  scripts/safety_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_nosetests(test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)

  add_rostest(rostest/test_cap_touch_chest_light.launch)
  add_rostest(rostest/test_chest_light_controller.launch)
  add_rostest(rostest/test_head_controller.launch)
  add_rostest(rostest/test_safety_controller.launch)
endif()
