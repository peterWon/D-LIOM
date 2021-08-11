cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

project(dlio)

set(PACKAGE_DEPENDENCIES
  cartographer_ros
  roscpp
  roslib
)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
google_initialize_cartographer_project()
google_enable_testing()

find_package(LuaGoogle REQUIRED)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

# Override Catkin's GTest configuration to use GMock.
set(GTEST_FOUND TRUE)
set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
set(GTEST_LIBRARIES ${GMOCK_LIBRARIES})

catkin_package(CATKIN_DEPENDS message_runtime ${PACKAGE_DEPENDENCIES})

install(DIRECTORY launch configuration_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)