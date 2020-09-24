cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME SampleCommunicationController)
set(PROJECT_DESCRIPTION "SampleCommunicationController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)

find_package(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc was compiled without ROS support")
endif()
find_package(3rd_party_ros REQUIRED COMPONENTS roscpp sensor_msgs)
if(NOT ${ROSCPP_FOUND})
  return()
endif()

add_subdirectory(src)
add_subdirectory(tools)

configure_file(etc/SampleCommunicationController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/SampleCommunicationController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/SampleCommunicationController.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

