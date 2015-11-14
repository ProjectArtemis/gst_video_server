cmake_minimum_required(VERSION 2.8.3)
project(gst_video_server)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nodelet
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

## GStreamer 1.0 package required
find_package(PkgConfig)

macro(find_gstreamer_component prefix pkg lib)
  pkg_check_modules(PC_${prefix} REQUIRED ${pkg})
  find_library(${prefix}_LIBRARIES NAMES ${lib} HINTS ${PC_${prefix}_LIBRARY_DIRS} ${PC_${prefix}_LIBDIR})
  list(APPEND gstreamer_INCLUDE_DIRS ${PC_${prefix}_INCLUDE_DIRS})
  list(APPEND gstreamer_LIBRARIES ${${prefix}_LIBRARIES})
  mark_as_advanced(${prefix}_LIBRARIES gstreamer_LIBRARIES gstreamer_INCLUDE_DIRS)
endmacro()

find_gstreamer_component(gst      gstreamer-1.0       gstreamer-1.0)
find_gstreamer_component(gstbase  gstreamer-base-1.0  gstbase-1.0)
find_gstreamer_component(gstapp   gstreamer-app-1.0   gstapp-1.0)
find_gstreamer_component(gstvideo gstreamer-video-1.0 gstvideo-1.0)

#message(STATUS "inc: ${gstreamer_INCLUDE_DIRS}")
#message(STATUS "lib: ${gstreamer_LIBRARIES}")

# C++11 required!
include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_STD_CXX11)
if (COMPILER_SUPPORTS_STD_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else ()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES gstvideoserver
  CATKIN_DEPENDS nodelet image_transport roscpp sensor_msgs std_msgs
  DEPENDS libgstreamer1.0-dev
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${gstreamer_INCLUDE_DIRS}
)

add_library(gstvideoserver
  src/server_nodelet.cpp
)
target_link_libraries(gstvideoserver
   ${catkin_LIBRARIES}
   ${gstreamer_LIBRARIES}
)

add_executable(server_node
  src/node.cpp
)
target_link_libraries(server_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS gstvideoserver server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rtsp_streamer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# vim:set ts=2 sw=2 et :
