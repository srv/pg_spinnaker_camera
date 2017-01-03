cmake_minimum_required(VERSION 2.8.3)
project(pg_spinnaker_camera)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  diagnostic_updater
  dynamic_reconfigure
  image_geometry
  image_transport
  message_filters
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
)

# If the user has manually installed flycapture, use the system path version. If not,
# download it. We can't resolve this dependency using the usual rosdep means because
# the Point Grey EULA prohibits redistributing the headers or the packages which
# contains them. Fortunately, we can download the archive directly from their
# website during this build process.
find_library(Spinnaker_LIBRARIES Spinnaker)
if(NOT Spinnaker_LIBRARIES)
  message(STATUS "libSpinnaker not found in system library path")
  include(cmake/DownloadSpinnaker.cmake)
  download_spinnaker(Spinnaker_LIBRARIES Spinnaker_INCLUDE_DIRS)
  message(STATUS "libSpinnaker library: ${Spinnaker_LIBRARIES}")
  message(STATUS "libSpinnaker include: ${Spinnaker_INCLUDE_DIRS}")
  include_directories(${Spinnaker_INCLUDE_DIRS})
endif()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS camera_info_manager diagnostic_updater dynamic_reconfigure image_geometry image_transport roscpp sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(pg_stereo_camera
  src/nodes/stereo_camera_node.cpp
  src/stereo_camera.cpp)
target_link_libraries(pg_stereo_camera
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Spinnaker_LIBRARIES})

add_dependencies(pg_stereo_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
