cmake_minimum_required(VERSION 2.8.3)
project(rc_monitor)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs quadrotor_msgs nodelet mav_manager cmake_modules)
find_package(Eigen 3 REQUIRED)

# If mavros_msgs not found, don't build but do not cause a build error
find_package(mavros_msgs)

if(NOT ${mavros_msgs_FOUND})
  message(WARNING "NOTE: mavros_msgs not found so not building mavros_interface")
else()
  include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp nav_msgs geometry_msgs quadrotor_msgs mavros_msgs nodelet
    DEPENDS mav_manager Eigen)
    
  #add_library(sub_library rc_monitor/rc_monitor.h)
  add_executable(rc_monitor src/rc_monitor.cpp)
  target_link_libraries(rc_monitor ${catkin_LIBRARIES})

  install(TARGETS rc_monitor
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

#  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
#  install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
