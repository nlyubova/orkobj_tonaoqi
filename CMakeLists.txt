## Author(s):
##  - Natalia Lyubova <nlyubova@aldebaran-robotics.com>
## Copyright (c) Aldebaran Robotics 2014 All Rights Reserved

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME msgrostonaoqi)

project(${PROJECTNAME})

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(NAOqi QUIET COMPONENTS
    alcommon
    alproxies
    alerror
    alvalue
    qi
    qitype
)

find_package(Boost QUIET COMPONENTS program_options signals)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(object_recognition_msgs REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp std_msgs object_recognition_msgs
)

set(SRCS
  src/msgrostonaoqi.cpp
  src/main.cpp
)

if( ${NAOqi_FOUND} AND ${Boost_FOUND} )
  message(STATUS "NAOqi C++ SDK found; C++ nodes will be built")

  include_directories( SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${object_recognition_msgs_INCLUDE_DIRS}
    ${NAOqi_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

  add_executable(${PROJECTNAME} ${SRCS})
  target_link_libraries(${PROJECTNAME}
    ${catkin_LIBRARIES}
    ${NAOqi_LIBRARIES}
    ${Boost_LIBRARIES})
  add_dependencies(${PROJECTNAME} ${catkin_EXPORTED_TARGETS})

  install(TARGETS ${PROJECTNAME}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
  message(STATUS "Cannot find NAOqi C++ sdk; C++ nodes will NOT be built")
endif()

install(TARGETS ${PROJECTNAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
