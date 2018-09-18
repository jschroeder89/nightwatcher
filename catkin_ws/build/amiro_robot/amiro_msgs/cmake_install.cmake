# Install script for directory: /homes/joschroeder/github/nightwatcher/catkin_ws/src/amiro_robot/amiro_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/homes/joschroeder/github/nightwatcher/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/amiro_msgs/msg" TYPE FILE FILES
    "/homes/joschroeder/github/nightwatcher/catkin_ws/src/amiro_robot/amiro_msgs/msg/UInt16MultiArrayStamped.msg"
    "/homes/joschroeder/github/nightwatcher/catkin_ws/src/amiro_robot/amiro_msgs/msg/Int32MultiArrayStamped.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/amiro_msgs/cmake" TYPE FILE FILES "/homes/joschroeder/github/nightwatcher/catkin_ws/build/amiro_robot/amiro_msgs/catkin_generated/installspace/amiro_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/homes/joschroeder/github/nightwatcher/catkin_ws/devel/include/amiro_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/homes/joschroeder/github/nightwatcher/catkin_ws/devel/share/roseus/ros/amiro_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/homes/joschroeder/github/nightwatcher/catkin_ws/devel/share/common-lisp/ros/amiro_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/homes/joschroeder/github/nightwatcher/catkin_ws/devel/share/gennodejs/ros/amiro_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/homes/joschroeder/github/nightwatcher/catkin_ws/devel/lib/python2.7/dist-packages/amiro_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/homes/joschroeder/github/nightwatcher/catkin_ws/devel/lib/python2.7/dist-packages/amiro_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/homes/joschroeder/github/nightwatcher/catkin_ws/build/amiro_robot/amiro_msgs/catkin_generated/installspace/amiro_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/amiro_msgs/cmake" TYPE FILE FILES "/homes/joschroeder/github/nightwatcher/catkin_ws/build/amiro_robot/amiro_msgs/catkin_generated/installspace/amiro_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/amiro_msgs/cmake" TYPE FILE FILES
    "/homes/joschroeder/github/nightwatcher/catkin_ws/build/amiro_robot/amiro_msgs/catkin_generated/installspace/amiro_msgsConfig.cmake"
    "/homes/joschroeder/github/nightwatcher/catkin_ws/build/amiro_robot/amiro_msgs/catkin_generated/installspace/amiro_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/amiro_msgs" TYPE FILE FILES "/homes/joschroeder/github/nightwatcher/catkin_ws/src/amiro_robot/amiro_msgs/package.xml")
endif()

