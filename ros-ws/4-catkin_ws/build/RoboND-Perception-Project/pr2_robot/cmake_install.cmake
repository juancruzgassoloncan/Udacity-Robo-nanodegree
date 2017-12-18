# Install script for directory: /media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/src/RoboND-Perception-Project/pr2_robot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_robot/srv" TYPE FILE FILES
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/src/RoboND-Perception-Project/pr2_robot/srv/PickPlace.srv"
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/src/RoboND-Perception-Project/pr2_robot/srv/Grasp.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_robot/cmake" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/build/RoboND-Perception-Project/pr2_robot/catkin_generated/installspace/pr2_robot-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/devel/include/pr2_robot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/devel/share/roseus/ros/pr2_robot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/devel/share/common-lisp/ros/pr2_robot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/devel/share/gennodejs/ros/pr2_robot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/devel/lib/python2.7/dist-packages/pr2_robot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/devel/lib/python2.7/dist-packages/pr2_robot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/build/RoboND-Perception-Project/pr2_robot/catkin_generated/installspace/pr2_robot.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_robot/cmake" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/build/RoboND-Perception-Project/pr2_robot/catkin_generated/installspace/pr2_robot-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_robot/cmake" TYPE FILE FILES
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/build/RoboND-Perception-Project/pr2_robot/catkin_generated/installspace/pr2_robotConfig.cmake"
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/build/RoboND-Perception-Project/pr2_robot/catkin_generated/installspace/pr2_robotConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_robot" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/4-catkin_ws/src/RoboND-Perception-Project/pr2_robot/package.xml")
endif()

