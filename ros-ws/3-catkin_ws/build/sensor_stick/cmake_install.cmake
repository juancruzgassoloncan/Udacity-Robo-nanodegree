# Install script for directory: /media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/src/sensor_stick

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/install")
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
  include("/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/build/sensor_stick/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_stick/msg" TYPE FILE FILES
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/src/sensor_stick/msg/DetectedObject.msg"
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/src/sensor_stick/msg/DetectedObjectsArray.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_stick/srv" TYPE FILE FILES
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/src/sensor_stick/srv/GetNormals.srv"
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/src/sensor_stick/srv/GetFloatArrayFeature.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_stick/cmake" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/build/sensor_stick/catkin_generated/installspace/sensor_stick-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/devel/include/sensor_stick")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/devel/share/roseus/ros/sensor_stick")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/devel/share/common-lisp/ros/sensor_stick")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/devel/share/gennodejs/ros/sensor_stick")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/devel/lib/python2.7/dist-packages/sensor_stick")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/devel/lib/python2.7/dist-packages/sensor_stick" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/devel/lib/python2.7/dist-packages/sensor_stick" FILES_MATCHING REGEX "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/devel/lib/python2.7/dist-packages/sensor_stick/.+/__init__.pyc?$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/sensor_stick" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/devel/include/sensor_stick/PclConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/devel/lib/python2.7/dist-packages/sensor_stick/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/sensor_stick" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/devel/lib/python2.7/dist-packages/sensor_stick/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/build/sensor_stick/catkin_generated/installspace/sensor_stick.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_stick/cmake" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/build/sensor_stick/catkin_generated/installspace/sensor_stick-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_stick/cmake" TYPE FILE FILES
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/build/sensor_stick/catkin_generated/installspace/sensor_stickConfig.cmake"
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/build/sensor_stick/catkin_generated/installspace/sensor_stickConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_stick" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/3-catkin_ws/src/sensor_stick/package.xml")
endif()

