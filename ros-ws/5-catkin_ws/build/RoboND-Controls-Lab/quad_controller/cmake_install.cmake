# Install script for directory: /media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/src/RoboND-Controls-Lab/quad_controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/install")
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
  include("/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/build/RoboND-Controls-Lab/quad_controller/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quad_controller/msg" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/src/RoboND-Controls-Lab/quad_controller/msg/EulerAngles.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quad_controller/srv" TYPE FILE FILES
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/src/RoboND-Controls-Lab/quad_controller/srv/SetFloat.srv"
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/src/RoboND-Controls-Lab/quad_controller/srv/SetInt.srv"
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/src/RoboND-Controls-Lab/quad_controller/srv/SetPose.srv"
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/src/RoboND-Controls-Lab/quad_controller/srv/SetPath.srv"
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/src/RoboND-Controls-Lab/quad_controller/srv/GetPath.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quad_controller/cmake" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/build/RoboND-Controls-Lab/quad_controller/catkin_generated/installspace/quad_controller-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/include/quad_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/share/roseus/ros/quad_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/share/common-lisp/ros/quad_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/share/gennodejs/ros/quad_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/lib/python2.7/dist-packages/quad_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/lib/python2.7/dist-packages/quad_controller" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/lib/python2.7/dist-packages/quad_controller" FILES_MATCHING REGEX "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/lib/python2.7/dist-packages/quad_controller/.+/__init__.pyc?$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/quad_controller" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/include/quad_controller/attitude_controller_paramsConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/quad_controller" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/include/quad_controller/position_controller_paramsConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/quad_controller" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/include/quad_controller/hover_controller_paramsConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/lib/python2.7/dist-packages/quad_controller/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/quad_controller" TYPE DIRECTORY FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/devel/lib/python2.7/dist-packages/quad_controller/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/build/RoboND-Controls-Lab/quad_controller/catkin_generated/installspace/quad_controller.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quad_controller/cmake" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/build/RoboND-Controls-Lab/quad_controller/catkin_generated/installspace/quad_controller-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quad_controller/cmake" TYPE FILE FILES
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/build/RoboND-Controls-Lab/quad_controller/catkin_generated/installspace/quad_controllerConfig.cmake"
    "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/build/RoboND-Controls-Lab/quad_controller/catkin_generated/installspace/quad_controllerConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quad_controller" TYPE FILE FILES "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/src/RoboND-Controls-Lab/quad_controller/package.xml")
endif()

