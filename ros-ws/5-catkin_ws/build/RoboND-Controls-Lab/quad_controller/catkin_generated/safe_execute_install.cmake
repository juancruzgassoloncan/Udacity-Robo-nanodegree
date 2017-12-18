execute_process(COMMAND "/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/build/RoboND-Controls-Lab/quad_controller/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/build/RoboND-Controls-Lab/quad_controller/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
