cmake_minimum_required(VERSION 3.0.2)
project(coppelia_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# (opcional) instalar o script ao fazer 'catkin_make install'
install(PROGRAMS
  scripts/ScriptComandoComCaptura.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
