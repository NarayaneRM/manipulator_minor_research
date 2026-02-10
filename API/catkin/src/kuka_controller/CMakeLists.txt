cmake_minimum_required(VERSION 3.0.2)
project(kuka_controller)

## Encontrar pacotes necessários
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## Declarar que este é um pacote Catkin
catkin_package()

## Incluir diretórios de cabeçalhos (se necessário)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
