cmake_minimum_required(VERSION 3.0.2)
project(navegador_autonomo)

# Encontra as dependências necessárias do Catkin
find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  tf2_ros
)

# Informa ao catkin sobre as dependências do pacote
catkin_package(
  CATKIN_DEPENDS rospy nav_msgs geometry_msgs tf2_ros
)

# Declara quais scripts Python devem ser instalados.
catkin_install_python(PROGRAMS
  scripts/navegador_final.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
