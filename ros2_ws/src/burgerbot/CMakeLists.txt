cmake_minimum_required(VERSION 3.8)
project(burgerbot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  config
  launch
  maps
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
