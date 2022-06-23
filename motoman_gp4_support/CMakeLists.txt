cmake_minimum_required(VERSION 3.12)

project(motoman_gp4_support)

find_package(ament_cmake REQUIRED)
ament_package()

install(DIRECTORY config meshes urdf
  DESTINATION share/${PROJECT_NAME})
