cmake_minimum_required(VERSION 3.8)
project(kros_car_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
