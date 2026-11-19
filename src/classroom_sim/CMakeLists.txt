cmake_minimum_required(VERSION 3.8)
project(classroom_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds urdf config rviz models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
