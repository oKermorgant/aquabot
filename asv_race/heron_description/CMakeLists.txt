cmake_minimum_required(VERSION 3.15)
project(heron_description)

find_package(ament_cmake REQUIRED)

# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")

install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})

ament_package()
