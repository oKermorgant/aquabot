cmake_minimum_required(VERSION 3.15)
project(asv_race_gz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models DESTINATION share/${PROJECT_NAME})

ament_package()
