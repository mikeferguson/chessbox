cmake_minimum_required(VERSION 3.5)
project(chess_player)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  PROGRAMS
    nodes/chess_executive.py
    nodes/festival_launcher.py
    nodes/move_head.py
    nodes/tuck_arm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
ament_package()
