cmake_minimum_required(VERSION 3.5)
project(chess_perception)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(chess_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

#find_package(Boost REQUIRED system filesystem)
#link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(chess_perception
  src/board_finder.cpp
  src/chess_perception.cpp
  src/piece_finder.cpp
)
ament_target_dependencies(chess_perception
  chess_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  rclcpp
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(chess_perception
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS
    chess_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
