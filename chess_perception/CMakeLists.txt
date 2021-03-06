cmake_minimum_required(VERSION 3.0.2)
project(chess_perception)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

# setup
find_package(catkin REQUIRED roscpp cv_bridge image_transport tf pcl_ros pcl_conversions chess_msgs)
find_package(Boost REQUIRED system filesystem)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

catkin_package(DEPENDS pcl_ros roscpp cv_bridge image_transport tf chess_msgs)

include_directories(include
                    SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                   )

# node
add_executable(chess_perception src/piece_finder.cpp
                                src/board_finder.cpp
                                src/chess_perception.cpp)
target_link_libraries(chess_perception ${Boost_LIBRARIES}
                                       ${catkin_LIBRARIES}
                                       ${PCL_LIBRARIES}
                                       ${OpenCV_LIBRARIES})
add_dependencies(chess_perception chess_msgs_gencpp)
