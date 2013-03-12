cmake_minimum_required(VERSION 2.8.3)
project(chess_player)


# setup
find_package(catkin REQUIRED dynamic_reconfigure roscpp cv_bridge image_transport tf)
find_package(Boost REQUIRED system filesystem)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

generate_dynamic_reconfigure_options(cfg/Locator.cfg
                                     cfg/PieceExtractionConfig.cfg)

catkin_package(DEPENDS dynamic_reconfigure pcl_ros pcl roscpp cv_bridge image_transport tf)

include_directories(include
                    SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    cfg/cpp)

# board locator
add_executable(chess_board_locator src/chess_board_locator.cpp)
target_link_libraries(chess_board_locator ${Boost_LIBRARIES}
                                          ${catkin_LIBRARIES}
                                          ${PCL_LIBRARIES}
                                          ${OpenCV_LIBRARIES})
add_dependencies(chess_board_locator chess_player_gencfg chess_msgs_gencpp)


# piece extraction
add_library(piece_extraction src/extract_pieces.cpp)
target_link_libraries(piece_extraction ${Boost_LIBRARIES}
                                       ${catkin_LIBRARIES}
                                       ${PCL_LIBRARIES}
                                       ${OpenCV_LIBRARIES})
add_dependencies(piece_extraction chess_player_gencfg chess_msgs_gencpp)
            
# Point Cloud Library - Segmentation
#rosbuild_add_library (piece_extraction
#                      src/extract_pieces.cpp
#                     )

#rosbuild_add_compile_flags (pcl_ros_segmentation ${SSE_FLAGS})
#rosbuild_link_boost (piece_extraction system filesystem)
