cmake_minimum_required(VERSION 2.8.3)
project(chess_msgs)

find_package(catkin REQUIRED genmsg geometry_msgs)

add_message_files(FILES
                  ChessPiece.msg
                  ChessBoard.msg
                 )

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(DEPENDS geometry_msgs)
