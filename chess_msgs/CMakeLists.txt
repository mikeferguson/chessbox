cmake_minimum_required(VERSION 3.0.2)
project(chess_msgs)

find_package(catkin REQUIRED message_generation geometry_msgs)

add_message_files(FILES
                  ChessPiece.msg
                  ChessBoard.msg
                 )

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(DEPENDS message_runtime geometry_msgs)
