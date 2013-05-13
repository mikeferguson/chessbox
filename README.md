## Chess for Robots

This is the n-th generation chess code, which was originally developed for AAAI 2011.

## Installation

    sudo apt-get install gnuchess gnuchess-book
    sudo apt-get install festlex-cmu ros-groovy-sound-drivers

    cd <catkin_ws>/src
    wstool init
    wstool merge https://raw.github.com/mikeferguson/chessbox/master/maxwell.rosinstall
    wstool update
    cd ..
    catkin_make

## Setup for festival

    cd /usr/share/festival/voices/english
    sudo wget -c http://www.speech.cs.cmu.edu/cmu_arctic/packed/cmu_us_awb_arctic-0.95-release.tar.bz2
    sudo tar jxf cmu_us_awb_arctic-0.95-release.tar.bz2 
    sudo ln -s cmu_us_awb_arctic cmu_us_awb_arctic_clunits

## Running

    roslaunch maxwell_defs bringup_maxwell.launch
    roslaunch maxwell_defs head_camera.launch
    roslaunch maxwell_moveit_config move_group.launch
    roslaunch chess_player play.launch

## Running without the robot

    roslaunch maxwell_defs fake_maxwell.launch
    roslaunch maxwell_moveit_config move_group.launch
    rosrun tf static_transform_publisher .15 .2286 .7366 -1.57 0 0 base_link chess_board 20
    rosrun chess_perception chess_perception
    rosrun ches_player chess_executive.py _side:=white --sim
