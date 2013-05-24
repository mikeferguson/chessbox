## Chess for Robots

This is the n-th generation chess code, which was originally developed for AAAI 2011.

## Installation

    sudo apt-get install gnuchess gnuchess-book
    sudo apt-get install festlex-cmu ros-groovy-sound-drivers
    sudo apt-get install python-pexpect ros-groovy-moveit-full

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

## Running on Maxwell -- pretty much only I can do this...

    roslaunch maxwell_defs bringup_maxwell.launch
    roslaunch maxwell_defs head_camera.launch
    roslaunch maxwell_moveit_config move_group.launch
    roslaunch chess_player play.launch

## Running without the robot
You can startup most of the pipeline without having a Maxwell:

    roslaunch maxwell_defs fake_maxwell.launch
    roslaunch maxwell_moveit_config move_group.launch
    rosrun tf static_transform_publisher .15 .2286 .7366 -1.57 0 0 base_link chess_board 20

You probably want to view things. So "rosrun rviz rviz". Be sure to add a PlanningScene display, and set the Planning Scene topic to "move_group/monitored_planning_scene". You can then run through some tests. For instance, it's nice to tuck the arm (it will start up pointing straight out):

    rosrun chess_player tuck_arm.py

The next test is to move all pawns forward 2 steps. You should be able to watch the chess board be created, a bunch of little blocks be inserted, and then all of the "pawns" be moved forward. Startup may take some time:

    rosrun chess_player grasp_utilities.py

Finally, you *might* be able to run the full executive and have the robot move some pieces, but this does get broken from time to time:

    rosrun chess_player chess_executive.py --sim
