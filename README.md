## Chess for Robots

This is the n-th generation chess code, which was originally developed for AAAI 2011.

## Installation

    sudo apt-get install gnuchess gnuchess-book python-pexpect
    sudo apt-get install festlex-cmu ros-groovy-audio-common
    sudo apt-get install ros-hydro-moveit-full

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

You probably want to view things. So "rosrun rviz rviz". Be sure to add a PlanningScene display, and set
the Planning Scene topic to "move_group/monitored_planning_scene". You can then run through some tests.
For instance, it's nice to tuck the arm (it will start up pointing straight out):

    rosrun chess_player tuck_arm.py

The next test is to move all pawns forward 2 steps. You should be able to watch the chess board be created,
a bunch of little blocks be inserted, and then all of the "pawns" be moved forward. Startup may take some time:

    rosrun chess_player grasp_utilities.py

Finally, you *might* be able to run the full executive and have the robot move some pieces, but this does get
broken from time to time:

    rosrun chess_player chess_executive.py --sim

## Running on that other robot

    rosrun tf static_transform_publisher .4 .2286 .7366 -1.57 0 0 base_footprint chess_board 20

## Adapting to a new robot
Much of this code is robot-independent. Below I list the files known not to be (in other words, if the file
isn't in this list, it is probably robot-independent).

### chess_perception
The only robot-related aspect is that the head should be pointed at the board. Also, if your robot does not
have *base_link*, you will need to set ~fixed_frame for the chess_perception_node. The other major issue is
that the perception code currently is not entirely robust to pieces being improperly placed on the board.

### chess_player/src/robot_defs.py
All of this file is potentially robot-specific.

### chess_player/src/head_utilities.py
This is currently hard coded with angles for Maxwell. Ideally this would eventually turn into a point_head
action, with a "search and then cache" function for "looking at the board".

### chess_player/src/grasp_utilities.py
Something should be done with OFF_BOARD positions used for capture.

The functions for getGripperPosture, getGripperTranslation, and all of the grasp generation stuff (getY, getP,
getGrasps, getPlaceLocations) are Maxwell-specific, although if your arm is *pretty good* at overhead grasps
this may just work for you.

### chess_player/nodes/tuck_arm.py
This just calls grasp_utilities.py. Obviously, this is currently hard-coded with the tuck positions of
Maxwell. Ideally we would load the tuck position from the SRDF file to make this robot-independent.

### chess_player/nodes/tilt_head.py
This calls head_utility.py stuff.
