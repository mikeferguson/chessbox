#!/usr/bin/env python

"""
Test the object insertion
"""

import rospy
from chess_player.robot_defs import *
from chess_player.chess_utilities import *

if __name__=='__main__':
    rospy.init_node('test_chess_grasping')
    move = ChessArmPlanner()

    rospy.sleep(3) # sleep for tf

    b = BoardState()
    b.newGame()

    move.update_objects(b)
