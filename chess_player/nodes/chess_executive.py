#!/usr/bin/env python

""" 
  Simple executive for playing AAAI robot chess
  Copyright (c) 2011 Michael E. Ferguson.  All right reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

from __future__ import print_function

import sys
import rospy

from chess_msgs.msg import *
from std_srvs.srv import *

from tf.listener import *

from chess_player.chess_utilities import *
from chess_player.sound_utilities import *
from chess_player.head_utilities import *

###############################################################################
# Executive for managing chess game

class ChessExecutive:
    def __init__(self, sim = False):
        """ Start the executive node """
        rospy.init_node("chess_executive")
        self.interactive = False
        self.listener = TransformListener()
        self.sim = sim

        # get arm planner
        rospy.loginfo('exec: Waiting for actions to connect.')
        self.planner = ChessArmPlanner(listener = self.listener)

        self.board = BoardState()

        if self.sim:
            self.yourMove = self.yourMoveKeyboard
            self.board.side = self.board.WHITE
            rospy.sleep(3.0)
        else:
            self.yourMove = self.yourMovePerception
            self.perception_times = list()

            # subscribe to input
            self.updater = BoardUpdater(self.board)
            rospy.Subscriber('chess_board_state', ChessBoard, self.updater.callback)

            # maybe set side?
            try:
                s = rospy.get_param('side')
                if s == 'w' or s == 'white':
                    self.board.side = self.board.WHITE
                else:
                    self.board.side = self.board.BLACK
            except:
                rospy.loginfo('No side set, will attempt to determine')

        # move the head and talk
        self.speech = SpeechEngine()
        self.head = HeadEngine()

        rospy.loginfo('exec: Done initializing...')

    ###########################################################################
    # your move prototypes

    def yourMoveKeyboard(self, suppress_output = False):
        if not suppress_output:
            self.head.look_at_board()
            print('Your move:')
        else:
            print('Invalid move, try again:')
        self.board.last_move = raw_input().rstrip()
        if self.board.last_move == 'exit':
            self.engine.exit()
            exit()
        self.board.applyMove(self.board.last_move)

    def yourMovePerception(self, suppress_output = False):
        if not suppress_output:
            self.speech.say("Your move.")
            rospy.sleep(10.0)
            self.head.look_at_board()
            rospy.sleep(10.0)
        # update board state
        self.updateBoardState()

    ###########################################################################
    # game playing

    def playGame(self):
        """ This function plays a complete game. """

        # default board representation
        self.engine = GnuChessEngine()
        self.board.newGame()
        self.head.look_at_board()
        if not self.sim:
            rospy.sleep(5.0)

        # are we white/black?
        if not self.sim:
            self.updateBoardState(True)
        if self.board.side == None:
            self.board.computeSide()

        if self.board.side == self.board.BLACK:
            self.board.setupSide()
            self.head.look_at_player()
            self.speech.say("Ok, I'll play black")
            # wait for opponents move
            self.yourMove()
        else:        
            self.speech.say("Ok, I'll play white. my turn")

        # loop!
        while not rospy.is_shutdown(): 
            # do move
            move = self.getMove()
            while move == None and not rospy.is_shutdown():
                # update board state
                self.board.revert()
                rospy.loginfo("exec: Bad move...")
                self.yourMove(True)
                move = self.getMove()
            # do move
            if self.board.last_move != "go":
                self.speech.say("I see you have moved your " + self.board.getMoveText(self.board.last_move))
            rospy.loginfo("exec: My move: %s", move)
            if move in castling_extras.keys():
                self.speech.say("Why oh why am I castling?")
            else:
                self.speech.say("Moving my " + self.board.getMoveText(move))
            self.board.applyMove(move, self.planner.execute(move,self.board))
            if not self.planner.success: 
                self.engine.startPawning()
                self.speech.say("Oh crap! I have failed")

            # wait for opponents move
            self.yourMove()
    
    def updateBoardState(self, acceptNone = False):
        """ Updates board state by triggering pipeline. """
        self.updater.up_to_date = False
        updated_t = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now()-updated_t).to_sec() > 5.0:
                self.head.wiggle_head()
                updated_t = rospy.Time.now()
            if self.updater.up_to_date:
                if self.board.last_move == "none":
                    if acceptNone:
                        break
                    else:
                        self.updater.up_to_date = False
                else:
                    if acceptNone:
                        self.updater.up_to_date = False
                    else:
                        break
                updated_t = rospy.Time.now()
            rospy.sleep(0.1)
        self.board.printBoard()
        # pass transform
        self.planner.transform = self.updater.transform

    def getMove(self):
        return self.engine.nextMove(self.board.last_move, self.board)

if __name__=="__main__":
    sim = False
    if '--sim' in sys.argv:
        sim = True
    try:
        executive = ChessExecutive(sim)
        executive.playGame()
        print('Final board state:')
        executive.board.printBoard()
        # shutdown gnuchess, so it doesn't shut us down
        executive.engine.exit()
    except KeyboardInterrupt:
        pass

