#!/usr/bin/env python3

""" 
  Simple executive for playing AAAI robot chess
  Copyright (c) 2011-2024 Michael E. Ferguson.  All right reserved.

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

import sys
import rclpy
from rclpy.node import Node

from chess_msgs.msg import ChessBoard
from std_srvs.srv import *

from tf2_ros.buffer import buffer
from tf2_ros.transform_listener import TransformListener

from chess_player.chess_utilities import (
    BoardState,
    BoardUpdater,
    GnuChessEngine,
    castling_extras
)
from chess_player.grasp_utilities import ChessArmPlanner
from chess_player.sound_utilities import SpeechEngine
from chess_player.head_utilities import HeadEngine

###############################################################################
# Executive for managing chess game

class ChessExecutive(Node):
    def __init__(self, sim = False):
        """ Start the executive node """
        super().__init__('chess_executive')

        self.interactive = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.sim = sim

        # get arm planner
        self.get_logger().info('exec: Waiting for actions to connect.')
        self.planner = ChessArmPlanner(self, buffer=self.buffer)
        self.planner.start()

        self.board = BoardState()

        if self.sim:
            self.yourMove = self.yourMoveKeyboard
            self.board.side = self.board.WHITE
        else:
            self.yourMove = self.yourMovePerception
            self.perception_times = list()

            # subscribe to input
            self.updater = BoardUpdater(self.board)
            self.subscriber = self.create_subscription(ChessBoard, 'chess_board_state', self.updater.callback, 10)

            # maybe set side?
            #try:
            #    node.declare_parameter('side', )
            #    s = node.get_parameter('side')
            #    if s == 'w' or s == 'white':
            #        self.board.side = self.board.WHITE
            #    else:
            #        self.board.side = self.board.BLACK
            #except:
            self.get_logger().info('No side set, will attempt to determine')

        # move the head and talk
        self.speech = SpeechEngine()
        self.head = HeadEngine()

        self.get_logger().info('exec: Done initializing...')

    ###########################################################################
    # your move prototypes

    def yourMoveKeyboard(self, suppress_output = False):
        if not suppress_output:
            self.head.look_at_board()
            print('Your move:')
        else:
            print('Invalid move, try again:')
        self.board.last_move = input().rstrip()
        if self.board.last_move == 'exit':
            self.engine.exit()
            exit()
        self.board.applyMove(self.board.last_move)

    def yourMovePerception(self, suppress_output = False):
        if not suppress_output:
            self.speech.say("Your move.")
            #rospy.sleep(10.0)
            self.head.look_at_board()
            #rospy.sleep(10.0)
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
        #if not self.sim:
        #    rospy.sleep(5.0)

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
                self.get_logger().info("exec: Bad move...")
                self.yourMove(True)
                move = self.getMove()
            # remove a captured piece from the board
            if self.updater.last_capture != None:
                self.planner._obj.remove(self.updater.last_capture)
            # do move
            if self.board.last_move != "go":
                self.speech.say("I see you have moved your " + self.board.getMoveText(self.board.last_move))
            self.get_logger().info("exec: My move: %s", move)
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
        updated_t = self.get_clock().now()
        while rclpy.ok():
            if (self.get_clock().now() - updated_t).seconds() > 5.0:
                self.head.wiggle_head()
                updated_t = self.get_clock().now()
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
                updated_t = self.get_clock().now()
            #rospy.sleep(0.1)
        self.board.printBoard()
        # pass transform
        self.planner.transform = self.updater.transform

    def getMove(self):
        return self.engine.nextMove(self.board.last_move, self.board)

if __name__=="__main__":
    rclpy.init()
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

