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

import roslib; roslib.load_manifest('chess_player')
import rospy

from chess_msgs.msg import *
from std_srvs.srv import *

from tf_utilities import *
from chess_utilities import *
from arm_utilities import *
from sound_utilities import *
from head_utilities import *

###############################################################################
# Executive for managing chess game

class ChessExecutive:
    def __init__(self):
        """ Start the executive node """
        rospy.init_node("chess_executive")
        self.interactive = False
        self.listener = SyncListener()

        # connect to camera_turnpike service
        rospy.loginfo('exec: Waiting for /camera_turnpike/trigger')
        rospy.wait_for_service('/camera_turnpike/trigger')
        self.camera_trigger = rospy.ServiceProxy('/camera_turnpike/trigger', Empty)

        # get arm planner
        rospy.loginfo('exec: Waiting for /simple_arm_server/move')
        #rospy.wait_for_service('/simple_arm_server/move')
        #self.planner = ArmPlanner( rospy.ServiceProxy('/simple_arm_server/move', MoveArm), listener = self.listener )
        #self.planner = actionlib.SimpleActionClient('move_arm', MoveArmAction)
        self.planner = ArmPlanner( listener = self.listener )

        # subscribe to input
        self.board = BoardState()
        self.updater = BoardUpdater(self.board, self.listener)
        rospy.Subscriber('/extract_pieces/output', ChessBoard, self.updater.callback) 

        # subscribe to your move services
        self.yourMove = self.yourMovePerception

        self.speech = SpeechEngine()
        self.head = HeadEngine()
        self.perception_times = list()

        # maybe set side?
        try:
            s = rospy.get_param("side")
            if s == "w" or s == "white":
                self.board.side = self.board.WHITE
            else:
                self.board.side = self.board.BLACK
        except:
            rospy.loginfo("No side set, will attempt to determine")

        rospy.loginfo("exec: Done initializing...")


    ###########################################################################
    # your move prototypes

    def yourMoveKeyboard(self):
        # stupid little function to know when move has been made
        print "Please press enter after making a move"
        x= raw_input()        
        if x.rstrip() == "exit":
            self.engine.exit()
            exit()            

    def yourMovePerception(self):
        self.speech.say("Your move.")

    def trigger(self):
        try:
            self.camera_trigger()
        except:
            pass    # let's not crash on exit

    ###########################################################################
    # game playing

    def playGame(self):
        """ This function plays a complete game. """

        # default board representation
        self.engine = GnuChessEngine()
        self.board.newGame()
        self.head.look_at_board()
        rospy.sleep(5.0)

        # are we white/black?
        self.updateBoardState(True)
        if self.board.side == None:
            self.board.computeSide()

        if self.board.side == self.board.BLACK:
            self.head.look_at_player()
            self.speech.say("Ok, I'll play black")
            # wait for opponents move
            self.yourMove()
            rospy.sleep(10.0)
            self.head.look_at_board()
            rospy.sleep(10.0)
            # update board state
            self.updateBoardState()
        else:        
            self.speech.say("Ok, I'll play white. my turn")

        # loop!
        while not rospy.is_shutdown(): 
            # do move
            move = self.getMove()
            while move == None and not rospy.is_shutdown():
                # update board state    
                self.board.revert()
                rospy.loginfo("exec: Bad move, triggering again...")
                self.updateBoardState()
                move = self.getMove()
            # do move
            self.head.look_at_player()
            if self.board.last_move != "go":
                self.speech.say("I see you have moved your " + self.board.getMoveText(self.board.last_move))
            rospy.loginfo("exec: My move: %s", move)
            if move in castling_extras.keys():
                self.speech.say("Why oh why am I castling?")
            else:
                self.speech.say("Moving my " + self.board.getMoveText(move))
            #self.head.look_at_board()
            self.board.applyMove(move, self.planner.execute(move,self.board))
            if not self.planner.success: 
                self.engine.startPawning()
                self.speech.say("Oh crap! I have failed")

            # wait for opponents move
            #self.head.look_at_player()
            self.yourMove()
            #rospy.sleep(10.0)
            self.head.look_at_board()
            rospy.sleep(5.0)

            # update board state
            self.updateBoardState()
    
    def updateBoardState(self, acceptNone = False):
        """ Updates board state by triggering pipeline. """
        self.updater.up_to_date = False
        rospy.loginfo("exec: Triggering...")
        self.trigger()
        output_t = rospy.Time.now()
        updated_t = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now()-output_t).to_sec() > 5.0:
                self.board.output = True
                self.head.wiggle_head()
                output_t = rospy.Time.now()
            if (rospy.Time.now()-updated_t).to_sec() > 5.0:
                rospy.logerr("exec: Failed to get updates, triggering again")
                self.trigger()
                updated_t = rospy.Time.now()
            if self.updater.up_to_date:
                if self.board.last_move == "fail":
                    self.updater.up_to_date = False 
                    rospy.loginfo("exec: Triggering again...")
                    self.trigger()
                elif self.board.last_move == "none":
                    if acceptNone:
                        break
                    else:
                        self.updater.up_to_date = False 
                        rospy.loginfo("exec: Triggering again...")
                        self.trigger()
                else:
                    if acceptNone:
                        self.updater.up_to_date = False 
                        rospy.loginfo("exec: Triggering again...")
                        self.trigger()
                    else:
                        break
                updated_t = rospy.Time.now()
            rospy.sleep(0.1)
        self.board.printBoard()

    def getMove(self):
        move = self.engine.nextMove(self.board.last_move, self.board)
        # check length of move
        if move != None:
            if move in castling_extras.keys():
                # asked to castle!
                m = castling_extras[move]
                reach = self.planner.getReach(m[0], m[1], self.board) 
                if reach > 0.45 and not self.interactive:
                    rospy.loginfo("Move " + m + ", source has reach of " + str(reach) + ", starting to pawn")
                    self.engine.startPawning()
                    return self.engine.nextMove(self.board.last_move, self.board)        
                reach = self.planner.getReach(m[2], m[3], self.board) 
                if reach > 0.45 and not self.interactive:
                    rospy.loginfo("Move " + m + ", destination has reach of " + str(reach) + ", starting to pawn")
                    self.engine.startPawning()
                    return self.engine.nextMove(self.board.last_move, self.board)   

            reach = self.planner.getReach(move[2], move[3], self.board) 
            if reach > 0.45 and not self.interactive:
                rospy.loginfo("Move " + move + " has reach of " + str(reach) + ", starting to pawn")
                self.engine.startPawning()
                return self.engine.nextMove(self.board.last_move, self.board)            
        return move

if __name__=="__main__":
    try:
        executive = ChessExecutive()
        executive.playGame()
        print "Final board state:"
        executive.board.printBoard()
        executive.head.look_at_board()
        # shutdown gnuchess, so it doesn't shut us down
        executive.engine.exit()
    except KeyboardInterrupt:
        pass

