#!/usr/bin/env python

"""
  Copyright (c) 2011 Michael E. Ferguson. All right reserved.

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

import copy, math
import rospy    # for logging
import pexpect  # for connecting to gnu chess
import threading

from chess_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import *

from chess_player.robot_defs import *

from moveit_utils.arm_interface import ArmInterface
from moveit_utils.grasping_interface import GraspingInterface
from moveit_utils.object_interface import ObjectInterface

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf.listener import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

SQUARE_SIZE = 0.05715

# extra move to be made for castling
castling_extras = { "e1c1" : "a1d1",
                    "e1g1" : "h1f1",
                    "e8c8" : "a8d8",
                    "e8g8" : "h8f8" }

class BoardState:
    """ A representation of a chess board state. """
    WHITE = 1
    BLACK = -1

    def __init__(self, side=None):
        """
        Initialize an empty board
        """
        self.values = [None for i in range(64)]
        self.last_move = "go"
        self.side = side
        self.max_changes = 2
        self.output = False
        self.castling_move = None

    def newGame(self):
        """
        Initialize a new board
        """
        self.last_move = "go"
        self.values = [None for i in range(64)]
        for i in range(8):
            self.setPiece(i, 2, self.makePiece(ChessPiece.WHITE_PAWN, i, 2))
            self.setPiece(i, 7, self.makePiece(ChessPiece.BLACK_PAWN, i, 7))

        self.setPiece('a', 1, self.makePiece(ChessPiece.WHITE_ROOK, 'a', 1))
        self.setPiece('b', 1, self.makePiece(ChessPiece.WHITE_KNIGHT, 'b', 1))
        self.setPiece('c', 1, self.makePiece(ChessPiece.WHITE_BISHOP, 'c', 1))
        self.setPiece('d', 1, self.makePiece(ChessPiece.WHITE_QUEEN, 'd', 1))
        self.setPiece('e', 1, self.makePiece(ChessPiece.WHITE_KING, 'e', 1))
        self.setPiece('f', 1, self.makePiece(ChessPiece.WHITE_BISHOP, 'f', 1))
        self.setPiece('g', 1, self.makePiece(ChessPiece.WHITE_KNIGHT, 'g', 1))
        self.setPiece('h', 1, self.makePiece(ChessPiece.WHITE_ROOK, 'h', 1))

        self.setPiece('a', 8, self.makePiece(ChessPiece.BLACK_ROOK, 'a', 8))
        self.setPiece('b', 8, self.makePiece(ChessPiece.BLACK_KNIGHT, 'b', 8))
        self.setPiece('c', 8, self.makePiece(ChessPiece.BLACK_BISHOP, 'c', 8))
        self.setPiece('d', 8, self.makePiece(ChessPiece.BLACK_QUEEN, 'd', 8))
        self.setPiece('e', 8, self.makePiece(ChessPiece.BLACK_KING, 'e', 8))
        self.setPiece('f', 8, self.makePiece(ChessPiece.BLACK_BISHOP, 'f', 8))
        self.setPiece('g', 8, self.makePiece(ChessPiece.BLACK_KNIGHT, 'g', 8))
        self.setPiece('h', 8, self.makePiece(ChessPiece.BLACK_ROOK, 'h', 8))

    def makePiece(self, val, column, rank):
        """
        Helper function to generate ChessPiece messages.
        """
        p = ChessPiece()
        p.header.frame_id = "chess_board"
        p.pose.position.x = SQUARE_SIZE * (0.5 + self.getColIdx(column))
        p.pose.position.y = SQUARE_SIZE * (0.5 + rank - 1)
        p.pose.position.z = 0.03
        p.type = val
        return p

    def copyPiece(self, val, copy):
        p = self.makePiece(val, 0, 1)
        p.pose = copy.pose
        return p

    def setPiece(self, column, rank, piece):
        """
        Set the value of a piece on the board. The piece
        should be a chess_msgs/ChessPiece, which has a
        pose and type, or None

        Column: 0 or 'a' = column A
        Rank:   1 = rank 1
        """
        try:
            self.values[int(int(rank)-1)*8+self.getColIdx(column)] = piece
        except:
            print column, rank
            rospy.loginfo("setPiece: invalid row/column")

    def getPiece(self, column, rank):
        try:
            return self.values[int(rank-1)*8+self.getColIdx(column)]
        except:
            return None
    def getPieceType(self, column, rank):
        try:
            return self.values[int(rank-1)*8+self.getColIdx(column)].type
        except:
            return 0

    def printBoard(self):
        """ Print board state to screen. """
        if self.side == self.WHITE or self.side == None:
            for r in [8,7,6,5,4,3,2,1]:
                for c in 'abcdefgh':
                    p = self.getPiece(c,r)    # print a8 first
                    if p == None:
                        print " ",
                    else:
                        print self.getPieceName(p.type),
                print ""
        else:
            for r in [1,2,3,4,5,6,7,8]:
                for c in 'hgfedcba':
                    p = self.getPiece(c,r)    # print h1 first
                    if p == None:
                        print " ",
                    else:
                        print self.getPieceName(p.type),
                print ""

    def revert(self):
        self.values = self.previous[0]
        self.last_move = self.previous[1]

    def copyType(self, col_f, row_f, col_t, row_t, board):
        fr = self.getPiece(col_f, row_f)
        to = board.getPiece(col_t, row_t)
        to.type = fr.type
        board.setPiece(col_t,row_t,to)

    def applyMove(self, move, pose=None):
        """ Update the board state, given a move from GNU chess. """
        (col_f, rank_f) = self.toPosition(move[0:2])
        (col_t, rank_t) = self.toPosition(move[2:])
        piece = self.getPiece(col_f, rank_f)
        piece.pose = pose
        self.setPiece(col_t, rank_t, piece)
        self.setPiece(col_f, rank_f, None)
        if move in castling_extras.keys():
            self.applyMove(castling_extras[move])

    def computeSide(self):
        """ Determine which side of the board we are on. """
        side = 0
        for c in 'abcdefgh':
            side += self.getPieceType(c,1)
            side += self.getPieceType(c,2)
            side -= self.getPieceType(c,7)
            side -= self.getPieceType(c,8)
            rospy.loginfo("Computed side value of: %d" % side)
        if side > 0:
            self.side = self.WHITE   # good to go
        else:
            self.side = self.BLACK
        self.last_move = "go"

    def setupSide(self):
        # need to setup board
        temp_board = BoardState(self.side)
        for i in range(8):
            temp_board.setPiece(i, 2, self.copyPiece(ChessPiece.WHITE_PAWN, self.getPiece(7-i, 7)) )
            temp_board.setPiece(i, 7, self.copyPiece(ChessPiece.BLACK_PAWN, self.getPiece(7-i, 2)) )

        temp_board.setPiece('a', 1, self.copyPiece(ChessPiece.WHITE_ROOK, self.getPiece('h',8)) )
        temp_board.setPiece('b', 1, self.copyPiece(ChessPiece.WHITE_KNIGHT, self.getPiece('g',8)))
        temp_board.setPiece('c', 1, self.copyPiece(ChessPiece.WHITE_BISHOP, self.getPiece('f',8)))
        temp_board.setPiece('d', 1, self.copyPiece(ChessPiece.WHITE_QUEEN, self.getPiece('e',8)))
        temp_board.setPiece('e', 1, self.copyPiece(ChessPiece.WHITE_KING, self.getPiece('d',8)))
        temp_board.setPiece('f', 1, self.copyPiece(ChessPiece.WHITE_BISHOP, self.getPiece('c',8)))
        temp_board.setPiece('g', 1, self.copyPiece(ChessPiece.WHITE_KNIGHT, self.getPiece('b',8)))
        temp_board.setPiece('h', 1, self.copyPiece(ChessPiece.WHITE_ROOK, self.getPiece('a',8)))

        temp_board.setPiece('a', 8, self.copyPiece(ChessPiece.BLACK_ROOK, self.getPiece('h',1)) )
        temp_board.setPiece('b', 8, self.copyPiece(ChessPiece.BLACK_KNIGHT, self.getPiece('g',1)) )
        temp_board.setPiece('c', 8, self.copyPiece(ChessPiece.BLACK_BISHOP, self.getPiece('f',1)) )
        temp_board.setPiece('d', 8, self.copyPiece(ChessPiece.BLACK_QUEEN, self.getPiece('e',1)) )
        temp_board.setPiece('e', 8, self.copyPiece(ChessPiece.BLACK_KING, self.getPiece('d',1)) )
        temp_board.setPiece('f', 8, self.copyPiece(ChessPiece.BLACK_BISHOP, self.getPiece('c',1)) )
        temp_board.setPiece('g', 8, self.copyPiece(ChessPiece.BLACK_KNIGHT, self.getPiece('b',1)) )
        temp_board.setPiece('h', 8, self.copyPiece(ChessPiece.BLACK_ROOK, self.getPiece('a',1)) )

        self.values = temp_board.values
        self.printBoard()

    #######################################################
    # helpers
    def toPosition(self, pos):
        """ Get position for a string name like 'a1'. """
        return [ord(pos[0])-ord('a'), int(pos[1])]

    def getColName(self, col):
        """ Convert to column string name. """
        try:
            return chr(ord('a') + col)
        except:
            return col

    def getColIdx(self, col):
        """ Convert to column integer index. """
        try:
            return int(col)
        except:
            return ord(col)-ord('a')

    def valid(self, col, rank):
        """ Is a particular position valid? """
        return rank <= 8 and rank > 0 and self.getColIdx(col) < 8 and self.getColIdx(col) >= 0

    def getPieceName(self, piece_type):
        if piece_type == ChessPiece.WHITE_PAWN:
            return "P"
        elif piece_type == ChessPiece.WHITE_ROOK:
            return "R"
        elif piece_type == ChessPiece.WHITE_KNIGHT:
            return "N"
        elif piece_type == ChessPiece.WHITE_BISHOP:
            return "B"
        elif piece_type == ChessPiece.WHITE_QUEEN:
            return "Q"
        elif piece_type == ChessPiece.WHITE_KING:
            return "K"
        elif piece_type == ChessPiece.BLACK_PAWN:
            return "p"
        elif piece_type == ChessPiece.BLACK_ROOK:
            return "r"
        elif piece_type == ChessPiece.BLACK_KNIGHT:
            return "n"
        elif piece_type == ChessPiece.BLACK_BISHOP:
            return "b"
        elif piece_type == ChessPiece.BLACK_QUEEN:
            return "q"
        elif piece_type == ChessPiece.BLACK_KING:
            return "k"
        elif piece_type > 0:
            return "X"
        else:
            return "x"

    def getMoveText(self, move):
        print move
        (col_f, rank_f) = self.toPosition(move[0:2])
        (col_t, rank_t) = self.toPosition(move[2:])
        piece = self.getPiece(col_f, rank_f)
        if piece == None:
            piece = self.getPiece(col_t, rank_t)
            if piece == None:
                return ""
        text = ""
        name = self.getPieceName(piece.type)
        if name == "p" or name == "P":
            text += "pawn "
        elif name == "r" or name == "R":
            text += "rook "
        elif name == "n" or name == "N":
            text += "knight "
        elif name == "b" or name == "B":
            text += "bishop "
        elif name == "q" or name == "Q":
            text += "queen "
        elif name == "k" or name == "K":
            text += "king "
        text += "from " + move[0:2] + " to " + move[2:]
        return text

    def isCastling(self, piece_new, piece_color, piece_gone):
        """
        Are we castling? Returns one of:
            e1c1 - (also, rook a1d1)
            e1g1 - (also, rook h1f1)
            e8c8 - (also, rook a8d8)
            e8g8 - (also, rook h8f8)
        """
        if len(piece_color) > 0 or len(piece_new) != 2 or len(piece_gone) != 2:
            return None
        if piece_new[0][1] == 1 and piece_new[1][1] == 1 and piece_gone[0][1] == 1 and piece_gone[1][1] == 1:
            # possibly white castling?
            if piece_new[0][0] == "c" and piece_new[1][0] == "d" and piece_gone[0][0] == "a" and piece_gone[1][0] == "e":
                return "e1c1"
            elif piece_new[1][0] == "c" and piece_new[0][0] == "d" and piece_gone[1][0] == "a" and piece_gone[0][0] == "e":
                return "e1c1"
            elif piece_new[0][0] == "g" and piece_new[1][0] == "f" and piece_gone[0][0] == "h" and piece_gone[1][0] == "e":
                return "e1g1"
            elif piece_new[1][0] == "g" and piece_new[0][0] == "f" and piece_gone[1][0] == "h" and piece_gone[0][0] == "e":
                return "e1g1"
            else:
                return None
        elif piece_new[0][1] == 8 and piece_new[1][1] == 8 and piece_gone[0][1] == 8 and piece_gone[1][1] == 8:
            # possibly black castling?
            if piece_new[0][0] == "c" and piece_new[1][0] == "d" and piece_gone[0][0] == "a" and piece_gone[1][0] == "e":
                return "e8c8"
            elif piece_new[1][0] == "c" and piece_new[0][0] == "d" and piece_gone[1][0] == "a" and piece_gone[0][0] == "e":
                return "e8c8"
            elif piece_new[0][0] == "g" and piece_new[1][0] == "f" and piece_gone[0][0] == "h" and piece_gone[1][0] == "e":
                return "e8g8"
            elif piece_new[1][0] == "g" and piece_new[0][0] == "f" and piece_gone[1][0] == "h" and piece_gone[0][0] == "e":
                return "e8g8"
            else:
                return None
        else:
            return None

class BoardUpdater:
    def __init__(self, board, listener):
        self.board = board
        self.listener = listener
        self.up_to_date = False # meaning has changed, now tells whether message has been recieved

    def callback(self, message):
        """
        Update the board state, given a new ChessBoard message.
        """
        # no need to update if already up to date
        if self.up_to_date == True:
            return

        piece_gone  = list()    # locations moved from
        piece_new   = list()    # locations moved to
        piece_color = list()    # locations that have changed color

        # process ChessBoard message
        temp_board = BoardState(self.board.side)
        for piece in message.pieces:
            # get col, rank as "x0"
            if self.board.side == self.board.WHITE or self.board.side == None:
                col = self.board.getColName(int(piece.pose.position.x/SQUARE_SIZE))
                rank = int(piece.pose.position.y/SQUARE_SIZE) + 1
            else:
                col = self.board.getColName(7 - int(piece.pose.position.x/SQUARE_SIZE))
                rank = 8 - int(piece.pose.position.y/SQUARE_SIZE)
            if not self.board.valid(col, rank):
                print "invalid: ", col, rank
                continue

            # update temp board
            if temp_board.getPiece(col, rank) == None:
                p = self.board.getPiece(col, rank)
                if p == None and not self.board.side == None:
                    piece_new.append([col, rank, piece])
                    rospy.logdebug("Piece moved to: %s%s" % (col,str(rank)))
                temp_board.setPiece(col, rank, piece)

        # see how board has changed
        for col in 'abcdefgh':
            for rank in [1,2,3,4,5,6,7,8]:
                old = self.board.getPiece(col,rank)
                new = temp_board.getPiece(col,rank)
                if new == None and old != None:
                    # this piece is gone!
                    piece_gone.append([col, rank, old])
                    rospy.loginfo("Piece moved from: %s%s" % (col,str(rank)))
                elif old != None and new != None and new.type/abs(float(new.type)) != old.type/abs(float(old.type)):
                    # capture!
                    piece_color.append([col, rank, new])
                    rospy.logdebug("Piece captured: %s%s" % (col,str(rank)))
                elif old != None and new != None:
                    # boring, but update types!
                    new.type = old.type
                    temp_board.setPiece(col,rank,new)

        # plausibility test: there can only be one change or new piece
        if self.board.side == None:
            temp_board.printBoard()
            if len(piece_color) + len(piece_new) == 0 and len(piece_gone) == 0:
                rospy.loginfo("No side set, but we are probably white.")
                self.board.last_move = "none"
                return self.setBoard(temp_board)

            elif len(piece_color) >= 32 and len(piece_new) == 0:
                rospy.loginfo("No side set, but we are probably black.")
                self.board.last_move = "none"
                return self.setBoard(temp_board)

            else:
                rospy.logdebug("Try again, %d" % (len(piece_new) + len(piece_color)))
                self.board.last_move = "fail"
                return

        elif len(piece_new) + len(piece_color) != 1:
            # castling
            self.board.castling_move = self.board.isCastling(piece_new, piece_color, piece_gone)
            if self.board.castling_move != None:
                # castling
                rospy.loginfo("Castling, %s" % self.board.castling_move)

                m = self.board.castling_move
                print m
                #self.copyType(m[0], m[1], m[2], m[3], temp_board)
                to = ChessPiece()
                to.type = self.board.side * ChessPiece.BLACK_KING
                temp_board.setPiece(m[2],int(m[3]),to)

                m = castling_extras[m]
                print m
                #self.copyType(m[0], m[1], m[2], m[3], temp_board)
                to = ChessPiece()
                to.type = self.board.side * ChessPiece.BLACK_ROOK
                temp_board.setPiece(m[2],int(m[3]),to)
                self.board.previous = [self.board.values, self.board.last_move]
                self.board.last_move = self.board.castling_move
                return self.setBoard(temp_board)

            rospy.logdebug("Try again, %d" % (len(piece_new) + len(piece_color)))
            self.board.last_move = "fail"
            return

        # if our pieces are missing, fill them in (bishops!)
        candidates = list()
        for entry in piece_gone:
            (col, rank, piece) = entry
            if piece.type/abs(piece.type) == self.board.side:
                # fill in
                temp_board.setPiece(col, rank, self.board.getPiece(col,rank))
            else:
                candidates.append(entry)
        if len(candidates) == 0:
            rospy.loginfo("Try again, no candidates")
            self.board.last_move = "fail"
            return
        if len(candidates) > 1: # too many possibilities (added 3AM on Wednesday!)
            rospy.loginfo("Try again, too many candidates %d" % len(candidates))
            self.board.last_move = "fail"
            return

        # find the corresponding piece
        if len(piece_new) == 1:
            piece_to = piece_new[0]
        else:
            piece_to = piece_color[0]
        piece_fr = candidates[0]
        # update type
        self.board.copyType(piece_fr[0], piece_fr[1], piece_to[0], piece_to[1], temp_board)

        # set outputs
        self.board.previous = [self.board.values, self.board.last_move]
        self.board.last_move = piece_fr[0] + str(piece_fr[1]) + piece_to[0] + str(piece_to[1])
        return self.setBoard(temp_board)

    def setBoard(self, temp_board):
        # patch board
        self.board.values = temp_board.values
        #if self.board.output:
        #    temp_board.printBoard()
        self.up_to_date = True

###########################################################
# chess engine logic
class GnuChessEngine:
    """ Connection to a GNU chess engine. """

    def __init__(self):
        """
        Start a connection to GNU chess.
        """
        self.engine = pexpect.spawn('/usr/games/gnuchess -x')
        self.history = list()
        self.pawning = False
        #self.nextMove = self.nextMoveUser
        self.nextMove = self.nextMoveGNU

    def startNewGame(self):
        self.engine.sendline('new')
        self.history = list()

    def nextMoveGNU(self, move="go", board=None):
        """
        Give opponent's move, get back move to make.
            returns None if given an invalid move.
        """
        # get move
        if self.pawning:
            while not rospy.is_shutdown():
                rows = [2,3,4,5]
                piece = ChessPiece.WHITE_PAWN
                if board.side == board.BLACK:
                    rows = [7,6,5,4]
                    piece = ChessPiece.BLACK_PAWN
                for row in rows:
                    for col in ['a','b','c','d','e','f','g','h']:
                        p1 = board.getPiece(col,row)
                        if p1 != None and abs(p1.type) == piece:
                            p2 = board.getPiece(col,row+1)
                            if p2 == None:
                                # this is a candidate
                                m = col + str(row) + col + str(row+1)
                                self.history.append(m)
                                return m
        else:
            self.engine.sendline(move)
            if self.engine.expect(['My move is','Illegal move']) == 1:
                return None
            self.engine.expect('([a-h][1-8][a-h][1-8][RrNnBbQq(\r\n)])')
            m = self.engine.after.rstrip()
        self.history.append(m)
        return m

    def nextMoveUser(self, move="go", board=None):
        print "Please enter a move"
        return raw_input().rstrip()

    def startPawning(self):
        self.history = self.history[0:-1]
        self.pawning = True
        self.history.append("Now pawning.")

    def exit(self):
        print "game review:"
        for h in self.history:
            print h
        self.engine.sendline('exit')

class ChessArmPlanner:

    # This is a bit hacky, basically I'm making the "table" thick, and not adding individual chess pieces
    BOARD_THICKNESS = 0.1
    CHESS_BOARD_FRAME = 'chess_board'

    """ Chess-specific stuff """
    def __init__(self, listener = None):
        self._grasp = GraspingInterface(GROUP_NAME_ARM, GROUP_NAME_GRIPPER)
        self._obj = ObjectInterface(FIXED_FRAME)
        self._listener = listener
        if self._listener == None:
            self._listener = TransformListener()
        self._move = ArmInterface(GROUP_NAME_ARM, FIXED_FRAME, self._listener)
        self.success = True

    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, pose):
        t = JointTrajectory()
        t.joint_names = gripper_joint_names
        tp = JointTrajectoryPoint()
        tp.positions = [pose/2.0 for j in t.joint_names]
        tp.effort = gripper_effort
        t.points.append(tp)
        return t

    def make_gripper_translation(self, min_dist, desired, axis=1.0):
        g = GripperTranslation()
        g.direction.vector.x = axis
        g.direction.header.frame_id = GRIPPER_FRAME
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

    def make_grasps(self, pose_stamped):
        # setup defaults of grasp
        g = Grasp()
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
        g.pre_grasp_approach = self.make_gripper_translation(0.1, 0.15)
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, -1.0)
        g.grasp_pose = pose_stamped

        # generate list of grasps
        grasps = []
        for y in [-1.57, -0.78, 0, 0.78, 1.57]:
            for p in [0, 0.2, -0.2, 0.4, -0.4]:
                q = quaternion_from_euler(0, 1.57-p, y)
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]
                g.id = str(len(grasps))
                g.grasp_quality = 1.0 - abs(p/2.0)
                grasps.append(copy.deepcopy(g))
        return grasps

    def make_places(self, pose_stamped):
        # setup default of place location
        l = PlaceLocation()
        l.post_place_posture = self.make_gripper_posture(GRIPPER_OPEN)
        l.pre_place_approach = self.make_gripper_translation(0.1, 0.15)
        l.post_place_retreat = self.make_gripper_translation(0.1, 0.15, -1.0)
        l.place_pose = pose_stamped

        # generate list of place locations
        places = []
        for y in [-1.57, -0.78, 0, 0.78, 1.57]:
            for p in [0, 0.2, -0.2, 0.4, -0.4]:
                q = quaternion_from_euler(0, p, y)  # now in object frame
                l.place_pose.pose.orientation.x = q[0]
                l.place_pose.pose.orientation.y = q[1]
                l.place_pose.pose.orientation.z = q[2]
                l.place_pose.pose.orientation.w = q[3]
                l.id = str(len(places))
                places.append(copy.deepcopy(l))
        return places

    def move_piece(self, start_pose, end_pose):
        # update table
        rospy.loginfo('Updating table position')
        self._obj.remove('table')
        p = PoseStamped()
        # back date header.stamp as perception can be slow
        p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
        p.header.frame_id = self.CHESS_BOARD_FRAME
        p.pose.position.x = SQUARE_SIZE * 4
        p.pose.position.y = SQUARE_SIZE * 4
        p.pose.position.z = 0
        p.pose.orientation.x = p.pose.orientation.y = p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        pt = self._listener.transformPose(FIXED_FRAME, p)
        self._obj.addBox('table', SQUARE_SIZE * 8, SQUARE_SIZE * 8, self.BOARD_THICKNESS,
                         pt.pose.position.x, pt.pose.position.y, pt.pose.position.z, wait=False)
        # insert piece
        rospy.loginfo('Updating piece position')
        p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
        p.pose.position.x = start_pose.position.x
        p.pose.position.y = start_pose.position.y
        p.pose.position.z = 0.0375 #0.045
        pt = self._listener.transformPose(FIXED_FRAME, p)
        self._obj.addCube('piece', 0.015, pt.pose.position.x, pt.pose.position.y, pt.pose.position.z, wait=False)
        self._obj.waitForSync()

        # pick it up
        grasps = self.make_grasps(pt)
        while True:
            result = self._grasp.pickup('piece', grasps)
            if result == MoveItErrorCodes.SUCCESS:
                rospy.loginfo('Pick succeeded')
                break
            elif result == MoveItErrorCodes.PLANNING_FAILED:
                rospy.logerr('Pick failed in the planning stage, try again...')
                rospy.sleep(0.5)  # short sleep to try and let state settle a bit?
                continue
            elif result == MoveItErrorCodes.CONTROL_FAILED or \
                 result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                rospy.logerr('Pick failed during execution, attempting to cleanup.')
                if 'piece' in self._obj.getKnownAttachedObjects():
                    rospy.loginfo('Pick managed to grab piece, retreat must have failed, continuing anyways')
                    break
                else:
                    rospy.loginfo('Pick did not grab piece, try again...')
                    continue
            else:
                rospy.logerr('Pick failed with error code: %d.' % result)
                return False

        # put it down
        rospy.loginfo('Placing piece')
        p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
        p.pose.position.x = end_pose.position.x
        p.pose.position.y = end_pose.position.y
        p.pose.position.z = 0.04 #0.045
        pt = self._listener.transformPose(FIXED_FRAME, p)

        places = self.make_places(pt)
        while True:
            result = self._grasp.place('piece', places, goal_is_eef = True)
            if result == MoveItErrorCodes.SUCCESS:
                rospy.loginfo('Place succeeded')
                break
            elif result == MoveItErrorCodes.PLANNING_FAILED:
                rospy.logerr('Place failed in the planning stage, try again...')
                rospy.sleep(0.5)  # short sleep to try and let state settle a bit?
                continue
            elif result == MoveItErrorCodes.CONTROL_FAILED or \
                 result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                rospy.logerr('Place failed during execution, attempting to cleanup.')
                if 'piece' in self._obj.getKnownAttachedObjects():
                    rospy.loginfo('Place did not place object, approach must have failed, will retry...')
                    continue
                else:
                    rospy.loginfo('Object no longer in gripper, must be placed, continuing...')
                    break
            else:
                rospy.logerr('Place failed with error code: %d.' % result)
                return False

        return True

    def execute(self, move, board):
        """ Execute a move. """

        # get info about move
        (col_f, rank_f) = board.toPosition(move[0:2])
        (col_t, rank_t) = board.toPosition(move[2:])
        fr = board.getPiece(col_f, rank_f)
        to = board.getPiece(col_t, rank_t)

        # is this a capture?
        if to != None:
            off_board = ChessPiece()
            off_board.header.frame_id = fr.header.frame_id
            off_board.pose.position.x = OFF_BOARD_X
            off_board.pose.position.y = OFF_BOARD_Y
            off_board.pose.position.z = OFF_BOARD_Z
            if not self.move_piece(to.pose, off_board.pose):
                rospy.logerr('Failed to move captured piece')
                self.success = False
                self.tuck()
                return None

        to = ChessPiece()
        to.header.frame_id = fr.header.frame_id
        to.pose = self.getPose(col_t, rank_t, board, fr.pose.position.z)
        if not self.move_piece(fr.pose, to.pose):
            rospy.logerr('Failed to move my piece')
            self.success = False
            self.tuck()
            return None

        if move in castling_extras:
            if not self.execute(castling_extras[move],board):
                rospy.logerr('Failed to carry out castling extra')

        self.tuck()
        return to.pose

    def getPose(self, col, rank, board, z=0):
        """ Find the reach required to get to a position """
        p = Pose()
        if board.side == board.WHITE:
            p.position.x = (col * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.y = ((rank-1) * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.z = z
        else:
            p.position.x = ((7-col) * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.y = ((8-rank) * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.z = z
        return p

    def tuck(self):
        if joints_tucked:
            self._move.moveToJointPosition(joint_names, joints_tucked)
        else:
            self._move.moveToJointPosition(joint_names, joints_ready)

    def untuck(self):
        if joints_untucked:
            self._move.moveToJointPosition(joint_names, joints_untucked)

