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

import roslib; roslib.load_manifest('chess_player')
import rospy    # for logging
import pexpect  # for connecting to gnu chess
import threading

from chess_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped

import math
import tf
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
            self.setPiece(i, 2, self.makePiece(ChessPiece.WHITE_PAWN))
            self.setPiece(i, 7, self.makePiece(ChessPiece.BLACK_PAWN))

        self.setPiece('a', 1, self.makePiece(ChessPiece.WHITE_ROOK))
        self.setPiece('b', 1, self.makePiece(ChessPiece.WHITE_KNIGHT))
        self.setPiece('c', 1, self.makePiece(ChessPiece.WHITE_BISHOP))
        self.setPiece('d', 1, self.makePiece(ChessPiece.WHITE_QUEEN))
        self.setPiece('e', 1, self.makePiece(ChessPiece.WHITE_KING))
        self.setPiece('f', 1, self.makePiece(ChessPiece.WHITE_BISHOP))
        self.setPiece('g', 1, self.makePiece(ChessPiece.WHITE_KNIGHT))
        self.setPiece('h', 1, self.makePiece(ChessPiece.WHITE_ROOK))

        self.setPiece('a', 8, self.makePiece(ChessPiece.BLACK_ROOK))
        self.setPiece('b', 8, self.makePiece(ChessPiece.BLACK_KNIGHT))
        self.setPiece('c', 8, self.makePiece(ChessPiece.BLACK_BISHOP))
        self.setPiece('d', 8, self.makePiece(ChessPiece.BLACK_QUEEN))
        self.setPiece('e', 8, self.makePiece(ChessPiece.BLACK_KING))
        self.setPiece('f', 8, self.makePiece(ChessPiece.BLACK_BISHOP))
        self.setPiece('g', 8, self.makePiece(ChessPiece.BLACK_KNIGHT))
        self.setPiece('h', 8, self.makePiece(ChessPiece.BLACK_ROOK))            

    def makePiece(self, val, copy=None):
        """ 
        Helper function to generate ChessPiece messages.
        """
        p = ChessPiece()
        p.header.frame_id = "chess_board"
        if copy != None:
            p.pose = copy.pose
        p.type = val
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
            # need to setup board 
            temp_board = BoardState(self.side) 
            for i in range(8):
                temp_board.setPiece(i, 2, self.makePiece(ChessPiece.WHITE_PAWN, self.getPiece(7-i, 7)) )
                temp_board.setPiece(i, 7, self.makePiece(ChessPiece.BLACK_PAWN, self.getPiece(7-i, 2)) )

            temp_board.setPiece('a', 1, self.makePiece(ChessPiece.WHITE_ROOK, self.getPiece('h',8)) )
            temp_board.setPiece('b', 1, self.makePiece(ChessPiece.WHITE_KNIGHT, self.getPiece('g',8)))
            temp_board.setPiece('c', 1, self.makePiece(ChessPiece.WHITE_BISHOP, self.getPiece('f',8)))
            temp_board.setPiece('d', 1, self.makePiece(ChessPiece.WHITE_QUEEN, self.getPiece('e',8)))
            temp_board.setPiece('e', 1, self.makePiece(ChessPiece.WHITE_KING, self.getPiece('d',8)))
            temp_board.setPiece('f', 1, self.makePiece(ChessPiece.WHITE_BISHOP, self.getPiece('c',8)))
            temp_board.setPiece('g', 1, self.makePiece(ChessPiece.WHITE_KNIGHT, self.getPiece('b',8)))
            temp_board.setPiece('h', 1, self.makePiece(ChessPiece.WHITE_ROOK, self.getPiece('a',8)))

            temp_board.setPiece('a', 8, self.makePiece(ChessPiece.BLACK_ROOK, self.getPiece('h',1)) )
            temp_board.setPiece('b', 8, self.makePiece(ChessPiece.BLACK_KNIGHT, self.getPiece('g',1)) )
            temp_board.setPiece('c', 8, self.makePiece(ChessPiece.BLACK_BISHOP, self.getPiece('f',1)) )
            temp_board.setPiece('d', 8, self.makePiece(ChessPiece.BLACK_QUEEN, self.getPiece('e',1)) )
            temp_board.setPiece('e', 8, self.makePiece(ChessPiece.BLACK_KING, self.getPiece('d',1)) )
            temp_board.setPiece('f', 8, self.makePiece(ChessPiece.BLACK_BISHOP, self.getPiece('c',1)) )
            temp_board.setPiece('g', 8, self.makePiece(ChessPiece.BLACK_KNIGHT, self.getPiece('b',1)) )
            temp_board.setPiece('h', 8, self.makePiece(ChessPiece.BLACK_ROOK, self.getPiece('a',1)) ) 

            self.values = temp_board.values
            self.printBoard()

        self.last_move = "go"                

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
                


class BoardUpdater(threading.Thread):

    def __init__(self, board, listener):
        threading.Thread.__init__(self)
        self.board = board
        self.listener = listener
        self.translation = None 
        self.rotation = None
        self.up_to_date = True
        self.start()
        
    def callback(self, message):
        """ 
        Update the board state, given a new ChessBoard message.
        """
        rospy.sleep(0.1)
        self.listener.mutex.acquire()
        piece_gone  = list()    # locations moved from
        piece_new   = list()    # locations moved to
        piece_color = list()    # locations that have changed color
    
        # transform pieces
        pieces_transformed = list()
        for piece in message.pieces:
            ps = PoseStamped()
            ps.header.frame_id = piece.header.frame_id
            ps.pose = piece.pose
            pose = self.listener.transformPose("chess_board_raw", ps)
            #print ps, pose
            piece.pose = pose.pose
            piece.header.frame_id = "chess_board_raw"
            #col = self.board.getColName(int(piece.pose.position.x/SQUARE_SIZE))
            #rank = int(piece.pose.position.y/SQUARE_SIZE) + 1
            #print col,rank
        
        #print message.pieces

        # iterate through ways board could be inproperly localized
        for x_off in [0,-1,1]:
            for y_off in [0,-1,1]:
                # process ChessBoard message
                temp_board = BoardState(self.board.side)
                for piece in message.pieces:
                    # get col, rank as "x0"
                    if self.board.side == self.board.WHITE or self.board.side == None:
                        col = self.board.getColName(int(piece.pose.position.x/SQUARE_SIZE + x_off))
                        rank = int(piece.pose.position.y/SQUARE_SIZE + y_off) + 1
                    else:
                        col = self.board.getColName(7 - int(piece.pose.position.x/SQUARE_SIZE + x_off))
                        rank = 8 - int(piece.pose.position.y/SQUARE_SIZE + y_off)
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
                        return self.setWithOffset(x_off, y_off, temp_board)

                    elif len(piece_color) >= 32 and len(piece_new) == 0:
                        rospy.loginfo("No side set, but we are probably black.")
                        self.board.last_move = "none"
                        return self.setWithOffset(x_off, y_off, temp_board)

                    else:
                        rospy.logdebug("Try again, %d" % (len(piece_new) + len(piece_color)))        
                        self.board.last_move = "fail"
                        self.up_to_date = True
                        self.listener.mutex.release()
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
                        return self.setWithOffset(x_off, y_off, temp_board)

                    rospy.logdebug("Try again, %d" % (len(piece_new) + len(piece_color))) 
                    self.board.last_move = "fail"
                    self.up_to_date = True
                    self.listener.mutex.release()
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
                    self.up_to_date = True
                    self.listener.mutex.release()
                    return
                if len(candidates) > 1: # too many possibilities (added 3AM on Wednesday!)
                    rospy.loginfo("Try again, too many candidates %d" % len(candidates))        
                    self.board.last_move = "fail"
                    self.up_to_date = True
                    self.listener.mutex.release()
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
                return self.setWithOffset(x_off,y_off,temp_board)


    def setWithOffset(self, x, y, temp_board):        
        # patch board
        self.board.values = temp_board.values
        for piece in self.board.values:
            if piece != None:
                piece.pose.position.x += x*SQUARE_SIZE
                piece.pose.position.y += y*SQUARE_SIZE
                piece.header.frame_id = "chess_board"

        # look up board location
        try: 
            (translation, rotation) = self.listener.lookupTransform('odom', 'chess_board_raw', rospy.Time(0))
        except:
            rospy.logerr("Failed to transform")
            return
        ps = PoseStamped()
        ps.header.frame_id = "chess_board_raw"
        ps.pose.position.x = x*SQUARE_SIZE
        ps.pose.position.y = y*SQUARE_SIZE
        pose = self.listener.transformPose("odom", ps)
        self.translation = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        self.rotation = rotation

        if True: #self.board.output: 
            temp_board.printBoard()
            #self.board.output = False

        self.listener.mutex.release()
        self.up_to_date = True
        return

    def run(self):
        br = tf.TransformBroadcaster()
        while not rospy.is_shutdown():
            if self.translation != None:
                br.sendTransform( self.translation,
                                  self.rotation,
                                  rospy.Time.now(),
                                  "chess_board",
                                  "odom" )   

            rospy.sleep(0.1)        


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
                for row in [2,3,4,5]:
                    for col in ['a','b','c','d','e','f','g','h']:
                        p1 = board.getPiece(col,row)
                        if p1 != None and abs(p1.type) == ChessPiece.WHITE_PAWN:
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

