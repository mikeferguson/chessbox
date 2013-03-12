#!/usr/bin/env python

from chess_utilities import *

def board_to_msg(board):
    msg = ChessBoard()
    for row in [1,2,3,4,5,6,7,8]:
        for col in 'abcdefgh':
            p = board.getPiece(row, col)
            if p != None:
                msg.pieces.append(p)
    return msg


for side in [BoardState.WHITE, BoardState.BLACK]:   # test each side
    for missing in [0,1,2,3,4,5,6,7,8]:             # variable number of missing
        for extra in 
        b = BoardState
        b.side = side
        b.newGame()    


        

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
            self.values[int(rank-1)*8+self.getColIdx(column)] = piece
        except:
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

    def applyUpdate(self, message):
        """ 
        Update the board state, given a new ChessBoard message.
        """
        piece_gone  = list()    # locations moved from
        piece_new   = list()    # locations moved to
        piece_color = list()    # locations that have changed color

        # process ChessBoard message  
        temp_board = BoardState(self.side)  
        for piece in message.pieces:
            # get col, rank as "x0"
            if self.side == self.WHITE or self.side == None:
                col = self.getColName(int(piece.pose.position.x/SQUARE_SIZE))
                rank = int(piece.pose.position.y/SQUARE_SIZE) + 1
            else:
                col = self.getColName(7 - int(piece.pose.position.x/SQUARE_SIZE))
                rank = 8 - int(piece.pose.position.y/SQUARE_SIZE)
            if not self.valid(col, rank):
                continue
            # update temp board
            if temp_board.getPiece(col, rank) == None:
                p = self.getPiece(col, rank)
                if p == None and not self.side == None:
                    piece_new.append([col, rank, piece]) 
                    rospy.logdebug("Piece moved to: %s%s" % (col,str(rank)))
                temp_board.setPiece(col, rank, piece)

        # see how board has changed
        for col in 'abcdefgh':
            for rank in [1,2,3,4,5,6,7,8]:
                old = self.getPiece(col,rank)
                new = temp_board.getPiece(col,rank)                    
                if new == None and old != None:
                    # this piece is gone!
                    piece_gone.append([col, rank, old])
                    rospy.logdebug("Piece moved from: %s%s" % (col,str(rank)))
                elif old != None and new != None and new.type/abs(float(new.type)) != old.type/abs(float(old.type)):
                    # capture!
                    piece_color.append([col, rank, new])
                    rospy.logdebug("Piece captured: %s%s" % (col,str(rank))) 
                elif old != None and new != None:
                    # boring, but update types!
                    new.type = old.type
                    temp_board.setPiece(col,rank,new)

        # plausibility test: there can only be one change or new piece
        if self.side == None:
            if len(piece_change) + len(piece_new) == 0:
                rospy.loginfo("No side set, but we are probably white.")
                self.last_move = "none"
                self.values = temp_board.values
            elif len(piece_change) >= 32 and len(piece_new) == 0:
                rospy.loginfo("No side set, but we are probably black.")
                self.last_move = "none"
                self.values = temp_board.values
            else:
                rospy.logdebug("Try again, %d" % len(piece_new) + len(piece_change))        
                self.last_move = "fail"
        elif len(piece_new) + len(piece_change) != 1:
            rospy.logdebug("Try again, %d" % len(piece_new) + len(piece_change))        
            self.last_move = "fail"

        # if our pieces are missing, fill them in (bishops!)
        candidates = list()
        for entry in piece_gone:
            (col, rank, piece) = entry
            if piece.type/abs(piece.type) == self.side:
                # fill in
                self.temp_board.setPiece(col, rank, self.getPiece(col,rank))
            else:
                candidates.append(entry)
        if len(candidates) == 0:
            rospy.logdebug("Try again, no candidates")        
            self.last_move = "fail"
        rospy.loginfo(candidates)

        # find the corresponding piece
        if len(piece_new) == 1:
            piece_to = piece_new[0]
        else:
            piece_to = piece_change[0]
        piece_fr = candidates[0]
        # update type
        fr = self.getPiece(piece_fr[0], piece_fr[1])
        to = temp_board.getPiece(piece_to[0], piece_to[1])
        to.type = fr.type
        temp_board.setPiece(piece_to[0],piece_to[1],to)
               
        # set outputs
        if self.output: 
            temp_board.printBoard()
            self.output = False
        self.previous = [self.values, self.last_move] 
        self.last_move = piece_fr[0] + str(piece_fr[1]) + piece_to[0] + str(piece_to[1])
        self.values = temp_board.values
        self.up_to_date = True

    def revert(self):
        self.values = self.previous[0]
        self.last_move = self.previous[1]

    def applyMove(self, move, pose=None):
        """ Update the board state, given a move from GNU chess. """
        (col_f, rank_f) = self.toPosition(move[0:2])
        (col_t, rank_t) = self.toPosition(move[2:])
        piece = self.getPiece(col_f, rank_f)
        piece.pose = pose
        self.setPiece(col_t, rank_t, piece) 
        self.setPiece(col_f, rank_f, None)       

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

                
class GnuChessEngine:
    """ Connection to a GNU chess engine. """

    def __init__(self):
        """
        Start a connection to GNU chess.
        """ 
        self.engine = pexpect.spawn('/usr/games/gnuchess -x')
        self.history = list()
        self.pawning = False

    def startNewGame(self):
        self.engine.sendline('new')
        self.history = list()

    def nextMove(self, move="go", board=None):
        """
        Give opponent's move, get back move to make. 
            returns None if given an invalid move.
        """
        m = None
        if self.pawning:
            while not rospy.is_shutdown():
                for col in ['a','b','c','d','e','f','g','h']:
                    for row in [2,3,4,5]:
                        piece = board.getPiece(col,row)
                        if 
                                    
                        
        else:
            self.engine.sendline(move)        
            if self.engine.expect(['My move is','Illegal move']) == 1:
                return None     
            self.engine.expect('([a-h][1-8][a-h][1-8][RrNnBbQq(\r\n)])')
            m = self.engine.after.rstrip()
        self.history.append(m)
        return m

    def startPawning(self):
        self.pawning = True
        self.history.append("Now pawning.")

    def exit(self):
        print "game review:"
        for h in self.history:
            print h
        self.engine.sendline('exit')

