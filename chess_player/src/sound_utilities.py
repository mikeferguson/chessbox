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

import socket, sys 
import subprocess
from threading import Thread

class SpeechEngine:

    def __init__(self, port=1314, host="localhost"):
        #print "Note, please start a server:"
        #print "  festival --server"
        self._host = host
        self._port = port
        self.conn = False
        #self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
    def open(self):
        try:
            self._sock.connect((self._host, self._port))
            self.conn = True
        except:
            print "Cannot find Festival Server!"
        
    def close(self):
        if self.conn:
            self._sock.close()
        
    def send(self, cmd):
        if self.conn:
            self._sock.send(cmd)
    
    def voice(self, name):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.open()
        self.send('(voice_%s)'%name)
        self.close()
        return "OK"

    def recv(self):
        if self.conn:
            data = self._sock.recv()
            return data
    
    def say(self, text):
        """ say an utterance, this will block until complete. """
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.open()
        self.send('(SayText "%s")'%text)
        self._sock.recv(50)
        self.close()
        return "OK"

class MPlayer(Thread):
    """ Plays a sound file, in a different thread. """
    
    def __init__(self, name):
        Thread.__init__ (self)
        self.name = name
        self.start()

    def run(self):
        subprocess.call(["mplayer", self.name])


if __name__ == "__main__":
    import time
    #MPlayer("../clips/openings/centuryfox.wav")
    #for i in range(10):
    #    print "hello"
    #    time.sleep(1)
    se = SpeechEngine()
    #se.say("Moving knight from e6 to d4")
    se.say("ha ha ha")

