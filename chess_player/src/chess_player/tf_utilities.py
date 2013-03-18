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
import rospy
import threading

from tf.listener import *

class TransformListenerThread(threading.Thread):
    def __init__(self, tl):
        threading.Thread.__init__(self)
        self.tl = tl
        self.mutex = threading.Lock()

    def run(self):
        rospy.Subscriber("/tf", tfMessage, self.transformlistener_callback)
        self.tl.frame_graph_server = rospy.Service('~tf_frames', FrameGraph, self.frame_graph_service)
        rospy.spin()

    def transformlistener_callback(self, data):
        self.mutex.acquire()
        who = data._connection_header.get('callerid', "default_authority")
        for transform in data.transforms:
            self.tl.setTransform(transform, who)
        self.mutex.release()

    def frame_graph_service(self, req):
        return FrameGraphResponse(self.tl.allFramesAsDot()) 

class SyncListener(TransformerROS):
    def __init__(self, *args):
        TransformerROS.__init__(self, *args)
        thr = TransformListenerThread(self)
        thr.setDaemon(True)
        thr.start()
        self.setUsingDedicatedThread(True)
        self.mutex = thr.mutex

