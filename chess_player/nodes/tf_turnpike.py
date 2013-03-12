#!/usr/bin/env python

""" 
  Copyright (c) 2011 Michael E. Ferguson.
  All right reserved.

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

from std_srvs.srv import *

import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class tf_turnpike:
    """ Captures a tf transform, republishes a modifed version """

    def __init__(self):
        self.listener = tf.TransformListener()
        self.translation = None 
        self.rotation = None 
        self.lookup() 

        rospy.Service('~trigger', Empty, self.callback)

    def lookup(self):
        try: 
            (self.translation, self.rotation) = self.listener.lookupTransform('odom', 'chess_board_raw', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to transform")

    def callback(self, req):
        self.lookup()
        return EmptyResponse()

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
        

if __name__=="__main__":
    rospy.init_node("tf_turnpike")
    turnpike = tf_turnpike()
    turnpike.run()

