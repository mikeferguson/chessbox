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
import actionlib
import sys

from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import *

class HeadEngine:   # a crazy name, but matches our convention
    
    def __init__(self, client=None):
        self.joints = ["head_pan_joint", "head_tilt_joint"] 
        self.last = [None, None] 
        self.temps = [0.0, 0.0]
        self.previous_tilt = None
        self.previous_pan = None
        self.iter = 0

        if client != None:
            self._client = client
        else:
            self._client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self._client.wait_for_server()

        rospy.Subscriber('joint_states', JointState, self.stateCb)
        rospy.Subscriber('diagnostics', DiagnosticArray, self.diagnosticCb)

        # setup home positions
        while self.last[0] == None:
            pass
        self.home_pan = 0.0
        self.home_tilt = 1.1 #1.05 #0.9561 #self.last[1]
    
    def stateCb(self, msg):
        """ Callback for JointState message. """
        try:
            indexes = [msg.name.index(name) for name in self.joints]
        except ValueError as val:
            rospy.logerr('/joint_stats.name is invalid.')
            return
        self.last = [ msg.position[k] for k in indexes ]

    def diagnosticCb(self, msg):
        """ ... """
        for entry in msg.status:
            name = entry.name.replace("Joint ", "")
            if name in self.joints:
                i = self.joints.index(name)
                for kv in entry.values:
                    if kv.key == "Temperature":
                        self.temps[i] = kv.value
        
    #######################################################
    # look at person/board
    def look_at_player(self):
        msg = JointTrajectory()
        msg.joint_names = self.joints
        msg.points = list()
    
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start = rospy.Duration(3.0)
        msg.points.append(point)

        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = msg
        self._client.send_goal(goal)
        rospy.sleep(3.0)

    def look_at_board(self):
        msg = JointTrajectory()
        msg.joint_names = self.joints
        msg.points = list()
    
        point = JointTrajectoryPoint()
        point.positions = [0.0, self.home_tilt]
        point.time_from_start = rospy.Duration(3.0)
        msg.points.append(point)

        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = msg
        self._client.send_goal(goal)

    def wiggle_head(self):
        self.iter = (self.iter+1)%5

        msg = JointTrajectory()
        msg.joint_names = self.joints
        msg.points = list()
    
        point = JointTrajectoryPoint()
        point.positions = [self.home_pan+(self.iter-2)*0.05, self.home_tilt]
        point.time_from_start = rospy.Duration(0.5)
        msg.points.append(point)

        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = msg
        self._client.send_goal(goal)
    
if __name__=="__main__":
    rospy.init_node("head_util_test")
    h = HeadEngine()
    
    h.look_at_player()
    rospy.sleep(5.0)
    h.look_at_board()
    rospy.sleep(5.0)
    
    for i in range(10):
        h.wiggle_head()
        rospy.sleep(2.0)


