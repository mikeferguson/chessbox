""" 
  Copyright (c) 2011-2024 Michael E. Ferguson. All right reserved.

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

import rclpy
from rclpy.duration import Duration
from simple_actions import SimpleActionClient
import sys

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from chess_player.robot_defs import head_joint_names, head_pose_look_at_board, head_pose_look_at_player

class HeadEngine:   # a crazy name, but matches our convention

    def __init__(self, node, client=None):
        self.node = node
        self.joints = head_joint_names
        self.iter = 0

        if client != None:
            self._client = client
        else:
            self._client = SimpleActionClient(node, FollowJointTrajectory, 'head_controller/follow_joint_trajectory')

    #######################################################
    # look at person/board
    def look_at_player(self):
        msg = JointTrajectory()
        msg.joint_names = self.joints
        msg.points = list()
    
        point = JointTrajectoryPoint()
        point.positions = head_pose_look_at_player
        point.velocities = [0.0 for j in self.joints]
        point.time_from_start = Duration(seconds=3).to_msg()
        msg.points.append(point)

        msg.header.stamp = self.node.get_clock().now().to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg
        result = self._client(goal)

    def look_at_board(self):
        msg = JointTrajectory()
        msg.joint_names = self.joints
        msg.points = list()

        point = JointTrajectoryPoint()
        point.positions = head_pose_look_at_board
        point.velocities = [0.0 for j in self.joints]
        point.time_from_start = Duration(seconds=3).to_msg()
        msg.points.append(point)

        msg.header.stamp = self.node.get_clock().now().to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg
        result = self._client(goal)

    def wiggle_head(self):
        """ We always wiggle the first joint """
        self.iter = (self.iter + 1) % 5

        msg = JointTrajectory()
        msg.joint_names = self.joints
        msg.points = list()

        point = JointTrajectoryPoint()
        point.positions = head_pose_look_at_board
        point.positions[0] += (self.iter - 2) * 0.05
        point.velocities = [0.0 for j in self.joints]
        point.time_from_start = Duration(seconds=1).to_msg()
        msg.points.append(point)

        msg.header.stamp = self.node.get_clock().now().to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg
        result = self._client(goal)

