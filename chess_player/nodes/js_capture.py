#!/usr/bin/env python

""" 
  Capture the arm state.
  Copyright (c) 2011-2021 Michael E. Ferguson.  All right reserved.

  This program is free software; you can redistribute it and/or modifyr
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

import rospy
from sensor_msgs.msg import JointState

servos = ['arm_lift_joint', 'arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint', 'arm_wrist_roll_joint']
positions = [0.0 for s in servos]

def callback(msg):
    global servos, positions
    idx = [msg.name.index(servo) for servo in servos]
    positions = [msg.position[i] for i in idx]

if __name__ == '__main__':
    rospy.init_node('js_capture')
    rospy.Subscriber('joint_states', JointState, callback)
    
    while not rospy.is_shutdown():
        input()
        print(positions)
