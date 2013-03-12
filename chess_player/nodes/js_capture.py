#!/usr/bin/env python

""" 
Capture the arm state.
"""

import roslib; roslib.load_manifest('chess_player')
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
        raw_input()
        print positions
