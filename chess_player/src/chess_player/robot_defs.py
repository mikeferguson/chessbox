#!/usr/bin/env python

""" 
  Simple executive for playing AAAI robot chess
  Copyright (c) 2011-2013 Michael E. Ferguson.  All right reserved.

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

#ROBOT_NAME = "maxwell"
ROBOT_NAME = "that_other_bot"

# TODO: load this from URDF?
if ROBOT_NAME == "maxwell":
    GROUP_NAME_ARM = 'arm'
    GROUP_NAME_GRIPPER = 'gripper'

    # The frame used for approach and retreat translations, gripper_link is local
    #   so approach/translation gets transformed by the grasp orientation
    GRIPPER_FRAME = 'gripper_link'

    # The frame that all objects/poses should be translated to, the frame in which
    #   moveit planning is done
    FIXED_FRAME = 'base_link'

    # This was previously 0.0075
    GRIPPER_CLOSED = 0.01
    GRIPPER_OPEN = 0.05

    # Tucking the arm requires a set of joint constraints
    joint_names = ['arm_lift_joint', 'arm_shoulder_pan_joint', 'arm_upperarm_roll_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint', 'arm_wrist_roll_joint']
    joints_tucked  = [0.0, -1.57, 0.0, -1.7, 1.7, 1.57, -0.066472500808377785]
    joints_untucked  = [0.0, 0.0, 0.0, -1.57, 1.57, 1.57, -0.066472500808377785]
    joints_ready = joints_tucked

    head_joint_names = ['head_pan_joint', 'head_tilt_joint']
    head_pose_look_at_board = [0.0, 1.15]
    head_pose_look_at_player = [0.0, 0.0]

elif ROBOT_NAME == "that_other_bot":
    GROUP_NAME_ARM = 'arm'
    GROUP_NAME_GRIPPER = 'gripper'

    # The frame used for approach and retreat translations, gripper_link is local
    #   so approach/translation gets transformed by the grasp orientation
    GRIPPER_FRAME = 'gripper_link'

    # The frame that all objects/poses should be translated to, the frame in which
    #   moveit planning is done
    FIXED_FRAME = 'base_link'

    # This was previously 0.0075
    GRIPPER_CLOSED = 0.015
    GRIPPER_OPEN = 0.05

    # Tucking the arm requires a set of joint constraints
    joint_names = ['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
    joints_tucked  = [0.0, -1.4486, 1.3439, 2.8327, -1.8119, 0.0, 1.6571, 0.0]
    joints_untucked  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joints_ready  = [0.2, -1.4486, 0, 1.68, -1.32, 0.0, 0.55, 1.38]

    head_joint_names = ['head_pan_joint', 'head_tilt_joint']
    head_pose_look_at_board = [0.0, -1.0]
    head_pose_look_at_player = [0.0, 0.0]
