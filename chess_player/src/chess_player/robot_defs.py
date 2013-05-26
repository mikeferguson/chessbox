#!/usr/bin/env python

""" 
  Simple executive for playing AAAI robot chess
  Copyright (c) 2011-2013 Michael E. Ferguson.  All right reserved.

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
