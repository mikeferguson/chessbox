#!/usr/bin/env python

""" 
  Copyright (c) 2013 Michael E. Ferguson. All right reserved.

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

#from __future__ import print_function

import rospy
import actionlib

from tf.listener import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import *

#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from control_msgs.msg import *

from chess_utilities import SQUARE_SIZE, castling_extras

# tucked position
servos = ['arm_lift_joint', 'arm_shoulder_pan_joint', 'arm_upperarm_roll_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint', 'arm_wrist_roll_joint']
tucked  = [0.0, -1.57, 0.0, -1.7, 1.7, 1.6157930965728755, -0.066472500808377785]

# gripper
GRIPPER_OPEN = 0.05
GRIPPER_CLOSE = 0.0075

class ChessMoveIt:
    fixed_frame = "base_link"
    board_frame = "base_link" #"chess_board"

    def __init__(self):
        self.listener = TransformListener()
        self.object_publisher = rospy.Publisher('collision_object', CollisionObject)
        self.gripper_publisher = rospy.Publisher('gripper_controller/command', Float64)

        self.moveit = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        self.moveit.wait_for_server()

    # build each trajectory
    # execute once all trajectories are known

    def execute(self, move, board, nested=False):
        """ Execute a move. """

        # untuck arm
        #self.tuck_server.untuck()
        #rospy.sleep(3.0)

        # get info about move
        (col_f, rank_f) = board.toPosition(move[0:2])
        (col_t, rank_t) = board.toPosition(move[2:])
        fr = board.getPiece(col_f, rank_f)
        to = board.getPiece(col_t, rank_t)

        goal = MoveArmGoal()
        goal.header.frame_id = fr.header.frame_id

        # is this a capture?
        if to != None:
            off_board = ChessPiece()
            off_board.header.frame_id = fr.header.frame_id
            off_board.pose.position.x = -2 * SQUARE_SIZE
            off_board.pose.position.y = SQUARE_SIZE
            off_board.pose.position.z = fr.pose.position.z
            self.addTransit(goal, to.pose, off_board.pose)

        to = ChessPiece()
        to.header.frame_id = fr.header.frame_id
        to.pose = self.getPose(col_t, rank_t, board, fr.pose.position.z)

        self.addTransit(goal, fr.pose, to.pose)

        # execute
        try:
            self.client.send_goal(goal)
            self.client.wait_for_result()
            print self.client.get_result()
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)

        if move in castling_extras:
            self.execute(castling_extras[move],board)

        if not nested:
            # tuck arm
            self.tuck_server.tuck()
            rospy.sleep(5.0)
        return to.pose

    def addChessboardObject(self):
        o = CollisionObject()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = self.fixed_frame
        o.id = self.board_frame

        s = SolidPrimitive()
        s.dimensions = [SQUARE_SIZE * 8, SQUARE_SIZE * 8, 0.10]
        s.type = s.BOX
        o.primitives.append(s)

        ps = PoseStamped()
        ps.header.frame_id = self.board_frame
        ps.pose.position.x = SQUARE_SIZE * 4 + .2
        ps.pose.position.y = SQUARE_SIZE * 4
        ps.pose.position.z = 0.05 + .65
        ps.pose.orientation.w = 1.0
        ps = self.listener.transformPose(self.fixed_frame, ps)
        o.primitive_poses.append(ps.pose)

        o.operation = o.ADD

        self.object_publisher.publish(o)
        
    def removeChessboardObject(self):
        p = rospy.Publisher('collision_object', CollisionObject)
        
        o = CollisionObject()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = self.fixed_frame
        o.id = self.board_frame
        o.operation = o.REMOVE

        rospy.sleep(3.0)
        self.object_publisher.publish(o)

    def constructMotionPlanRequestFromJointStates(self, joints, positions):
        m = MoveGroupGoal()

        # 1. fill in workspace_parameters
        # 2. fill in start_state
        # 3. fill in goal_constraints
        c1 = Constraints()

        c1.position_constraints.append(PositionConstraint())
        c1.position_constraints[0].header.frame_id = self.fixed_frame
        c1.position_constraints[0].link_name = "gripper_link"
        b = BoundingVolume()
        s = SolidPrimitive()
        s.dimensions = [0.0001]
        s.type = s.SPHERE
        b.primitives.append(s)
        b.primitive_poses.append(goal_transformed.pose)
        c1.position_constraints[0].constraint_region = b
        c1.position_constraints[0].weight = 1.0

        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.frame_id = self.fixed_frame
        c1.orientation_constraints[0].orientation = goal_transformed.pose.orientation
        c1.orientation_constraints[0].link_name = "gripper_link"
        c1.orientation_constraints[0].absolute_x_axis_tolerance = 1.0
        c1.orientation_constraints[0].absolute_y_axis_tolerance = 1.0
        c1.orientation_constraints[0].absolute_z_axis_tolerance = 0.5
        c1.orientation_constraints[0].weight = 1.0

        m.request.goal_constraints.append(c1)

        # 4. fill in path constraints
        # 5. fill in trajectory constraints
        # 6. fill in planner id
        # 7. fill in group name
        m.request.group_name = 'Arm'
        # 8. fill in number of planning attempts
        m.request.num_planning_attempts = 1
        # 9. fill in allowed planning time
        m.request.allowed_planning_time = 15.0

        # TODO: fill in
        # m.planning_options.planning_scene_diff.allowed_collision_matrix

        m.planning_options.plan_only = True #False
        m.planning_options.look_around = False
        m.planning_options.replan = False
        return m



    # goal is PoseStamped
    def constructMotionPlanRequest(self, goal_stamped):
        m = MoveGroupGoal()

        goal_transformed = self.listener.transformPose(self.fixed_frame, goal_stamped)
        print(goal_transformed)

        # 1. fill in workspace_parameters
        # 2. fill in start_state
        # 3. fill in goal_constraints
        c1 = Constraints()

        c1.position_constraints.append(PositionConstraint())
        c1.position_constraints[0].header.frame_id = self.fixed_frame
        c1.position_constraints[0].link_name = "gripper_link"
        b = BoundingVolume()
        s = SolidPrimitive()
        s.dimensions = [0.0001]
        s.type = s.SPHERE
        b.primitives.append(s)
        b.primitive_poses.append(goal_transformed.pose)
        c1.position_constraints[0].constraint_region = b
        c1.position_constraints[0].weight = 1.0

        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.frame_id = self.fixed_frame
        c1.orientation_constraints[0].orientation = goal_transformed.pose.orientation
        c1.orientation_constraints[0].link_name = "gripper_link"
        c1.orientation_constraints[0].absolute_x_axis_tolerance = 1.0
        c1.orientation_constraints[0].absolute_y_axis_tolerance = 1.0
        c1.orientation_constraints[0].absolute_z_axis_tolerance = 0.5
        c1.orientation_constraints[0].weight = 1.0

        m.request.goal_constraints.append(c1)

        # 4. fill in path constraints
        # 5. fill in trajectory constraints
        # 6. fill in planner id
        # 7. fill in group name
        m.request.group_name = 'Arm'
        # 8. fill in number of planning attempts
        m.request.num_planning_attempts = 1
        # 9. fill in allowed planning time
        m.request.allowed_planning_time = 15.0

        # TODO: fill in
        # m.planning_options.planning_scene_diff.allowed_collision_matrix

        m.planning_options.plan_only = False
        m.planning_options.look_around = False
        m.planning_options.replan = False
        return m
        
        
from chess_player.head_utilities import *

if __name__ == "__main__":
    rospy.init_node("moveit_utilities")
    h = HeadEngine()
    h.look_at_board()
    cmit = ChessMoveIt()
    rospy.sleep(2.0)
    #cmit.addChessboardObject()

    for y in range(8):
        for x in range(8):
            p = PoseStamped()
            p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
            p.header.frame_id = "chess_board"
            p.pose.position.x = 0.05715*(0.5+x)
            p.pose.position.y = 0.05715*(0.5+y)
            p.pose.position.z = 0.05
            q = quaternion_from_euler(0.0, 1.57, 0)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            
            req = cmit.constructMotionPlanRequest(p)
            try:
                cmit.moveit.send_goal(req)
                cmit.moveit.wait_for_result()
                print(cmit.moveit.get_result())
            except rospy.ServiceException, e:
               print("Service did not process request: %s"%str(e))

            rospy.sleep(2)


#    print("pick")
#    p.pose.position.z = 0.70 + 0.05
    
#    req = cmit.constructMotionPlanRequest(p)
#    try:
#        cmit.moveit.send_goal(req)
#        cmit.moveit.wait_for_result()
        #print(cmit.moveit.get_result())
#    except rospy.ServiceException, e:
#       print("Service did not process request: %s"%str(e))

#    rospy.sleep(12.0)
#    cmit.removeChessboardObject()
    print("OK")
