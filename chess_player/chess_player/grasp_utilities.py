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

import copy, math
import rclpy

from chess_player.robot_defs import *
from chess_player.chess_utilities import castling_extras

from moveit_python import MoveGroupInterface, PickPlaceInterface, PlanningSceneInterface

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf2_ros.broadcaster import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf.transformations import quaternion_from_euler

from threading import Thread


class ChessArmPlanner(Thread):

    CHESS_BOARD_FRAME = 'chess_board'

    """ Chess-specific stuff """
    def __init__(self, node, buffer=None):
        Thread.__init__(self)
        self._grasp = PickPlaceInterface(GROUP_NAME_ARM, GROUP_NAME_GRIPPER)
        self._obj = PlanningSceneInterface(FIXED_FRAME)
        self._buffer = buffer
        if self._buffer == None:
            self._buffer = Buffer()
            self._listener = TransformListener(self.buffer, node)
        self._broadcaster = TransformBroadcaster()
        self._move = MoveGroupInterface(GROUP_NAME_ARM, FIXED_FRAME, self._buffer)
        self.success = True
        self.transform = None

    def run(self):
        while rclpy.ok():
            if self.transform != None:
                translation = [self.transform.transform.translation.x, \
                               self.transform.transform.translation.y, \
                               self.transform.transform.translation.z]
                rotation    = [self.transform.transform.rotation.x, \
                               self.transform.transform.rotation.y, \
                               self.transform.transform.rotation.z, \
                               self.transform.transform.rotation.w]
                self._broadcaster.sendTransform(translation,
                                                rotation,
                                                rospy.Time.now(),
                                                self.CHESS_BOARD_FRAME,
                                                "base_link")
            rospy.sleep(0.1)

    def transform_pose(self, pose):
        if self.transform:
            # TODO transform manually
            return self._buffer.transform(pose, FIXED_FRAME)
        else:
            return self._buffer.transform(pose, FIXED_FRAME)

    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, pose):
        t = JointTrajectory()
        t.joint_names = gripper_joint_names
        tp = JointTrajectoryPoint()
        tp.positions = [pose/2.0 for j in t.joint_names]
        tp.effort = gripper_effort
        t.points.append(tp)
        return t

    def make_gripper_translation(self, min_dist, desired, axis=1.0):
        g = GripperTranslation()
        g.direction.vector.x = axis
        g.direction.header.frame_id = GRIPPER_FRAME
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

    def make_grasps(self, pose_stamped, mega_angle=False):
        # setup defaults of grasp
        g = Grasp()
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
        g.pre_grasp_approach = self.make_gripper_translation(0.1, 0.15)
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, -1.0)
        g.grasp_pose = pose_stamped

        pitch_vals = [0, 0.2, -0.2, 0.4, -0.4]
        if mega_angle:
            pitch_vals += [0.3, -0.3, 0.5, -0.5, 0.6, -0.6]

        # generate list of grasps
        grasps = []
        for y in [-1.57, -0.78, 0, 0.78, 1.57]:
            for p in pitch_vals:
                q = quaternion_from_euler(0, 1.57-p, y)
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]
                g.id = str(len(grasps))
                g.grasp_quality = 1.0 - abs(p/2.0)
                grasps.append(copy.deepcopy(g))
        return grasps

    def make_places(self, pose_stamped, mega_angle=False):
        # setup default of place location
        l = PlaceLocation()
        l.post_place_posture = self.make_gripper_posture(GRIPPER_OPEN)
        l.pre_place_approach = self.make_gripper_translation(0.1, 0.15)
        l.post_place_retreat = self.make_gripper_translation(0.1, 0.15, -1.0)
        l.place_pose = pose_stamped

        pitch_vals = [0, 0.2, -0.2, 0.4, -0.4]
        if mega_angle:
            pitch_vals += [0.3, -0.3, 0.5, -0.5, 0.6, -0.6]

        # generate list of place locations
        places = []
        for y in [-1.57, -0.78, 0, 0.78, 1.57]:
            for p in pitch_vals:
                q = quaternion_from_euler(0, p, y)  # now in object frame
                l.place_pose.pose.orientation.x = q[0]
                l.place_pose.pose.orientation.y = q[1]
                l.place_pose.pose.orientation.z = q[2]
                l.place_pose.pose.orientation.w = q[3]
                l.id = str(len(places))
                places.append(copy.deepcopy(l))
        return places

    def update_objects(self, board):
        # update table position
        self._obj.removeCollisionObject('table')
        p = PoseStamped()
        p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
        p.header.frame_id = self.CHESS_BOARD_FRAME
        p.pose.position.x = SQUARE_SIZE * 4
        p.pose.position.y = SQUARE_SIZE * 4
        p.pose.position.z = 0
        p.pose.orientation.x = p.pose.orientation.y = p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        pt = self.transform_pose(p)

        thickness = pt.pose.position.z
        pt.pose.position.x = 0.255 + .375
        pt.pose.position.z = pt.pose.position.z/2.0
        self._obj.addBox('table', 0.75, 1.5, thickness,
                         pt.pose.position.x, pt.pose.position.y, pt.pose.position.z)
        self._obj.setColor('table', 223.0/256.0, 90.0/256.0, 12.0/256.0)

        # update piece positions
        for r in [1,2,3,4,5,6,7,8]:
            for c in 'abcdefgh':
                p = board.getPiece(c,r)
                if p != None:
                    # insert this piece
                    height = board.getPieceHeight(p.type)
                    radius = 0.015
                    ps = PoseStamped()
                    ps.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
                    ps.header.frame_id = self.CHESS_BOARD_FRAME
                    ps.pose.position.x = p.pose.position.x
                    ps.pose.position.y = p.pose.position.y
                    ps.pose.position.z = height/2.0
                    ps.pose.orientation.x = p.pose.orientation.y = p.pose.orientation.z = 0.0
                    ps.pose.orientation.w = 1.0
                    pt = self.transform_pose(ps)

                    self._obj.addCylinder(board.getPieceId(p), height, radius, \
                                          pt.pose.position.x, pt.pose.position.y, pt.pose.position.z)
                    if p.type < 0:
                        self._obj.setColor(board.getPieceId(p), 0, 0, 0)
                    else:
                        self._obj.setColor(board.getPieceId(p), 0.8, 0.8, 0.8)

        self._obj.waitForSync()
        self._obj.sendColors()
        rospy.loginfo('Done updating objects')

    def move_piece(self, name, start_pose, end_pose):
        rospy.loginfo('Moving %s' % name)
        # pick it up
        grasps = self.make_grasps(start_pose)
        success, _ = self._grasp.pick_with_retry(name, grasps, 10, support_name="table")
        if not success:
            grasps = self.make_grasps(start_pose, True)  # regen grasps with wider angles
            success, _ = self._grasp.pick_with_retry(name, grasps, 10, support_name="table")
            if not success:
                return False

        # put it down
        rospy.loginfo('Placing %s' % name)
        places = self.make_places(end_pose)
        success, _ = self._grasp.place_with_retry(name, places, 10, support_name="table")
        if not success:
            places = self.make_places(end_pose, True)  # regen places with wider angles
            success, _ = self._grasp.place_with_retry(name, places, 10, support_name="table")
            if not success:
                return False

    def execute(self, move, board):
        """ Execute a move. """

        self.update_objects(board)

        # get info about move
        (col_f, rank_f) = board.toPosition(move[0:2])
        (col_t, rank_t) = board.toPosition(move[2:])
        fr_piece = board.getPiece(col_f, rank_f)
        to_piece = board.getPiece(col_t, rank_t)

        # get name of piece
        fr_id = board.getPieceId(fr_piece)

        # transform
        fr = PoseStamped()
        fr.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
        fr.header.frame_id = "chess_board"
        fr.pose = fr_piece.pose
        fr.pose.position.z = board.getPieceHeight(fr_piece.type)
        fr = self.transform_pose(fr)

        # is this a capture?
        if to_piece != None:
            # get name of piece
            to_id = board.getPieceId(to_piece)
            print('Capturing', to_id)

            to = PoseStamped()
            to.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
            to.header.frame_id = "chess_board"
            to.pose = to_piece.pose
            to.pose.position.z = board.getPieceHeight(to_piece.type)
            to = self.transform_pose(to)

            # get name of piece
            to_id = board.getPieceId(to_piece)

            off_board = PoseStamped()
            off_board.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
            off_board.header.frame_id = "chess_board"
            off_board.pose.position.x = OFF_BOARD_X
            off_board.pose.position.y = OFF_BOARD_Y
            off_board.pose.position.z = OFF_BOARD_Z
            off_board = self.transform_pose(off_board)

            if not self.move_piece(to_id, to, off_board):
                rospy.logerr('Failed to move captured piece')
                self.success = False
                self.tuck()
                return None

            # remove from planning scene
            self._obj.removeCollisionObject(to_id)

        to = PoseStamped()
        to.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
        to.header.frame_id = "chess_board"
        height = board.getPieceHeight(fr_piece.type)/2.0 + 0.0075  # object-centric use half height plus small margin
        to.pose = self.getPose(col_t, rank_t, board, height)
        to = self.transform_pose(to)

        if not self.move_piece(fr_id, fr, to):
            rospy.logerr('Failed to move %s' % move[0:2])
            self.success = False
            self.tuck()
            return None

        if move in castling_extras:
            if not self.execute(castling_extras[move],board):
                rospy.logerr('Failed to carry out castling extra')

        self.tuck()
        return to.pose

    def getPose(self, col, rank, board, z=0):
        """ Find the reach required to get to a position """
        p = Pose()
        if board.side == board.WHITE:
            p.position.x = (col * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.y = ((rank-1) * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.z = z
        else:
            p.position.x = ((7-col) * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.y = ((8-rank) * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.z = z
        return p

    def tuck(self):
        if joints_tucked:
            self._move.moveToJointPosition(joint_names, joints_tucked)
        else:
            self._move.moveToJointPosition(joint_names, joints_ready)

    def untuck(self):
        if joints_untucked:
            self._move.moveToJointPosition(joint_names, joints_untucked)
