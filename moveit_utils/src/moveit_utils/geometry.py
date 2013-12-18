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

# Helpers for dealing with geometry_msgs and transformations in Python

from geometry_msgs.msg import *
from tf.transformations import *

## @brief Get a translation matrix from a geometry_msgs/Point
## @param point geometry_msgs/Point to turn into matrix
def matrix_from_point_msg(point):
    return translation_matrix((point.x, point.y, point.z))

## @brief Get a rotation matrix from a geometry_msgs/Quaternion
## @param quaternion geometry_msgs/Quaternion to turn into matrix
def matrix_from_quaternion_msg(quaternion):
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    return quaternion_matrix(q)

## @brief Get a transformation matrix from a geometry_msgs/Pose
## @param pose geometry_msgs/Pose to turn into matrix
def matrix_from_pose_msg(pose):
    t = matrix_from_point_msg(pose.position)
    r = matrix_from_quaternion_msg(pose.orientation)
    return concatenate_matrices(t, r)

## @brief Get a geometry_msgs/Point from a transformation matrix
## @param transformation The matrix to convert to a point
def point_msg_from_matrix(transformation):
    msg = Point()
    msg.x = transformation[0][3]
    msg.y = transformation[1][3]
    msg.z = transformation[2][3]
    return msg

## @brief Get a geometry_msgs/Quaternion from a transformation matrix
## @param transformation The matrix to convert to a quaternion
def quaternion_msg_from_matrix(transformation):
    q = quaternion_from_matrix(transformation)
    msg = Quaternion()
    msg.x = q[0]
    msg.y = q[1]
    msg.z = q[2]
    msg.w = q[3]
    return msg

## @brief Get a geometry_msgs/Pose from a transformation matrix
## @param transformation The matrix to convert to a pose
def pose_msg_from_matrix(transformation):
    msg = Pose()
    msg.position = point_msg_from_matrix(transformation)
    msg.orientation = quaternion_msg_from_matrix(transformation)
    return p

## @brief Translate a geometry_msgs/Pose
## @param pose The pose to translate
## @param x The displacement in X coordinate axis
## @param y The displacement in Y coordinate axis
## @param z The displacement in Z coordinate axis
def translate_pose_msg(pose, x, y, z):
    initial = matrix_from_pose_msg(pose)
    transform = translation_matrix((x,y,z))
    return pose_from_matrix(concatenate_matrices(initial, transform))

## @brief Rotate a geometry_msgs/Pose
## @param pose The pose to rotate
## @param r The roll
## @param p The pitch
## @param y The yaw
def rotate_pose_msg_by_euler_angles(pose, r, p, y):
    initial = matrix_from_pose_msg(pose)
    transform = quaternion_matrix(quaternion_from_euler(r, p, y))
    return pose_msg_from_matrix(concatenate_matrices(initial, transform))

