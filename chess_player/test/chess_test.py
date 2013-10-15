#!/usr/bin/env python

"""
Test the chess grasping code
"""

import rospy
from tf.listener import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from chess_player.robot_defs import *
from chess_player.chess_utilities import *

from moveit_utils.arm_interface import ArmInterface
from moveit_utils.grasping_interface import GraspingInterface
from moveit_utils.object_interface import ObjectInterface


def make_simple_grasp(pose_stamped, name="test"):
    g = Grasp()
    g.id = name

    t = JointTrajectory()
    t.joint_names = ['left_gripper_joint', 'right_gripper_joint']
    tp = JointTrajectoryPoint()
    tp.positions = [0.01, 0.01]
    tp.effort = [28.0, 28.0]
    t.points.append(tp)
    g.pre_grasp_posture = t

    t = JointTrajectory()
    t.joint_names = ['left_gripper_joint', 'right_gripper_joint']
    tp = JointTrajectoryPoint()
    tp.positions = [0.0, 0.0]
    tp.effort = [28.0, 28.0]
    t.points.append(tp)
    g.grasp_posture = t

    g.grasp_pose = pose_stamped
    g.grasp_quality = 1.0

    gt = GripperTranslation()
    gt.direction.vector.x = 1.0
    gt.direction.header.frame_id = 'gripper_link'
    gt.min_distance = 0.1
    gt.desired_distance = 0.15
    g.pre_grasp_approach = gt

    gt = GripperTranslation()
    gt.direction.vector.x = -1.0
    gt.direction.header.frame_id = 'gripper_link'
    gt.min_distance = 0.1
    gt.desired_distance = 0.15
    g.post_grasp_retreat = gt

    return g

def make_simple_place(pose_stamped, name="test"):
    l = PlaceLocation()
    l.id = name

    l.place_pose = pose_stamped

    t = JointTrajectory()
    t.joint_names = ['left_gripper_joint', 'right_gripper_joint']
    tp = JointTrajectoryPoint()
    tp.positions = [0.0125, 0.0125] # max of 0.045
    tp.effort = [28.0, 28.0]
    t.points.append(tp)
    l.post_place_posture = t

    gt = GripperTranslation()
    gt.direction.vector.x = 1.0
    gt.direction.header.frame_id = 'gripper_link'
    gt.min_distance = 0.1
    gt.desired_distance = 0.15
    l.pre_place_approach = gt

    gt = GripperTranslation()
    gt.direction.vector.x = -1.0
    gt.direction.header.frame_id = 'gripper_link'
    gt.min_distance = 0.1
    gt.desired_distance = 0.15
    l.post_place_retreat = gt

    return l

if __name__=='__main__':
    rospy.init_node('test_chess_grasping')
    grasp = GraspingInterface(GROUP_NAME_ARM, GROUP_NAME_GRIPPER)
    obj = ObjectInterface(FIXED_FRAME)
    listener = TransformListener()
    move = ArmInterface('arm', FIXED_FRAME, listener)

    # need time for listener to get data
    rospy.sleep(3.0)

    #move.moveToJointPosition(joint_names, joints_ready)

    ####################################################
    # this code will display the grasp pose array
    if 0:
        pub = rospy.Publisher('grasp_poses', PoseArray)
        pa = PoseArray()
        pa.header.frame_id = FIXED_FRAME
        rospy.sleep(3.0)
        p = PoseStamped()
        p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
        p.header.frame_id = 'chess_board'
        p.pose.position.x = SQUARE_SIZE * (0.5 + 4)
        p.pose.position.y = SQUARE_SIZE * (0.5 + 1)
        p.pose.position.z = 0.03
        q = quaternion_from_euler(0.0, 1.57, 0.0)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        p_transformed = listener.transformPose(FIXED_FRAME, p)
        for g in getGrasps(p_transformed):
            pa.header.stamp = rospy.Time.now()
            pa.poses.append(copy.deepcopy(g.grasp_pose.pose))
            print("adding pose")
            print(g.grasp_pose.pose)
        pa.header.stamp = rospy.Time.now()
        pub.publish(pa)

    ####################################################
    # this was the original testing code
    p = PoseStamped()
    p.header.frame_id = 'chess_board'
    q = quaternion_from_euler(0.0, 0, 0)
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]

    if 1:
        # remove old pieces/table if any
        obj.remove('part', wait=False)
        obj.remove('table', wait=False)
        for y in [0,1,6,7]:
            for x in range(8):
                obj.remove(chr(97+x)+str(y+1), wait=False)
        obj.waitForSync()

    if 1:
        # add table
        p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
        p.pose.position.x = SQUARE_SIZE * 4
        p.pose.position.y = SQUARE_SIZE * 4
        p.pose.position.z = -0.075
        p_transformed = listener.transformPose(FIXED_FRAME, p)

        obj.addBox('table', 0.05715 * 8, 0.05715 * 8, .15, p_transformed.pose.position.x, p_transformed.pose.position.y, p_transformed.pose.position.z)
        p.pose.position.z = 0

    if 1:
        # add pieces
        for y in [0,1,6,7]:
            for x in range(8):
                p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
                p.pose.position.x = SQUARE_SIZE*(0.5+x)
                p.pose.position.y = SQUARE_SIZE*(0.5+y)
                p.pose.position.z = 0.03
                p_transformed = listener.transformPose(FIXED_FRAME, p)
                print(chr(97+x)+str(y+1))
                obj.addCube(chr(97+x)+str(y+1), 0.015, p_transformed.pose.position.x, p_transformed.pose.position.y, p_transformed.pose.position.z, wait=False)
        obj.waitForSync()

    if 1:
        import time
        t = time.time()

        # move all pawns forward
        i = 0
        for col in 'abcdefgh':
            print('grasping '+col+'2')

            # transform grasp to fixed frame and form grasp message
            p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
            p.pose.position.x = SQUARE_SIZE * (0.5 + i)
            p.pose.position.y = SQUARE_SIZE * (0.5 + 1)
            p.pose.position.z = 0.03
            q = quaternion_from_euler(0.0, 1.77, 0.78)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            p_transformed = listener.transformPose(FIXED_FRAME, p)
            g = make_simple_grasp(p_transformed)

            if not grasp.pickup(col+"2", [g, ]):
                # retry?
                if not grasp.pickup(col+"2", [g, ]):
                    break
                    continue
            print("sleep before place")
            rospy.sleep(2.0)

            # transform place to fixed frame and form place message
            p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
            p.pose.position.y = SQUARE_SIZE * (0.5 + 1) + SQUARE_SIZE*2
            q = quaternion_from_euler(0.0, 1.77, 0.0)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            place_transformed = listener.transformPose(FIXED_FRAME, p)
            place = make_simple_place(place_transformed)

            if not grasp.place(col+"2", [place,], goal_is_eef = True):
                print("failed to place, replacing piece where it started")
                place = make_simple_place(p_transformed)
                if not grasp.place(col+"2", [place,], goal_is_eef = True):
                    break

            move.moveToJointPosition(joint_names, joints_ready)
            rospy.sleep(1)
            i+=1

        print("elapsed time: ", time.time() - t)

        exit(0)
        # move back row forward
        i = 0
        for col in 'abcdefgh':
            print(col+"1")
            # manipulate a part
            p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
            p.pose.position.x = SQUARE_SIZE * (0.5 + i)
            p.pose.position.y = SQUARE_SIZE * (0.5)
            p_transformed = listener.transformPose(FIXED_FRAME, p)

            if not pick.pickup(col+"1", p_transformed):
                # retry?
                if not pick.pickup(col+"1", p_transformed):
                    continue
            rospy.sleep(1.0)

            p_transformed.pose.position.x += SQUARE_SIZE*2
            if not place.place(col+"1", p_transformed):
                print("failed to place, replacing piece where it started")
                p_transformed.pose.position.x -= SQUARE_SIZE*2
                place.place(col+"1", p_transformed)

            move.moveToJointPosition(joint_names, joints_ready)
            rospy.sleep(1)
            i+=1
        print("elapsed time: ", time.time() - t)

        i = 0
        for col in 'abcdefgh':
            print(col+"2")
            # manipulate a part
            p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
            p.pose.position.x = SQUARE_SIZE * (0.5 + i)
            p.pose.position.y = SQUARE_SIZE * (0.5 + 3)
            p_transformed = listener.transformPose(FIXED_FRAME, p)

            if not pick.pickup(col+"2", p_transformed):
                # retry?
                if not pick.pickup(col+"2", p_transformed):
                    continue
            rospy.sleep(1.0)

            p_transformed.pose.position.y += SQUARE_SIZE*2
            if not place.place(col+"2", p_transformed):
                print("failed to place, replacing piece where it started")
                p_transformed.pose.position.y -= SQUARE_SIZE*2
                place.place(col+"2", p_transformed)

            move.moveToJointPosition(joint_names, joints_ready)
            rospy.sleep(1)
            i+=1
        print("elapsed time: ", time.time() - t)

