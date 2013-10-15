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




if __name__=='__main__':
    rospy.init_node('test_chess_grasping')
    grasp = GraspingInterface(GROUP_NAME_ARM, GROUP_NAME_GRIPPER)
    obj = ObjectInterface(FIXED_FRAME)
    listener = TransformListener()
    move = ArmInterface(GROUP_NAME_ARM, FIXED_FRAME, listener)

    planner = ChessArmPlanner()

    # need time for listener to get data
    rospy.sleep(3.0)

    move.moveToJointPosition(joint_names, joints_ready)

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
            p.pose.position.z = 0.0375
            p_transformed = listener.transformPose(FIXED_FRAME, p)
            grasps = planner.make_grasps(p_transformed)

            while True:
                result = grasp.pickup(col+"2", grasps)
                if result == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo('Pick succeeded')
                    break
                elif result == MoveItErrorCodes.PLANNING_FAILED:
                    rospy.logerr('Pick failed in the planning stage, try again...')
                    rospy.sleep(0.5)  # short sleep to try and let state settle a bit?
                    continue
                elif result == MoveItErrorCodes.CONTROL_FAILED or \
                     result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                    rospy.logerr('Pick failed during execution, attempting to cleanup.')
                    if col+"2" in obj.getKnownAttachedObjects():
                        rospy.loginfo('Pick managed to grab piece, retreat must have failed, continuing anyways')
                        break
                    else:
                        rospy.loginfo('Pick did not grab piece, try again...')
                        continue
                else:
                    rospy.logerr('Pick failed with error code: %d.' % result)
                    continue

            # transform place to fixed frame and form place message
            p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
            p.pose.position.y = SQUARE_SIZE * (0.5 + 3)
            p.pose.position.z = 0.04
            place_transformed = listener.transformPose(FIXED_FRAME, p)
            places = planner.make_places(place_transformed)

            while True:
                result = grasp.place(col+"2", places) #goal_is_eef = True)
                if result == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo('Place succeeded')
                    break
                elif result == MoveItErrorCodes.PLANNING_FAILED:
                    rospy.logerr('Place failed in the planning stage, try again...')
                    rospy.sleep(0.5)  # short sleep to try and let state settle a bit?
                    continue
                elif result == MoveItErrorCodes.CONTROL_FAILED or \
                     result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                    rospy.logerr('Place failed during execution, attempting to cleanup.')
                    if col+"2" in obj.getKnownAttachedObjects():
                        rospy.loginfo('Place did not place object, approach must have failed, will retry...')
                        continue
                    else:
                        rospy.loginfo('Object no longer in gripper, must be placed, continuing...')
                        break
                else:
                    rospy.logerr('Place failed with error code: %d.' % result)
                    continue

            # move arm to side to percieve a bit
            print("move arm to side")
            move.moveToJointPosition(joint_names, joints_ready)
            rospy.sleep(1)

            i+=1
