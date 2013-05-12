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

from __future__ import print_function

import thread, copy
import rospy
import actionlib

from tf.listener import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PickupAction, PickupGoal, PlaceAction, PlaceGoal
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from manipulation_msgs.msg import Grasp, GripperTranslation, PlaceLocation

TRANSLATION_FRAME = 'arm_wrist_roll_link'

class GraspManager:
    def __init__(self, group, ee):
        self.group = group
        self.effector = ee

    def get_grasps(self, pose_stamped):
        g = Grasp()
        g.id = 'name'
        g.pre_grasp_posture = self.get_gripper_posture(0.05)
        g.grasp_posture = self.get_gripper_posture(0.01) # was 0.0075
        g.grasp_pose = pose_stamped
        g.grasp_quality = 1.0
        g.approach = self.get_gripper_translation(0.05, 0.15)
        g.retreat = self.get_gripper_translation(0.05, 0.15, -1.0)
        #g.max_contact_force
        #g.allowed_touch_objects[]
        return [g]

    def get_place_locations(self, pose_stamped):
        l = PlaceLocation()
        l.id = 'name2'
        l.place_pose = pose_stamped
        l.approach = self.get_gripper_translation(0.05, 0.15)
        l.retreat = self.get_gripper_translation(0.05, 0.15, -1.0)
        l.post_place_posture = self.get_gripper_posture(0.05)
        return [l]

    def get_gripper_posture(self, pose):
        """ This is Maxwell-specific. """
        js = JointState()
        js.name = ['l_gripper_joint', 'r_gripper_joint']
        js.position = [pose, pose]
        js.velocity = [0.0, 0.0]
        js.effort = [1.0, 1.0]
        return js

    def get_gripper_translation(self, min_dist, desired, axis=1.0):
        gt = GripperTranslation()
        gt.direction.vector.x = axis
        gt.direction.header.frame_id = TRANSLATION_FRAME
        gt.min_distance = min_dist
        gt.desired_distance = desired
        return gt

class PickupManager(GraspManager):
    def __init__(self, group, ee):
        GraspManager.__init__(self, group, ee)
        self.action = actionlib.SimpleActionClient('pickup', PickupAction)
        self.action.wait_for_server()

    def pickup(self, name, grasps):
        g = PickupGoal()
        g.target_name = name
        g.group_name = self.group
        g.end_effector = self.effector
        g.possible_grasps = grasps
        #g.support_surface_name = ??
        #g.allow_gripper_support_collision = ??
        g.attached_object_touch_links = list() # empty list = use all links of end-effector
                #["right_gripper_finger_link","right_gripper_finger_link"] 
        #g.path_constraints = ??
        #g.allowed_touch_objects = ['part']
        g.allowed_planning_time = 30.0
        #g.planning_options.planning_scene_diff = ??
        g.planning_options.plan_only = False
        self.action.send_goal(g)
        self.action.wait_for_result()
        print(self.action.get_result().error_code.val)

class PlaceManager(GraspManager):
    def __init__(self, group, ee):
        GraspManager.__init__(self, group, ee)
        self.action = actionlib.SimpleActionClient('place', PlaceAction)
        self.action.wait_for_server()

    def place(self, name, locations):
        g = PlaceGoal()
        g.group_name = self.group
        g.attached_object_name = name
        g.place_locations = locations
        #g.support_surface_name = ??
        #g.allow_gripper_support_collision = ??
        #g.path_constraints = ??
        #g.allowed_touch_objects = ['part']
        g.allowed_planning_time = 30.0
        #g.planning_options.planning_scene_diff = ??
        g.planning_options.plan_only = False
        self.action.send_goal(g)
        self.action.wait_for_result()
        print(self.action.get_result().error_code.val)
        

class ObjectManager:
    def __init__(self, frame):
        self._fixed_frame = frame

        # publisher to send objects
        self._pub = rospy.Publisher('collision_object', CollisionObject)
        self._attached_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject)

        # subscribe to planning scene, track the attached and collision objects
        self._mutex = thread.allocate_lock()
        self._attached = list()
        self._collision = list()
        rospy.Subscriber('move_group/monitored_planning_scene', PlanningScene, self.sceneCb)

    def addCube(self, name, size, x, y, z):
        o = CollisionObject()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = self._fixed_frame
        o.id = name

        s = SolidPrimitive()
        s.dimensions = [size, size, size]
        s.type = s.BOX
        o.primitives.append(s)

        ps = PoseStamped()
        ps.header.frame_id = self._fixed_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        o.primitive_poses.append(ps.pose)

        o.operation = o.ADD

        self._pub.publish(o)
        while not name in self.getKnownCollisionObjects():
            rospy.logdebug('Waiting for object to add')
            self._pub.publish(o)
            rospy.sleep(1.0)

    def remove(self, name):
        """ Remove a an object. """
        o = CollisionObject()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = self._fixed_frame
        o.id = name
        o.operation = o.REMOVE

        self._pub.publish(o)
        while name in self.getKnownCollisionObjects():
            rospy.logdebug('Waiting for object to remove')
            self._pub.publish(o)
            rospy.sleep(1.0)

    def sceneCb(self, msg):
        """ Recieve updates from move_group. """
        self._mutex.acquire()
        for obj in msg.world.collision_objects:
            try:
                if obj.operation == obj.ADD:
                    self._collision.append(obj.id)
                    rospy.logdebug('ObjectManager: Added Collision Object "%s"' % obj.id)
                elif obj.operation == obj.REMOVE:
                    self._collision.remove(obj.id)
                    rospy.logdebug('ObjectManager: Removed Collision Object "%s"' % obj.id)
            except ValueError:
                pass
        self._attached = list()
        for obj in msg.robot_state.attached_collision_objects:
            rospy.logdebug('ObjectManager: attached collision objects includes "%s"' % obj.object.id)
            self._attached.append(obj.object.id)
        self._mutex.release()

    def getKnownCollisionObjects(self):
        self._mutex.acquire()
        l = copy.deepcopy(self._collision)
        self._mutex.release()
        return l

    def getKnownAttachedObjects(self):
        self._mutex.acquire()
        l = copy.deepcopy(self._attached)
        self._mutex.release()
        return l


if __name__=='__main__':
    rospy.init_node('grasp_utilities')
    pick = PickupManager('Arm', 'Gripper')
    place = PlaceManager('Arm', 'Gripper')
    obj = ObjectManager('base_link')
    listener = TransformListener()
    rospy.sleep(3.0)

    obj.remove('part')
    print('Removed part')

    p = PoseStamped()
    p.header.stamp = rospy.Time.now() - rospy.Duration(1.0)
    p.header.frame_id = 'chess_board'
    p.pose.position.x = .15 #0.05715*(0.5+2)
    p.pose.position.y = .10 #0.05715*(0.5)
    p.pose.position.z = -0.3 #0.05
    q = quaternion_from_euler(0.0, 1.57, 0)
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    p_transformed = listener.transformPose('base_link', p)

    obj.addCube('part', 0.01, p_transformed.pose.position.x, p_transformed.pose.position.y, p_transformed.pose.position.z)
    print('Added part')

    g = pick.get_grasps(p_transformed)
    pick.pickup('part', g)
    rospy.sleep(1.0)

    p_transformed.pose.position.y += 0.05
    #p_transformed.pose.position.x -= 0.025
    #p_transformed.pose.position.z += 0.1
    l = place.get_place_locations(p_transformed)
    place.place('part', l)
            


    
