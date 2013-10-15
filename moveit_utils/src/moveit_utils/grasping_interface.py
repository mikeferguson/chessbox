"""
Interface to MoveIt! pick&place
"""

from __future__ import print_function

import rospy
import actionlib

from tf.listener import *

from geometry_msgs.msg import *
from moveit_msgs.msg import *
from shape_msgs.msg import *

## @brief Simple interface to pick and place actions
class GraspingInterface:

    ## @brief Create a grasp manager, connect actions
    ## @param group Name of arm planning group
    ## @param ee_group Name of end effector planning group
    ## @param plan_only Should we only plan, but not execute?
    def __init__(self, group = 'arm', ee_group = 'gripper', plan_only = False):
        self._group = group
        self._effector = ee_group
        self._pick_action = actionlib.SimpleActionClient('pickup', PickupAction)
        self._pick_action.wait_for_server()
        self._place_action = actionlib.SimpleActionClient('place', PlaceAction)
        self._place_action.wait_for_server()
        self._plan_only = plan_only

    ## @brief Plan and grasp something
    ## @param name Name of the object to grasp
    ## @param grasps Grasps to try (moveit_msgs/Grasp)
    ## @param support_name Name of the support surface
    def pickup(self, name, grasps, support_name = 'table',
               allow_gripper_support_collision = True,
               allowed_touch_objects = list()):
        """ This will try to pick up a chess piece. """
        g = PickupGoal()
        g.target_name = name
        g.group_name = self._group
        g.end_effector = self._effector
        g.possible_grasps = grasps
        g.support_surface_name = support_name
        g.allow_gripper_support_collision = allow_gripper_support_collision
        g.attached_object_touch_links = list() # empty list = use all links of end-effector
        #g.path_constraints = ??
        #g.planner_id = ??
        g.allowed_touch_objects = allowed_touch_objects
        g.allowed_planning_time = 30.0
        #g.planning_options.planning_scene_diff = ??
        g.planning_options.plan_only = self._plan_only
        self._pick_action.send_goal(g)
        self._pick_action.wait_for_result()
        if self._pick_action.get_result().error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Pick succeeded")
            return True
        rospy.loginfo("Pick failed")
        return False

    ## @brief Plan and grasp something
    ## @param name Name of the object to grasp
    ## @param grasps Grasps to try (moveit_msgs/Grasp)
    ## @param support_name Name of the support surface
    ## @param goal_is_eef Set to true if the place goal is for the
    ##        end effector frame, default is object frame.
    def place(self, name, locations, support_name = 'table',
              allow_gripper_support_collision = True,
              allowed_touch_objects = list(),
              goal_is_eef = False):
        g = PlaceGoal()
        g.group_name = self._group
        g.attached_object_name = name
        g.place_locations = locations
        g.place_eef = goal_is_eef
        g.support_surface_name = support_name
        g.allow_gripper_support_collision = allow_gripper_support_collision
        #g.path_constraints = ??
        #g.planner_id = ??
        g.allowed_touch_objects = allowed_touch_objects
        g.allowed_planning_time = 30.0
        #g.planning_options.planning_scene_diff = ??
        g.planning_options.plan_only = self._plan_only
        self._place_action.send_goal(g)
        self._place_action.wait_for_result()
        if self._place_action.get_result().error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Place succeeded")
            return True
        rospy.loginfo("Place failed")
        return False

