"""
An easy interface for managing objects in MoveIt!
"""

from __future__ import print_function

import thread, copy
import rospy

from geometry_msgs.msg import *
from moveit_msgs.msg import *
from moveit_msgs.srv import *
from shape_msgs.msg import *

# TODO: Add a 'addCylinder' function

## @brief A class for managing the state of the objects in the planning scene
## @param frame The fixed frame in which planning is being done (needs to be part of robot?)
## @param init_from_service Whether to initialize our list of objects by calling the service
##            NOTE: this requires that said service be in the move_group launch file, which
##            is not the default from the setup assistant.
class ObjectInterface:
    def __init__(self, frame, init_from_service = True):
        self._fixed_frame = frame

        # publisher to send objects to MoveIt
        self._pub = rospy.Publisher('collision_object', CollisionObject)
        self._attached_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject)

        # track the attached and collision objects
        self._mutex = thread.allocate_lock()
        self._attached = list()
        self._collision = list()
        self._objects = dict()

        # get the initial planning scene
        if init_from_service:
            rospy.loginfo('Waiting for get_planning_scene')
            rospy.wait_for_service('get_planning_scene')
            self._service = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)
            try:
                req = PlanningSceneComponents()
                req.components = PlanningSceneComponents.WORLD_OBJECT_NAMES + \
                                 PlanningSceneComponents.WORLD_OBJECT_GEOMETRY + \
                                 PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                scene = self._service(req)
                self.sceneCb(scene.scene)
            except rospy.ServiceException as e:
                print('Failed to get initial planning scene, results may be wonky: %s' % e)

        # subscribe to planning scene
        rospy.Subscriber('move_group/monitored_planning_scene', PlanningScene, self.sceneCb)

    ## @brief Insert a solid primitive into planning scene
    ## @param wait When true, we wait for planning scene to actually update,
    ##             this provides immunity against lost messages.
    def addSolidPrimitive(self, name, solid, pose, wait = True):
        o = CollisionObject()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = self._fixed_frame
        o.id = name
        o.primitives.append(solid)
        o.primitive_poses.append(pose)
        o.operation = o.ADD

        self._objects[name] = o

        self._pub.publish(o)
        while wait and not name in self.getKnownCollisionObjects():
            rospy.logdebug('Waiting for object to add')
            self._pub.publish(o)
            rospy.sleep(0.1)

    ## @brief Insert new box into planning scene
    ## @param wait When true, we wait for planning scene to actually update,
    ##             this provides immunity against lost messages.
    def addBox(self, name, size_x, size_y, size_z, x, y, z, wait = True):
        s = SolidPrimitive()
        s.dimensions = [size_x, size_y, size_z]
        s.type = s.BOX

        ps = PoseStamped()
        ps.header.frame_id = self._fixed_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0

        self.addSolidPrimitive(name, s, ps.pose, wait)

    ## @brief Insert new cube to planning scene
    ## @param wait When true, we wait for planning scene to actually update,
    ##             this provides immunity against lost messages.
    def addCube(self, name, size, x, y, z, wait = True):
        self.addBox(name, size, size, size, x, y, z, wait)

    ## @brief Send message to remove object
    ## @param wait When true, we wait for planning scene to actually update,
    ##             this provides immunity against lost messages.
    def remove(self, name, wait = True):
        """ Remove a an object. """
        o = CollisionObject()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = self._fixed_frame
        o.id = name
        o.operation = o.REMOVE

        try:
            del self._objects[name]
        except KeyError:
            pass

        self._pub.publish(o)
        while wait and name in self.getKnownCollisionObjects():
            rospy.logdebug('Waiting for object to remove')
            self._pub.publish(o)
            rospy.sleep(0.1)

    ## @brief Update the object lists from a PlanningScene message
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

    ## @brief Get a list of names of collision objects
    def getKnownCollisionObjects(self):
        self._mutex.acquire()
        l = copy.deepcopy(self._collision)
        self._mutex.release()
        return l

    ## @brief Get a list of names of attached objects
    def getKnownAttachedObjects(self):
        self._mutex.acquire()
        l = copy.deepcopy(self._attached)
        self._mutex.release()
        return l

    ## @brief Wait for sync
    def waitForSync(self, max_time = 10.0):
        sync = False
        t = rospy.Time.now()
        while not sync:
            sync = True
            # delete objects that should be gone
            for name in self._collision + self._attached:
                if name not in self._objects.keys():
                    # should be removed, is not
                    self.remove(name, False)
                    sync = False
            # add missing objects
            for name in self._objects.keys():
                if name not in self._collision + self._attached:
                    self._pub.publish(self._objects[name])
                    sync = False
            # timeout
            if rospy.Time.now() - t > rospy.Duration(max_time):
                rospy.logerr('ObjectManager: sync timed out.')
                break
            rospy.sleep(0.1)

