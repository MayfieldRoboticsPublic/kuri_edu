import os.path

import rospy
from tf.transformations import euler_from_quaternion

import mayfield_msgs.msg
import mayfield_utils
import may_nav_py.nav

import geometry_msgs.msg

from .map_manager import MapManager
from .power_monitor import PowerMonitor


class NavController(object):
    '''
    The nav controller is responsible for loading saved maps.  This
    class will look in the home directory for a simlink to a map created
    with kuri_mapping.launch and attempt to load it and start localization
    '''

    MAP_PATH = os.path.expanduser("~/map.map_capnp")

    def __init__(self):

        self._pub = rospy.Publisher(
            "node_online",
            mayfield_msgs.msg.NodeStatus,
            latch=True,
            queue_size=1,
        )

        # Controllers need to be loaded before we can run
        mayfield_utils.wait_for_nodes(
            node_names=['controllers'],
        )

        self._map_manager = MapManager()
        self._power_monitor = None  # Created when we start to run

        self._nav_client = may_nav_py.nav.Nav(
            on_nav_cb=lambda: None,
            on_nav_end_cb=lambda: None,
            on_move_cb=lambda fb, tilt: None,
            robot_pose_se2=lambda: None
        )

        # If you're using RVIZ, you can send a nav goal to the robot on the
        # /move_base_simple/goal topic.  We'll listen on that topic and
        # forward the goal over to may_nav
        self._nav_goal_sub = rospy.Subscriber(
            "move_base_simple/goal",
            geometry_msgs.msg.PoseStamped,
            self._goal_received_cb
        )

    def run(self):
        '''
        Look in the home directory to see if there's a simlink to an active
        map.  If there is, have OORT load it, and start AMCL localization
        '''
        if os.path.isfile(NavController.MAP_PATH):
            self._map_manager.load_map(NavController.MAP_PATH)
            self._map_manager.start_localization()
            # Assume that on start-up we're on or near the dock.  It's as good
            # a guess as any. . .
            self._map_manager.localize_on_dock()
            self._power_monitor = PowerMonitor(
                dock_changed_cb=self._dock_changed_cb
            )

        else:
            # If we didn't find a map, that may be OK.  The user may not have
            # created one yet.
            # See kuri_mapping.launch and the assosciated activity to see how
            # to create a map
            rospy.logwarn(
                "Did not find a simlink to a map file at ~/map.map_capnp"
            )

        # Indicate to the world that we're running and ready to go:
        self._pub.publish(
            mayfield_msgs.msg.NodeStatus("nav_controller", True)
        )

        try:
            rospy.spin()
        except rospy.exceptions.ROSInterruptException:
            return

    def shutdown(self):
        self._nav_goal_sub.unregister()
        self._map_manager.shutdown()

    def _dock_changed_cb(self, msg):
        '''
        To help AMCL, relocalize to the dock whenever we're on it
        '''
        if msg.dock_present:
            self._map_manager.localize_on_dock()

    def _goal_received_cb(self, msg):
        '''
        For nav goals received from rviz
        '''
        # euler from quaternion expects a list, not a geometry_msgs.quaternion
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

        self._nav_client.go_to_pos(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            theta=euler_from_quaternion(quat)[2],
            mode=may_nav_py.nav.Nav.TOUCH_DRIVE
        )
