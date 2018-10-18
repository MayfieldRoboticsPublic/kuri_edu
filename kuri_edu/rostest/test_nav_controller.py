#!/usr/bin/env python
import threading

import rostest
import rospy
from tf.transformations import quaternion_from_euler

import amcl.msg
import geometry_msgs.msg
import nav_msgs.msg

import gizmo_hw_sim.srv

import mayfield_utils

import maytest.desktop


class LocalizationMonitor(object):
    '''
    Watch the 'initialpose_cloud' topic to see when we relocalize
    '''
    def __init__(self):
        self._msgs = []
        self._event = threading.Event()

        self.sub = rospy.Subscriber(
            "initialpose_cloud",
            amcl.msg.HypothesisSet,
            self._subscriber_callback
        )

        # The relocalization topic is not latched, and subscribers
        # take some time behind the scenes to actually hook up (ugh)
        # so we need to take some time here to make sure our subscriber
        # is fully hooked up before we expect to receive a message
        rospy.sleep(0.75)

    def _subscriber_callback(self, msg):
        self._msgs.append(msg)
        self._event.set()

    def wait_for_message(self, timeout=10):
        assert self._event.wait(timeout)
        self._event.clear()
        return self._msgs[-1]


class TestNavController(maytest.desktop.RosTestBase):

    @classmethod
    def setUpClass(cls):
        cls.dock_srv = rospy.ServiceProxy(
            "/sim_interface/dock",
            gizmo_hw_sim.srv.Dock
        )

        cls.goal_pub = rospy.Publisher(
            "move_base_simple/goal",
            geometry_msgs.msg.PoseStamped
        )

        # Wait for the DUT to be online before starting
        mayfield_utils.wait_for_nodes(['nav_controller'])

    def test_01_map_is_loaded(self):

        loaded_map = rospy.wait_for_message(
            "/map",
            nav_msgs.msg.OccupancyGrid,
            timeout=10.0
        )

        # Make sure the map has a bunch of known space filled in:
        self.assertGreater(
            sum(d == 0 for d in loaded_map.data),
            200
        )

    def test_02_docking_triggers_localization(self):

        monitor = LocalizationMonitor()

        self.dock_srv(is_docked=True, is_charging=False)

        monitor.wait_for_message()

    def test_03_test_autonomous_navigation(self):

        initial_loc = self.get_gizmo_location()
        rospy.loginfo("Kuri started at {}".format(initial_loc))

        self.goal_pub.publish(
            geometry_msgs.msg.PoseStamped(
                pose=geometry_msgs.msg.Pose(
                    # Launch file starts kuri at -4.5, -5.4.  Move fowrad 1m
                    position=geometry_msgs.msg.Point(-3.5, -5.4, 0),
                    orientation=geometry_msgs.msg.Quaternion(0, 0, 0, 1)
                )
            )
        )

        # Poll to see that Kuri moves to the new destination
        rate = rospy.Rate(1)
        start = rospy.get_time()

        # Wait until we're done moving
        while rospy.get_time() - start  < 15:
            loc = self.get_gizmo_location()
            print loc
            if abs(loc[0] - 1) <= 0.1:
                break
            rate.sleep()

        # Check that we navigated to where we want to go
        final_loc = self.get_gizmo_location()
        self.assertAlmostEquals(final_loc[0], 1, places=1)
        self.assertAlmostEquals(final_loc[1], 0, places=1)


if __name__ == '__main__':
    rospy.init_node("test_nav_controller")
    rostest.rosrun("kuri_edu", "test_nav_controller", TestNavController)
