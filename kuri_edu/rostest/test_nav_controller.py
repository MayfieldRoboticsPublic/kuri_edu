#!/usr/bin/env python
import os.path
import threading

import rostest
import rospy

import amcl.msg
import geometry_msgs.msg
import nav_msgs.msg

import gizmo_hw_sim.srv
import oort_msgs.srv

import mayfield_utils
import mobile_base

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


if __name__ == '__main__':
    rospy.init_node("test_nav_controller")
    rostest.rosrun("kuri_edu", "test_nav_controller", TestNavController)
