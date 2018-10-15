#!/usr/bin/env python
import os.path

import rostest
import rospy

import geometry_msgs.msg

import mobile_base
import nav_msgs.msg
import oort_msgs.srv

import gizmo_hw_sim.srv
import mayfield_utils
import maytest.desktop


class TestNavController(maytest.desktop.RosTestBase):

    @classmethod
    def setUpClass(cls):
        # Wait for the DUT to be online before starting
        mayfield_utils.wait_for_nodes(['nav_controller'])

    def test_01(self):
        pass


if __name__ == '__main__':
    rospy.init_node("test_nav_controller")
    rostest.rosrun("kuri_edu", "test_nav_controller", TestNavController)
