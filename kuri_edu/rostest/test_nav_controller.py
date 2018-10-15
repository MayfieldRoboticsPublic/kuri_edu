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


if __name__ == '__main__':
    rospy.init_node("test_nav_controller")
    rostest.rosrun("kuri_edu", "test_nav_controller", TestNavController)
