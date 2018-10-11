#!/usr/bin/env python
import threading

import rostest
import rospy

import geometry_msgs.msg

import mobile_base
import mobile_base_driver.msg

import gizmo_hw_sim.srv
import mayfield_utils
import maytest.desktop


class TestMappingController(maytest.desktop.RosTestBase):

    @classmethod
    def setUpClass(cls):
        # To monitor joints:
        cls.joints = mobile_base.JointStates()

        # For sending drive commands to the safety controller:
        cls.vel_pub = rospy.Publisher(
            "/cmd_vel",
            geometry_msgs.msg.Twist,
            latch=True,
            queue_size=2,
        )

        # Wait for the DUT to be online before starting
        mayfield_utils.wait_for_nodes(['mapping_controller'])

    def test_01_kuri_drives_on_standard_topics(self):
        rospy.sleep(15)


if __name__ == '__main__':
    rospy.init_node("test_mapping_controller")
    rostest.rosrun("kuri_edu", "test_mapping_controller", TestMappingController)
