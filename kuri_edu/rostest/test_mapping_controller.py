#!/usr/bin/env python
import threading

import rostest
import rospy

import geometry_msgs.msg

import mobile_base
import mobile_base_driver.msg
import oort_msgs.srv

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

        cls.map_state_srv = rospy.ServiceProxy(
            "/oort_ros_mapping/map/state",
            oort_msgs.srv.GetString
        )

        cls.dock_srv = rospy.ServiceProxy(
            "/sim_interface/dock",
            gizmo_hw_sim.srv.Dock
        )

        # Wait for the DUT to be online before starting
        mayfield_utils.wait_for_nodes(['mapping_controller'])

    def test_01_kuri_starts_no_mapping(self):
        # Check that kuri starts with head down, eyes closed, and not
        # mapping

        # Negative numbers for 'tilt' are higher up
        self.assertGreater(
            self.joints.get_tilt_pos(),
            mobile_base.HeadClient.TILT_NEUTRAL
        )

        # Smaller numbers are 'more open' for the eyes
        self.assertGreater(
            self.joints.get_eye_pos(),
            mobile_base.HeadClient.EYES_NEUTRAL
        )

        self.assertEqual(
            self.map_state_srv().data,
            "not_mapping"
        )

    def test_02_start_mapping_on_dock(self):
        # Check that placing Kuri on the dock will open the eyes, raise
        # the head, and start mapping

        # We'll just key off of 'is_docked'  Charging doesn't matter
        self.dock_srv(is_docked=True, is_charging=False)

        rate = rospy.Rate(4)
        # Wait for mapping to start
        for _ in range(20):
            if self.map_state_srv().data == "mapping":
                break
            rate.sleep()
        else:
            self.assertTrue(False, "Timed out waiting for mapping to start")

        self.assertLess(
            self.joints.get_tilt_pos(),
            mobile_base.HeadClient.TILT_DOWN
        )

        self.assertLess(
            self.joints.get_eye_pos(),
            mobile_base.HeadClient.EYES_CLOSED
        )


if __name__ == '__main__':
    rospy.init_node("test_mapping_controller")
    rostest.rosrun("kuri_edu",
                   "test_mapping_controller",
                   TestMappingController)
