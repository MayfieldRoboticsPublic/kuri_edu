#!/usr/bin/env python
import rostest
import rospy

import mayfield_utils
import maytest.desktop
import mobile_base
import vision_bridge


class TestHeadController(maytest.desktop.RosTestBase):

    @classmethod
    def setUpClass(cls):
        cls.joints = mobile_base.JointStates()
        cls.vision_client = vision_bridge.VisionClient()
        # Wait for the DUT to be online before starting
        mayfield_utils.wait_for_nodes(['head_controller'])

    def test_01_test_head_up_eye_open(self):
        rospy.loginfo("test_01_test_head_up_eye_open")

        # Negative numbers for 'tilt' are higher up
        self.assertLess(
            self.joints.get_tilt_pos(),
            mobile_base.HeadClient.TILT_NEUTRAL
        )

        # Smaller numbers are 'more open' for the eyes
        self.assertLess(
            self.joints.get_eye_pos(),
            mobile_base.HeadClient.EYES_NEUTRAL
        )

    def test_02_vision_is_active(self):
        rospy.loginfo("test_02_vision_is_active")
        # We expect the head controller node to have started the face detector
        self.assertIn("face_detector", self.vision_client.active_modules())


if __name__ == '__main__':
    rospy.init_node("test_head_controller")
    rostest.rosrun("kuri_edu", "test_head_controller", TestHeadController)
