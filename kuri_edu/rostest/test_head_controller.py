#!/usr/bin/env python
import rostest
import rospy

import mayfield_utils
import maytest.desktop


class TestHeadController(maytest.desktop.RosTestBase):

    @classmethod
    def setUpClass(cls):
        # Wait for the DUT to be online before starting
        mayfield_utils.wait_for_nodes(['head_controller'])

    def test_01_test_head_up_eye_open(self):
        pass


if __name__ == '__main__':
    rospy.init_node("test_head_controller")
    rostest.rosrun("kuri_edu", "test_head_controller", TestHeadController)
