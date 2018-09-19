#!/usr/bin/env python
import rostest
import rospy

import gizmo_hw_sim.srv
import mobile_base_driver.msg
import mayfield_utils
import maytest.desktop


class TestChestLightController(maytest.desktop.RosTestBase):

    @classmethod
    def setUpClass(cls):

        cls.power_srv = rospy.ServiceProxy(
            "/sim_interface/power",
            gizmo_hw_sim.srv.Power
        )

        # Wait for the DUT to be online before starting
        mayfield_utils.wait_for_nodes(['chest_light_controller'])

    def test_full_battery_means_green(self):
        rospy.loginfo("test_full_battery_means_green")
        self.power_srv(100)

        rospy.sleep(0.5)

        light_frames = []

        # Grab 10 frames and then analyze them
        for _ in range(10):
            light_frames.append(
                rospy.wait_for_message(
                    "mobile_base/commands/chest_leds",
                    mobile_base_driver.msg.ChestLeds
                )
            )

        LED_OFF = mobile_base_driver.msg.Led(0, 0, 0)
        LED_GREEN = mobile_base_driver.msg.Led(0, 255, 0)

        # At 100% power, LEDs should either be off, or 100% green:
        for frame in light_frames:
            for led in frame.leds:
                if led != LED_OFF:
                    self.assertEqual(led, LED_GREEN)

    def test_low_battery_means_red(self):
        rospy.loginfo("test_low_battery_means_red")
        self.power_srv(10)

        rospy.sleep(0.5)

        light_frames = []

        # Grab 10 frames and then analyze them
        for _ in range(10):
            light_frames.append(
                rospy.wait_for_message(
                    "mobile_base/commands/chest_leds",
                    mobile_base_driver.msg.ChestLeds
                )
            )

        LED_OFF = mobile_base_driver.msg.Led(0, 0, 0)
        LED_RED = mobile_base_driver.msg.Led(255, 0, 0)

        # At 100% power, LEDs should either be off, or 100% green:
        for frame in light_frames:
            for led in frame.leds:
                if led != LED_OFF:
                    self.assertEqual(led, LED_RED)


if __name__ == '__main__':
    rospy.init_node("test_chest_light_controller")
    rostest.rosrun("kuri_edu",
                   "test_chest_light_controller",
                   TestChestLightController)
