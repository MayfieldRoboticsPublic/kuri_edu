#!/usr/bin/env python
import rostest
import rospy

import threading

import gizmo_hw_sim.srv
import mobile_base_driver.msg
import mayfield_utils
import maytest.desktop


class LightMonitor(object):
    '''
    Look at the chest LED topic and record the messages as they come in.
    Also provide mechanisms for waiting for messages to come in     
    '''
    def __init__(self):
        self._msgs = []
        self._event = threading.Event()

        self.sub = rospy.Subscriber(
            "mobile_base/commands/chest_leds",
            mobile_base_driver.msg.ChestLeds,
            self._subscriber_callback
        )

        # The chest LEDs topic is latched in the DUT.  Wait for the
        # latched message to come in so tests can wait on the next message
        self.wait_for_message()

    def _subscriber_callback(self, msg):
        self._msgs.append(msg)
        self._event.set()

    def wait_for_message(self, timeout=10):
        assert self._event.wait(timeout)
        self._event.clear()
        return self._msgs[-1]

class TestCapTouchChestLight(maytest.desktop.RosTestBase):

    @classmethod
    def setUpClass(cls):
        # Fake touch-messages - alas not simulated :-(
        cls.touch_pub = rospy.Publisher(
            "mobile_base/touch",
            mobile_base_driver.msg.Touch,
            latch=True
        )

        # Get one light message latched on the output of the DUT so
        # the LightMonitor can work correctly
        cls.touch_pub.publish([False] * 8)

        # Wait for the DUT to be online before starting
        mayfield_utils.wait_for_nodes(['cap_touch_chest_light'])

    def setUp(self):
        super(TestCapTouchChestLight, self).setUp()
        rospy.wait_for_message(
            "mobile_base/commands/chest_leds",
            mobile_base_driver.msg.ChestLeds,
            1.0
        )

    def test_no_touch_no_light(self):

        monitor = LightMonitor()

        self.touch_pub.publish([False] * 8)

        lights = monitor.wait_for_message()

        for led in lights.leds:
            self.assertEqual(led, mobile_base_driver.msg.Led(0, 0, 0))

    def test_all_touch_all_light(self):
        monitor = LightMonitor()

        self.touch_pub.publish([True] * 8)

        lights = monitor.wait_for_message()

        # We expect some LEDs to be on.  This is a very weak
        # assertion, but we want it to still be true if different
        # patterns are implemented
        for led in lights.leds:
            if led != mobile_base_driver.msg.Led(0,0,0):
                break;
        else:
            self.assertTrue(False, "Did not find any lit LEDs")


if __name__ == '__main__':
    rospy.init_node("test_cap_touch_chest_light")
    rostest.rosrun("kuri_edu",
                   "test_cap_touch_chest_light",
                   TestCapTouchChestLight)
