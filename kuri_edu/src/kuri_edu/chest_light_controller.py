import rospy

import mayfield_msgs.msg
import mayfield_utils
import mobile_base
import mobile_base_driver.msg

from .pulse_animation import PulseAnimation


class ChestLedController(object):

    CHEST_LIGHT_FRAMERATE = 15

    def __init__(self):

        self._pub = rospy.Publisher(
            "node_online",
            mayfield_msgs.msg.NodeStatus,
            latch=True,
            queue_size=1,
        )

        # mobile base driver needs to be running before we can run:
        mayfield_utils.wait_for_nodes(
            node_names=['controllers'],
        )

        self._light_client = mobile_base.ChestLightClient()
        self._anim = PulseAnimation(
            framerate=self.CHEST_LIGHT_FRAMERATE
        )

        # By default, we want the color of the chest light to be controlled
        # by the battery level of the robot.  Subscribe to mobile_base/power
        # to get this information
        self._power_sub = rospy.Subscriber(
            "mobile_base/power",
            mobile_base_driver.msg.Power,
            self._power_cb
        )

    def run(self):

        # Indicate to the world that we're running and ready to go:
        self._pub.publish(
            mayfield_msgs.msg.NodeStatus("chest_light_controller", True)
        )

        rate = rospy.Rate(self.CHEST_LIGHT_FRAMERATE)

        try:
            for frame in self._anim:
                if rospy.is_shutdown():
                    return
                self._light_client.put_pixels(frame)
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            return

    def shutdown(self):
        self._light_client.shutdown()
        self._power_sub.unregister()

    def _power_cb(self, msg):
        color = self._calculate_led_color(msg.battery.rounded_pct)

        # This will update the color of the animation, but not output.
        # Outputting is handled by the 'run' method
        self._anim.set_color(color)

    @staticmethod
    def _calculate_led_color(battery_percent):

        # The easy case:  Low battery is red
        if battery_percent <= 20:
            return (255, 0, 0)

        # Otherwise, it's time to interpolate.  Calculate
        # our battery level on a scale of 0.0 to 1.0
        interp_factor = 1.0 * (battery_percent - 20) / 80

        # Higher battery levels mean less red and more green
        return (
            int(255 * (1 - interp_factor)),
            int(255 * interp_factor),
            0
        )
