import rospy

import mobile_base
import mobile_base_driver.msg


class BatteryMonitor(object):
    '''
    The BatteryMonitor class demonstrates how to get Kuri's battery level
    and how to send colors to Kuri's chest LED

    This class will monitor kuri's battery level and interpolate a color
    between green and red to display on Kuri's chest LED.

    Green = 100% battery
    Red = 20% or lower battery
    '''

    def __init__(self):
        self._last_power_reading = -1
        self._light_client = mobile_base.ChestLightClient()

        self._power_sub = rospy.Subscriber(
            "mobile_base/power",
            mobile_base_driver.msg.Power,
            self._power_cb
        )

    def shutdown(self):
        self._light_client.shutdown()
        self._power_sub.unregister()

    def _power_cb(self, msg):
        try:
            if self._last_power_reading != msg.battery.rounded_pct:
                # The battery level changed!  We're going to send out a new
                # color to the chest LED
                self._light_client.put_pixels(
                    self._light_client.all_leds(
                        self._calculate_led_color(msg.battery.rounded_pct)
                    )
                )
        finally:
            self._last_power_reading = msg.battery.rounded_pct

    def _calculate_led_color(self, battery_percent):

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
