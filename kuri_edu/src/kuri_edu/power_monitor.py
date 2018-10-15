import rospy

import mayfield_utils
import mobile_base_driver.msg


class PowerMonitor(object):

    onDockChanged = mayfield_utils.Event()
    onChargingChanged = mayfield_utils.Event()

    def __init__(self,
                 dock_changed_cb=None,
                 charging_changed_cb=None):
        self._on_dock = False
        self._charging = False

        if dock_changed_cb:
            self.onDockChanged.connect(dock_changed_cb)

        if charging_changed_cb:
            self.onChargingChanged.connect(charging_changed_cb)

        self._power_sub = rospy.Subscriber(
            "mobile_base/power",
            mobile_base_driver.msg.Power,
            self._power_cb
        )

    def shutdown(self):
        self._power_sub.unregister()

    def _power_cb(self, msg):
        if self._on_dock != msg.dock_present:
            self._on_dock = msg.dock_present
            self.onDockChanged(msg)

        if self._charging != msg.is_charging:
            self._charging = msg.is_charging
            self.onChargingChanged(msg)
