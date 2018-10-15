from kuri_edu import PowerMonitor

import threading

import mobile_base_driver.msg

import fakerospy
import maytest


class TestPowerMonitor(maytest.TestBase):

    def setUp(self):
        super(TestPowerMonitor, self).setUp()
        self.patch("kuri_edu.power_monitor.rospy", fakerospy)

        self.power_pub = fakerospy.Publisher(
            "mobile_base/power",
            mobile_base_driver.msg.Power
        )

    def test_ctor_event_hookups(self):

        dock_changed = threading.Event()
        charging_changed = threading.Event()

        dut = PowerMonitor(
            dock_changed_cb=lambda x: dock_changed.set(),
            charging_changed_cb=lambda x: charging_changed.set()
        )
        self.addCleanup(dut.shutdown)

        self.power_pub.publish(
            mobile_base_driver.msg.Power(
                dock_present=True
            )
        )

        self.assertTrue(dock_changed.is_set())
        self.assertFalse(charging_changed.is_set())

        self.power_pub.publish(
            mobile_base_driver.msg.Power(
                dock_present=True,
                is_charging=True,
            )
        )

        self.assertTrue(dock_changed.is_set())
        self.assertTrue(charging_changed.is_set())
