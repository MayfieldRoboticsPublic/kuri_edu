#!/usr/bin/env python
import threading

import rostest
import rospy

import geometry_msgs.msg

import mobile_base_driver.msg

import gizmo_hw_sim.srv
import mayfield_utils
import maytest.desktop


class TestSafetyController(maytest.desktop.RosTestBase):

    @classmethod
    def setUpClass(cls):
        # For sending drive commands to the safety controller:
        cls.vel_pub = rospy.Publisher(
            "/cmd_vel",
            geometry_msgs.msg.Twist,
            latch=True,
            queue_size=2,
        )

        # For triggering the bump shell:
        rospy.wait_for_service("/sim_interface/kick")
        cls.kick_srv = rospy.ServiceProxy(
            "/sim_interface/kick",
            gizmo_hw_sim.srv.Kick
        )

        # Wait for the DUT to be online before starting
        mayfield_utils.wait_for_nodes(['safety_controller'])

    def test_01_kuri_drives_on_standard_topics(self):
        rospy.loginfo("test_01_kuri_drives_on_standard_topics started")

        initial_pos = self.get_gizmo_location()

        r = rospy.Rate(5)
        # Drive forward half a meter:
        while self.get_gizmo_location()[0] - initial_pos[0] <= 0.5:
            self.vel_pub.publish(
                geometry_msgs.msg.Twist(
                    linear=geometry_msgs.msg.Vector3(1.0, 0, 0),
                    angular=geometry_msgs.msg.Vector3(0, 0, 0)
                )
            )
            r.sleep()

        self.vel_pub.publish(
            geometry_msgs.msg.Twist(
                linear=geometry_msgs.msg.Vector3(0, 0, 0),
                angular=geometry_msgs.msg.Vector3(0, 0, 0)
            )
        )
        rospy.sleep(1.0)

        self.assertGreater(
            self.get_gizmo_location()[0] - initial_pos[0],
            0.5
        )

    def test_02_kuri_backs_up_when_bumped(self):
        rospy.loginfo("test_01_kuri_backs_up_when_bumped started")
        self._check_bump_backup([True, False, False])
        self._check_bump_backup([False, True, False])
        self._check_bump_backup([False, False, True])

    def _check_bump_backup(self, bumps=[True, True, True]):
        # Establish initial conditions:
        initial_pos = self.get_gizmo_location()
        status_mon = SafetyStatusMonitor()

        # Test Stimulus
        self.kick_srv(*bumps)

        # Wait for kuri to react
        status_mon.assertSafetyConditionSeen()
        status_mon.assertSafetyConditionCleared()

        # Make sure we backed up at least a cm
        final_pos = self.get_gizmo_location()
        backup_distance = final_pos[0] - initial_pos[0]
        self.assertLess(backup_distance, -0.01)

    def test_03_kuri_continues_to_drive(self):
        self.test_01_kuri_drives_on_standard_topics()


class SafetyStatusMonitor(object):
    '''
    Monitors the /mobile_base/safety_status topic to tell when bumps are
    being handled
    '''

    def __init__(self):
        self._safety_condition = threading.Event()
        self._condition_cleared = threading.Event()

        self._sub = rospy.Subscriber(
            "/mobile_base/safety_status",
            mobile_base_driver.msg.SafetyStatus,
            self._safety_status_callback
        )

    def _safety_status_callback(self, msg):
        '''
        Operates in two stages.  If we haven't observed a safety
        condition yet, sets the safety condition event when we get a non
        zero safety status.

        If we've observed a safety condition already, sets the condition
        cleared event when we get a zero safety status
        '''
        if not self._safety_condition.is_set():
            if msg.status != 0:
                self._safety_condition.set()
        else:
            if msg.status == 0:
                self._condition_cleared.set()

    def assertSafetyConditionSeen(self, timeout=10):
        assert self._safety_condition.wait(timeout)

    def assertSafetyConditionCleared(self, timeout=10):
        assert self._condition_cleared.wait(timeout)


if __name__ == '__main__':
    rospy.init_node("test_safety_controller")
    rostest.rosrun("kuri_edu", "test_safety_controller", TestSafetyController)
