import threading

import geometry_msgs.msg
import rospy

import mayfield_utils
import mobile_base


class SafetyController(object):

    # A set of the safety events we're equipped to handle.
    # It's just bumpers so we don't need to deal with compexities
    # like "What happens if a front bump and a rear cliff triggers
    # at the same time"
    HANDLED_EVENTS = {'BPR_bp', 'BPM_bp', 'BPL_bp'}
    SAFETY_HZ = 10

    def __init__(self):

        # Controllers need to be loaded before we can run
        mayfield_utils.wait_for_nodes(
            node_names=['controllers'],
        )

        self._safety_client = mobile_base.SafetyClient()
        # Override the safety controller for all events that we don't
        # handle so they'll be ignored by the hardware
        self.UNHANDLED_EVENTS = (
            self._safety_client.safety_statuses()  # All statuses
            - self.HANDLED_EVENTS                  # Minus the ones we handle
        )
        self._safety_client.safety_override(self.UNHANDLED_EVENTS)

        self._sync_lock = threading.Lock()
        self._block_twists = False

        # Set up publishers before subscribers because once subscribers are
        # set up, we can start getting callbacks
        self._cmd_vel_pub = rospy.Publisher(
            "mobile_base/commands/velocity",
            geometry_msgs.msg.Twist,
            queue_size=1
        )

        self._cmd_vel_sub = rospy.Subscriber(
            "cmd_vel",
            geometry_msgs.msg.Twist,
            self._forward_twists
        )

    def run(self):

        rate = rospy.Rate(self.SAFETY_HZ) # 10hz by default
        while not rospy.is_shutdown():
            try:

                current_status = self._safety_client.get_safety_status()

                if self.HANDLED_EVENTS.intersection(current_status) != set():
                    with self._sync_lock:
                        self._block_twists = True

                    # Override the safety status and back the robot up
                    self._stop()  # in case a forward velocity is enqueued
                    self._safety_client.safety_override(current_status)

                    self._back_up(rate)

                    self._safety_client.safety_override(self.UNHANDLED_EVENTS)

                    self._safety_client.safety_clear(current_status)
                    self._block_twists = False

                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                return

    def _stop(self):
        self._cmd_vel_pub.publish(
            geometry_msgs.msg.Twist(
                linear=geometry_msgs.msg.Vector3(0, 0, 0),
                angular=geometry_msgs.msg.Vector3(0, 0, 0)
            )
        )

    def _back_up(self, rate):
        for _ in range(self.SAFETY_HZ / 2):
            self._cmd_vel_pub.publish(
                geometry_msgs.msg.Twist(
                    linear=geometry_msgs.msg.Vector3(-0.1, 0, 0),
                    angular=geometry_msgs.msg.Vector3(0, 0, 0)
                )
            )
            rate.sleep()
        self._stop()

    def shutdown(self):
        self._safety_client.shutdown()

    def _forward_twists(self, msg):
        '''
        This callback listens for twist messages come on standard topics.
        If we're not handling a safety event, we forward the message on
        to the wheels.  If we're currently handling a safety event, then
        we block the message from going to the wheels
        '''
        with self._sync_lock:
            if not self._block_twists:
                self._cmd_vel_pub.publish(msg)
