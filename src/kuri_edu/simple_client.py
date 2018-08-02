import rospy

import geometry_msgs.msg

import mayfield_utils

from mobile_base import SafetyClient
from mobile_base import JointStates
from mobile_base import HeadClient


class SimpleKuriClient(object):

    def __init__(self):

        # Controllers need to be loaded before we can run
        mayfield_utils.wait_for_nodes(
            node_names=['controllers'],
        )

        self._safety_client = SafetyClient()
        self._joint_states = JointStates()
        self._head_client = HeadClient(self._joint_states)

        # The simple client doesn't do anything with the safety events.
        # We're just going to override all of the safety systems so the
        # robot can be driven directly
        self._safety_client.safety_override(
            self._safety_client.safety_statuses()  # All safety bits
        )

        # Point Kuri's head forward and open the eyes
        assert self._head_client.wait_for_server(timeout=rospy.Duration(30.0))
        self._head_client.pan_and_tilt(
            pan=HeadClient.PAN_NEUTRAL,
            tilt=HeadClient.TILT_NEUTRAL,
            duration=1.0
        )
        self._head_client.eyes_to(
            radians=HeadClient.EYES_OPEN,
            duration=0.5
        )

        # The mobile base driver listens on the /mobile_base/commands/velocity
        # topic, but standard tools publish on the /cmd_vel topic.  We're
        # going to forward /cmd_vel over to /mobile_base/commands/velocity.
        #
        # A more advanced client with safety recovery behavior might want
        # to have logic to interrupt the flow of twist messages from
        # /cmd_vel to /mobile_base/commands/velocity while it automatically
        # recovers from a safety event

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

    def shutdown(self):
        self._cmd_vel_sub.unregister()
        self._cmd_vel_pub.unregister()

        self._joint_states.shutdown()
        self._head_client.shutdown()
        self._safety_client.shutdown()

    def _forward_twists(self, msg):
        # A more advanced RC-Kuri would conditionally forward these messages
        # on whether or not a safety event was being handled
        self._cmd_vel_pub.publish(msg)
