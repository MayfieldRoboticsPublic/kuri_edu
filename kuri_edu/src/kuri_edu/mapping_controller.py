import rospy

import mayfield_msgs.msg
import mayfield_utils
import mobile_base
from mobile_base import HeadClient


class MappingController(object):

    def __init__(self):

        self._pub = rospy.Publisher(
            "node_online",
            mayfield_msgs.msg.NodeStatus,
            latch=True,
            queue_size=1,
        )

        # Controllers need to be loaded before we can run
        mayfield_utils.wait_for_nodes(
            node_names=['controllers'],
        )

        # Joint states keeps track of the latest position of all the robot's
        # joints
        self._joint_states = mobile_base.JointStates()

        # Head client allows us to send trajectories down to the hardware
        self._head_client = mobile_base.HeadClient(self._joint_states)

        # Make sure the head client is ready to be used
        assert self._head_client.wait_for_server(timeout=rospy.Duration(30.0))

    def run(self):

        # At the start, Kuri should be head down and asleep
        self._head_client.pan_and_tilt(
            pan=HeadClient.PAN_NEUTRAL,
            tilt=HeadClient.TILT_DOWN,
            duration=1.0
        )
        self._head_client.eyes_to(
            radians=HeadClient.EYES_CLOSED,
            duration=0.5
        )

        assert self._head_client.wait_for_done(5.0), "head client timed out"

        # Indicate to the world that we're running and ready to go:
        self._pub.publish(
            mayfield_msgs.msg.NodeStatus("mapping_controller", True)
        )

        try:
            rospy.spin()
        except rospy.exceptions.ROSInterruptExcepion:
            return

    def shutdown(self):
        self._head_client.shutdown()
        self._joint_states.shutdown()
