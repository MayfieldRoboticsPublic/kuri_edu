import threading

import geometry_msgs.msg
import rospy

import mayfield_msgs.msg
import mayfield_utils
import mobile_base


class HeadController(object):

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

        # Indicate to the world that we're running and ready to go:
        self._pub.publish(
            mayfield_msgs.msg.NodeStatus("safety_controller", True)
        )

        # At the start, open Kuri's eyes and point the head up:
        self._head_client.pan_and_tilt(
            pan=HeadClient.PAN_NEUTRAL,
            tilt=HeadClient.TILT_UP,
            duration=1.0
        )
        self._head_client.eyes_to(
            radians=HeadClient.EYES_OPEN,
            duration=0.5
        )

        try:
            rospy.spin()
        except rospy.exceptions.ROSInterruptExcepion:
            return 

    def shutdown(self):
        self._joint_states.shutdown()
        self._head_client.shutdown()
