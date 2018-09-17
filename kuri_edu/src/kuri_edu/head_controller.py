import rospy

import mayfield_msgs.msg
import mayfield_utils
import mobile_base
from mobile_base import HeadClient


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

        # Vision client activates and deactivates face detection
        self._vision_client = VisionClient()
        # Activate vision detections.  This eats up a lot of CPU and it may
        # not be possible to run this at all times, especially when mapping
        self._vision_client.activate_module(
            module_name=VisionClient.FACE_DETECTOR
        )


    def run(self):

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

        assert self._head_client.wait_for_done(5.0), "head client timed out"

        # Indicate to the world that we're running and ready to go:
        self._pub.publish(
            mayfield_msgs.msg.NodeStatus("head_controller", True)
        )

        try:
            rospy.spin()
        except rospy.exceptions.ROSInterruptExcepion:
            return

    def shutdown(self):
        try:
            self._vision_client.deactivate_all_modules()
        except Exception:
            # This could fail if the ROS master is shutting down
            pass

        self._head_client.shutdown()
        self._joint_states.shutdown()
