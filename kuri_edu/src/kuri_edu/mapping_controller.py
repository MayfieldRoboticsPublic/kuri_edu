import rospy

import mayfield_msgs.msg
import mayfield_utils
import mobile_base
from mobile_base import HeadClient

from .map_manager import MapManager
from .power_monitor import PowerMonitor


class MappingController(object):
    '''
    The mapping controller node allows you to create a map using kuri's
    mapping system, OORT.

    Mapping will start once Kuri detects its on the dock.  The eyes will open
    and Kuri will look forward as an indication that mapping is started

    The user then drives Kuri using the keyboard teleop node.

    You can monitor the progress of mapping using a tool like rviz.  When the
    map reaches 20 squre meters, Kuri will start to smile.

    To stop mapping, driver Kuri back onto the dock.  Kuri's eyes will close
    and the head will go down.  The map will be saved to the user's home
    directory:

    ~/oort_maps/<uuid>/map.map_capnp      - OORT map file
    ~/oort_maps/<uuid>/map.map_meta_capnp - OORT map file
    ~/oort_maps/<uuid>/map.md5            - OORT map file checksum
    ~/oort_maps/<uuid>/map.pgm            - map_saver output (standard format)
    ~/oort_maps/<uuid>/map.yaml           - map_saver output (standard format)

    A different UUID will be used each time the mapping_controller creates a
    new map
    '''

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

        self._power_monitor = None  # Created when we start to run
        self._map_manager = MapManager()
        self._map_manager.onMapGrew.connect(self._map_grew)

        self._mapping_complete = False

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

        self._power_monitor = PowerMonitor(
            dock_changed_cb=self._dock_changed_cb
        )

        # _start_mapping and _stop_mapping are called from ROS callbacks
        # once _stop mapping runs, it will set self._mapping_complete
        # to True.  At this point, the main loop below will save the map
        # in a standard format, and then stop

        try:
            while not rospy.is_shutdown():
                if self._mapping_complete:
                    # Telling OORT to close off the map can be a time
                    # consuming process, so we're doing it from the main
                    # thread instead of from a ROS callback
                    self._map_manager.stop_mapping()
                    rospy.logwarn("Mapping complete.  Converting. . .")
                    self._map_manager.convert_map()
                    rospy.logwarn("Map conversion complete")
                    rospy.spin()  # Hang until shutdown
                    return
                rospy.sleep(0.5)
        except rospy.exceptions.ROSInterruptException:
            return

    def shutdown(self):
        self._head_client.shutdown()
        self._joint_states.shutdown()

        if self._power_monitor:
            self._power_monitor.shutdown()

        self._map_manager.shutdown()

    def _dock_changed_cb(self, msg):
        '''
        Called when the Power Monitor detects we've moved on or off the
        dock

        This callback will start mapping when we've moved onto the dock,
        and will stop mapping when we're back on the dock with a large enough
        map
        '''

        # Early-out.  If we've already mapped, we're all done here
        if self._mapping_complete:
            return

        if msg.dock_present:
            map_state = self._map_manager.get_map_state()
            if map_state == "not_mapping":
                self._start_mapping()
            elif map_state == "mapping":
                self._stop_mapping()

    def _map_grew(self, map_size):
        '''
        Makes kuri smile once the map is big enough
        '''
        if map_size > 20:
            self._head_client.eyes_to(
                radians=HeadClient.EYES_HAPPY,
                duration=0.25
            )

    def _start_mapping(self):
        '''
        Begins the mapping process.  Starts OORT, has Kuri open eyes and look
        forward
        '''
        # At the start, Kuri should be head down and asleep
        self._head_client.pan_and_tilt(
            pan=HeadClient.PAN_NEUTRAL,
            tilt=HeadClient.TILT_NEUTRAL,
            duration=1.0
        )
        self._head_client.eyes_to(
            radians=HeadClient.EYES_OPEN,
            duration=0.5
        )

        assert self._head_client.wait_for_done(5.0), "head client timed out"

        map_path = self._map_manager.start_mapping()
        # This is a warning so it's easier to see in the log
        rospy.logwarn("Started mapping.  Map stored at {}".format(map_path))

    def _stop_mapping(self):
        '''
        At the end of mapping, put the head back down and close off the map
        '''

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

        # Telling OORT to finish off the map is time consuming.  We'll do that
        # from the main thread
        self._mapping_complete = True
