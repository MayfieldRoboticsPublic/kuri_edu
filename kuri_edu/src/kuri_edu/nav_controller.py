import rospy

import mayfield_msgs.msg
import mayfield_utils
import mobile_base
from mobile_base import HeadClient

from .map_manager import MapManager
from .power_monitor import PowerMonitor


class NavController(object):
    '''
    The nav controller is responsible for loading saved maps.  This
    class will look in the home directory for a simlink to a map created
    with kuri_mapping.launch and attempt to load it and start localization
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

        self._map_manager = MapManager()


    def run(self):

        # Indicate to the world that we're running and ready to go:
        self._pub.publish(
            mayfield_msgs.msg.NodeStatus("nav_controller", True)
        )

        try:
            rospy.spin()
        except rospy.exceptions.ROSInterruptException:
            return

    def shutdown(self):
        self._map_manager.shutdown()

