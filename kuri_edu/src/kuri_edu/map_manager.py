import rospy

import oort_msgs.srv
import std_srvs.srv

class MapManager(object):

    def __init__(self):
        self._mapping_state_srv = rospy.ServiceProxy(
            "oort_ros_mapping/map/state",
            oort_msgs.srv.GetString
        )

        self._mapping_start_srv = rospy.ServiceProxy(
            "oort_ros_mapping/map/start",
            oort_msgs.srv.SetString
        )

        self._mapping_stop_srv = rospy.ServiceProxy(
            "oort_ros_mapping/map/stop",
            std_srvs.srv.Empty
        )


    def get_map_state(self):
        return self._mapping_state_srv().data

    def start_mapping(self):
        self._mapping_start_srv("")

    def stop_mapping(self):
        self._mapping_stop_srv()

    def shutdown(self):
        self._mapping_state_srv.close()
