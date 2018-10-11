import rospy

import nav_msgs.msg
import std_srvs.srv

import oort_msgs.srv

import mayfield_utils


class MapManager(object):

    onMapGrew = mayfield_utils.Event()

    def __init__(self):

        self._map_size = 0

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

        self._map_sub = rospy.Subscriber(
            "map",
            nav_msgs.msg.OccupancyGrid,
            self._map_cb
        )

    def get_map_state(self):
        return self._mapping_state_srv().data

    def start_mapping(self):
        self._mapping_start_srv("")

    def stop_mapping(self):
        self._mapping_stop_srv()

    def shutdown(self):
        self._mapping_state_srv.close()
        self._mapping_start_srv.close()
        self._mapping_stop_srv.close()
        self._map_sub.unregister()

    def _map_cb(self, msg):
        # Calculate map size in square meters.
        pxl_area = msg.info.resolution * msg.info.resolution
        map_size = sum(d == 0 for d in msg.data) * pxl_area

        if map_size - self._map_size > 1:
            self._map_size = map_size
            self.onMapGrew(map_size)
