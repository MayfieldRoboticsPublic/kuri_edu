import uuid
import os
import subprocess

import rospy

import nav_msgs.msg
import std_srvs.srv

import oort_msgs.srv
import mayfield_utils


class MapManager(object):

    onMapGrew = mayfield_utils.Event()

    def __init__(self):

        self._map_path = None
        self._map_size = 0

        self._graph_loc_create_srv = rospy.ServiceProxy(
            "oort_ros_mapping/graph_loc/create",
            oort_msgs.srv.LocCreate
        )

        self._map_load_srv = rospy.ServiceProxy(
            "oort_ros_mapping/map/load",
            oort_msgs.srv.SetString
        )

        self._map_republish_srv = rospy.ServiceProxy(
            "oort_ros_mapping/map/republish_maps",
            std_srvs.srv.Empty
        )

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

        self._notify_dock_srv = rospy.ServiceProxy(
            "oort_ros_mapping/map/notify_docked",
            std_srvs.srv.Empty
        )

        self._amcl_start_srv = rospy.ServiceProxy(
            "localization_start",
             std_srvs.srv.Empty
        )

        self._map_sub = rospy.Subscriber(
            "map",
            nav_msgs.msg.OccupancyGrid,
            self._map_cb
        )

    def convert_map(self):
        assert self._map_path, "Mapping not started.  No map to convert"

        subprocess.check_call(
            ["rosrun", "map_server", "map_saver"],
            cwd=self._map_path
        )

    def get_map_state(self):
        return self._mapping_state_srv().data

    def load_map(self, map_path):
        '''
        Loads an existing OORT map

        :param map_path: A full path to the map.map_map_capnp file to load 
        '''
        # Need to resolve simlinks, or OORT will look for the md5 in the wrong
        # place.  We also need to remove the file extension before we give the
        # path to OORT
        map_path = os.path.realpath(map_path)
        if ".map_capnp" in map_path:
            map_path = os.path.splitext(map_path)[0]
        self._map_load_srv(map_path)

    def notify_docked(self):
        self._notify_dock_srv()

    def save_waypoint(self, nspace, wp_name):
        self._graph_loc_create_srv(nspace, wp_name)

    def start_localization(self):
        '''
        Starts AMCL
        This method expects OORT to have a map.  Either loaded from disk, or
        created
        '''
        # Make sure we're not actively mapping, or both OORT and AMCL are
        # going to try to publish odom
        assert self.get_map_state() != "mapping"

        self._amcl_start_srv()
        # AMCL will use the first map it sees after it's started:
        self._map_republish_srv()

    def start_mapping(self):
        '''
        Starts OORT mapping and returns the full path to the saved map data

        :returns: A path to the directory used to store the map
        '''
        map_path = os.path.join(
            os.path.expanduser("~"),
            "oort_maps",
            str(uuid.uuid4())
        )
        os.makedirs(map_path)  # Recursive

        # OORT node will drop a few files in the map_path directory:
        # - map.map_capnp
        # - map.map_meta_capnp
        # - map.md5
        self._mapping_start_srv(os.path.join(map_path, "map"))
        self._map_path = map_path
        return map_path

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
