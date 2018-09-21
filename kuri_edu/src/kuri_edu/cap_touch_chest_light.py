import rospy

import mayfield_msgs.msg
import mayfield_utils
import mobile_base
import mobile_base_driver.msg
from mobile_base import ChestLightClient as clc

class CapTouchChestLed(object):

    # Map touch inputs to LED outputs
    TOUCH_TO_LIGHT = ([clc.IDX_OUTER_BOTTOM_MID_RIGHT],
                      [clc.IDX_OUTER_UPPER_MID_RIGHT],
                      [clc.IDX_OUTER_UPPER_TOP_RIGHT],
                      range(clc.IDX_INNER_LEFT+1),
                      [clc.IDX_OUTER_BOTTOM_LOW_LEFT, clc.IDX_OUTER_BOTTOM_LOW_RIGHT],
                      [clc.IDX_OUTER_UPPER_TOP_LEFT],
                      [clc.IDX_OUTER_UPPER_MID_LEFT],
                      [clc.IDX_OUTER_BOTTOM_MID_LEFT])

    TOUCH_TO_COLOR = (clc.ON,
                      clc.ON,
                      clc.ON,
                      clc.ON,
                      clc.ON,
                      clc.ON,
                      clc.ON,
                      clc.ON)

    def __init__(self):

        self._pub = rospy.Publisher(
            "node_online",
            mayfield_msgs.msg.NodeStatus,
            latch=True,
            queue_size=1,
        )

        # mobile base driver needs to be running before we can run:
        mayfield_utils.wait_for_nodes(
            node_names=['controllers'],
        )

        self._light_client = mobile_base.ChestLightClient()

        self._touch_sub = rospy.Subscriber(
            "mobile_base/touch",
            mobile_base_driver.msg.Touch,
            self._touch_cb
        )

    def run(self):
 
        # Indicate to the world that we're running and ready to go:
        self._pub.publish(
            mayfield_msgs.msg.NodeStatus("cap_touch_chest_light", True)
        )
        rospy.spin()
 
    def shutdown(self):
        self._light_client.shutdown()
        self._touch_sub.unregister()
 
    def _touch_cb(self, msg):
        frame = [(0,0,0)] * clc.NUM_LEDS
        for i, electrode in enumerate(msg.electrodes):
            if(electrode):
                for led_idx in self.TOUCH_TO_LIGHT[i]:
                    frame[led_idx] = self.TOUCH_TO_COLOR[i]
        self._light_client.put_pixels(frame)
