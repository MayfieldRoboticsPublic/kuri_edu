import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from collections import namedtuple
import math
import mobile_base
from .pulse_animation import PulseAnimation
import threading
import os
import subprocess


joy_input = namedtuple("joy_input", ["input_type", "index"])

PROFILES = {
    "xbox360": {
        "linear": joy_input("axes", 1),
        "angular": joy_input("axes", 0),
        "deadman": joy_input("buttons", 4),
        "linear_scale": 6.,
        "angular_scale": 9.,
        "color_set": joy_input("buttons", 5),
        "color_x": joy_input("axes", 3),
        "color_y": joy_input("axes", 4),
        "button_top": joy_input("buttons", 3),
        "button_left": joy_input("buttons", 2),
        "button_right": joy_input("buttons", 1),
        "button_bottom": joy_input("buttons", 0),
        "look_top": joy_input("buttons", 13),
        "look_left": joy_input("buttons", 11),
        "look_right": joy_input("buttons", 12),
        "look_bottom": joy_input("buttons", 14),
        "shoot": joy_input("axes", 5)
    }
}

SOUNDS_LOC = "/opt/ros/indigo/share/sound_play/sounds/"

SOUND_MAP = {
    "button_top": "say-beep.wav",
    "button_left": "NEEDS_UNPLUGGING.ogg",
    "button_right": "NEEDS_PLUGGING.ogg",
    "button_bottom": "BACKINGUP.ogg"
}

CHEST_LIGHT_FRAMERATE = 5.


class Joystick:
    def __init__(self):
        self._light_client = mobile_base.ChestLightClient()

        self._joint_states = mobile_base.JointStates()
        self._head_client = mobile_base.HeadClient(self._joint_states)
        assert self._head_client.wait_for_server(timeout=rospy.Duration(30.0))
        # At the start, open Kuri's eyes and point the head up:
        self._head_client.pan_and_tilt(
            pan=mobile_base.HeadClient.PAN_NEUTRAL,
            tilt=mobile_base.HeadClient.TILT_UP,
            duration=1.0
        )
        self._head_client.eyes_to(
            radians=mobile_base.HeadClient.EYES_OPEN,
            duration=0.5
        )

        self.vel_pub = rospy.Publisher(
            "/mobile_base/commands/velocity", Twist, queue_size=1)
        self.joy_sub = rospy.Subscriber(
            "/joy", Joy, self.joy_cb, queue_size=1)
        self.load_profile()
        self.last_msg = Joy()
        self.las_vel = Twist()
        self.last_played = 0.
        self.current_play = None
        self.deadman_pressed = False
        self.zero_twist_published = False
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_vel)

    def load_profile(self, profile=None):
        if profile is None:
            profile = rospy.get_param("profile", "xbox360")
        rospy.logwarn("LOADING PROFILE {}".format(profile))
        self.profile = PROFILES[profile]

    def get_in(self, key):
        msg = {
            "buttons": self.last_msg.buttons,
            "axes": self.last_msg.axes
        }
        return msg[self.profile[key].input_type][self.profile[key].index]

    def joy_cb(self, joy):
        self.last_msg = joy
        vel = Twist()
        vel.angular.z = self.profile["angular_scale"] * self.get_in("angular")
        vel.linear.x = self.profile["linear_scale"] * self.get_in("linear")
        self.las_vel = vel
        self.deadman_pressed = self.get_in("deadman")
        if self.get_in("button_top"):
            self.play("button_top")
        if self.get_in("button_left"):
            self.play("button_left")
        if self.get_in("button_right"):
            self.play("button_right")
        if self.get_in("button_bottom"):
            self.play("button_bottom")
        if self.get_in("color_set"):
            self.set_color()
        if self.get_in("look_top"):
            self.set_head("look_top")
        if self.get_in("look_left"):
            self.set_head("look_left")
        if self.get_in("look_right"):
            self.set_head("look_right")
        if self.get_in("look_bottom"):
            self.set_head("look_bottom")

    def _map_color(aself, rad_offset, radians, radius):
        if radians >= 0.:
            cradians = radians
        else:
            cradians = math.pi*2. + radians
        val = abs(cradians - rad_offset)
        if val > math.pi/2.:
            val = 0
        else:
            val = int((1. - val/(math.pi/2.))*255.*radius)
        return val

    def set_head(self, direction):
        if direction == "look_top":
            self._head_client.pan_and_tilt(
                pan=mobile_base.HeadClient.PAN_NEUTRAL,
                tilt=mobile_base.HeadClient.TILT_UP,
                duration=0.5
            )
            self._head_client.eyes_to(
                radians=mobile_base.HeadClient.EYES_HAPPY,
                duration=0.5
            )
        if direction == "look_left":
            self._head_client.pan_and_tilt(
                pan=mobile_base.HeadClient.PAN_LEFT,
                tilt=mobile_base.HeadClient.TILT_NEUTRAL,
                duration=0.5
            )
            self._head_client.eyes_to(
                radians=mobile_base.HeadClient.EYES_NEUTRAL,
                duration=0.5
            )
        if direction == "look_right":
            self._head_client.pan_and_tilt(
                pan=mobile_base.HeadClient.PAN_RIGHT,
                tilt=mobile_base.HeadClient.TILT_NEUTRAL,
                duration=0.5
            )
            self._head_client.eyes_to(
                radians=mobile_base.HeadClient.EYES_NEUTRAL,
                duration=0.5
            )
        if direction == "look_bottom":
            self._head_client.pan_and_tilt(
                pan=mobile_base.HeadClient.PAN_NEUTRAL,
                tilt=mobile_base.HeadClient.TILT_DOWN,
                duration=0.5
            )
            self._head_client.eyes_to(
                radians=mobile_base.HeadClient.EYES_SUPER_SAD,
                duration=0.5
            )

    def set_color(self):
        x = -1.*self.get_in("color_x")
        y = self.get_in("color_y")
        radians = math.atan2(y, x)
        radius = math.sqrt(x**2. + y**2.)
        if radius > 1.:
            radius = 1.
        radius = 1.
        R = self._map_color(0., radians, radius)
        G = self._map_color(math.pi*2./3., radians, radius)
        B = self._map_color(math.pi*4./3., radians, radius)
        self._light_client.put_pixels([(R, G, B)]*mobile_base.ChestLightClient.NUM_LEDS)

    def play(self, input_name):
        if self.last_msg.header.stamp.to_sec() - self.last_played > 1.0:
            #if self.current_play is not None:
            #    self.current_play.cancel()
            self.last_played = self.last_msg.header.stamp.to_sec()
            self.current_play = self.play_sound(SOUND_MAP[input_name])

    def play_sound(self, sound_file):
        #rospy.logwarn("PLAYING {}".format(sound_file))
        command = ['mplayer',
                   '-slave', '-quiet', '-novideo', '-ao', 'alsa',
                   SOUNDS_LOC + sound_file]
        with open(os.devnull, 'w') as DEVNULL:
            self._process = subprocess.Popen(command,
                                  stdin=subprocess.PIPE,
                                  stdout=DEVNULL,
                                  stderr=DEVNULL,
                                  preexec_fn=os.setsid)
        return None

    def publish_vel(self, event):
        if self.deadman_pressed:
            self.vel_pub.publish(self.las_vel)
            self.zero_twist_published = False
        elif not self.deadman_pressed and not self.zero_twist_published:
            self.vel_pub.publish(Twist())
            self.zero_twist_published = True

    def run(self):
        rospy.spin()

    def shutdown(self):
        self.timer.shutdown()
        self.joy_sub.unregister()

