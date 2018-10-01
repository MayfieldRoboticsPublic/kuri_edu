import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from collections import namedtuple


joy_input = namedtuple("joy_input", ["input_type", "index"])

PROFILES = {
    "xbox360": {
        "linear": joy_input("axes", 1),
        "angular": joy_input("axes", 0),
        "deadman": joy_input("buttons", 4),
        "linear_scale": 1.,
        "angular_scale": 6.,
        "color_x": joy_input("axes", 3),
        "coloy_y": joy_input("axes", 4),
        "button_top": joy_input("buttons", 3),
        "button_left": joy_input("buttons", 2),
        "button_right": joy_input("buttons", 1),
        "button_bottom": joy_input("buttons", 0),
        "shoot": joy_input("axes", 5)
    }
}

SOUND_MAP = {
    "button_top": "top.wav",
    "button_left": "left.wav",
    "button_right": "right.wav",
    "button_bottom": "bottom.wav"
}


class Joystick:
    def __init__(self):
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

    def play(self, input_name):
        if self.last_msg.header.stamp.to_sec() - self.last_played > 1.0:
            if self.current_play is not None:
                self.current_play.cancel()
            self.last_played = self.last_msg.header.stamp.to_sec()
            self.current_play = self.play_sound(SOUND_MAP[input_name])

    def play_sound(self, sound_file):
        rospy.logwarn("PLAYING {}".format(sound_file))
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
        self.joy_sub.unregister()
