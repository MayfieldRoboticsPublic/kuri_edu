# Kuri EDU
A minimal kuri robot that exposes kuri's subsystems without any autonomous behavior

## Launch Files
 * kuri_remote.launch: Launches a Kuri that can be driven with the standard keyboard_teleop node
 * kuri_edu.launch: Launches kuri_remote along with the following subsystems
   * madmux: Kuri's camera

## Keyboard Teleop Demo

To keyboard teleop a Kuri, run kuri_remote.launch or kuri_edu.launch on the robot:
```
roslaunch kuri_edu kuri_remote.launch
```

Then to use keyboard teleop (http://wiki.ros.org/teleop_twist_keyboard) on the robot, or on a remote system set up to communicate
with the ROS master on Kuri:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

By default, kuri_remote disables all safety systems and allows the robot to be driven wherever the
user wishes.  Be careful not to drive it down stairs!
