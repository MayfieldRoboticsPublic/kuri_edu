## Joystick control for Kuri

This activity is to setup limited control of Kuri using an Xbox for windows controller, and extend it as desired.

This requires attaching the controller's receiver to the USB hub on Kuri, which requires partial disassembly.

By default this allows joystick driving, chest light color wheel, simple head movements and playing sounds. For keymappings look at the python node code in joystick_teleop.py, which includes deadman buttons.


### The files of interest are:

#### Launch file: launch/joystick_teleop.launch
- Note the joy_node (standard ROS node) /dev/input path, which needs to match what your device appears as. You also likely need to change permissions.

#### Main launch file: launch/kuri_edu.launch
- The joystick_teleop.launch is commented out by default. When enabling it, this should be uncommented, and the chest_light_controller should be commented out if you want to use the chestlight thumbwheel on the joystick.

#### Launch script: scripts/joystick_teleop

#### Node: src/kuri_edu/joystick_teleop.py
- Note the configuration at the top. Includes mapping to button and axes, matching what appears in the msg the joy_node published. Also sounds that are provided by the sound_play ROS package, which is not necessary.
