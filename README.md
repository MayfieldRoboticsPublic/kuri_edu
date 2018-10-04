# Kuri EDU
This repository contains software and activities intended to be used for
educational purposes.  The kuri_edu folder contains a ROS package that provides 
a minimal running robot framework and the activities folder contains ideas and
instructions for extending the robot behavior

Note that the kuri_edu package is not stand-alone.  It expects to overlay
a binary install on the Kuri robot - either `/opt/gizmo` or `/opt/edu` will
contain the kuri_edu dependencies.

## Stand-alone launch Files
 * kuri_edu.launch: Launches Kuri, and the nodes required to operate Kuri with 
minimal autonomous behavior.  This launch file glues together several other launch files:
    * kuri_drive.launch - Runs the hardware abstraction layer and depth-sensor.
    * madmux_daemon.launch - Provides a hardware abstraction layer to Kuri's camera
    * vision_bridge.launch - Performs face and object detection

## Getting Started:
Before attempting any activities, you should make sure you're able to launch the
kuri EDU software without any modifications.

Start by establishing an SSH connection to Kuri
`ssh mayfield@kuri_hostname`

Where kuri_hostname is probably `kuri-LAST_7_DIGITS_OF_SERIAL_NUMBER`

To set up your shell, source the setup.sh or setup.bash files in the installed
kuri EDU binary

`source /opt/edu/setup.sh`

Finally, launch the kuri_edu software
`roslaunch kuri_edu kuri_edu.launch`

This should launch a ROS master, and some ROS nodes:
```
mayfield@kuri-0001c:~$ roslaunch kuri_edu kuri_edu.launch &
[1] 19589
mayfield@kuri-0001c:~$ ... logging to /home/mayfield/.ros/log/8997b7cc-bde9-11e8-9393-00073249657e/roslaunch-kuri-0001c-19589.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://kuri-0001c:36850/

. . .

process[robot_state_publisher-2]: started with pid [19637]
process[mobile_base-3]: started with pid [19638]
process[depthscan-4]: started with pid [19639]
process[madmux_daemon-5]: started with pid [19645]
process[vision_bridge-6]: started with pid [19658]
process[safety_controller-7]: started with pid [19665]
process[head_controller-8]: started with pid [19668]
process[chest_light_controller-9]: started with pid [19678]
```

At this point, you should observe that your Kuri robot's eyes open, the head 
looks up, and the chest light begins to play a simple pattern.  The color of 
the chest LED represents Kuri's battery level.  Green = 100%, Red = 20% or 
lower.  Kuri should also back up when the bump shell is pressed.

You can stop the kuri_edu software with ctrl-c

## Keyboard Teleop Demo

You can also drive Kuri like a remote control car using the 
teleop_twist_keyboard node.  You may need to install this node on Kuri.

```
sudo apt-get update
sudo apt-get install ros-indigo-teleop-twist-keyboard
```

See the Troubleshooting section below if you're having trouble installing
the teleop twist keyboard package

Next, you'll need to start the kuri_edu software and the keyboard teleop node. 
You can do this using two different terminals SSH'd to Kuri, or you can
SSH to kuri and run `tmux` to multiplex your terminals, or you can run
kuri_edu in the background like
```
roslaunch kuri_edu kuri_edu.launch &
```

If you launch kuri_edu in the background, you can stop it by bringing it to the
forground with 
```
fg
```
and then stopping it with ctrl-c

Once kuri_edu is running in another terminal or in the background, launch the
keyboard teleop node:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

By default, kuri_edu disables all the cliff sesnors and allows Kuri to be 
wherever you want.  Be careful not to drive it down stairs!

### Troubleshooting
If you get an error like:
```
mayfield@kuri-0001c:~$ sudo apt-get update

. . .

mayfield@kuri-0001c:~$ sudo apt-get install ros-indigo-teleop-twist-keyboard
Reading package lists... Done
Building dependency tree       
Reading state information... Done
E: Unable to locate package ros-indigo-teleop-twist-keyboard
```

You may need to add the ROS indigo debian sources to your robot.  If the
`/etc/apt/sources.list.d` folder does not exist, create it with

```
sudo mkdir -p /etc/apt/sources.list.d
```

Then

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

See http://wiki.ros.org/indigo/Installation/Ubuntu for more information

## Next Steps
If you've made it this far, you're ready to start an activity.  Check 
README.md in the 'activities' folder for an overview of the set-up steps
required to complete any activity.
