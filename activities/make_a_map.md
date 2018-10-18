## Making a Map

Kuri uses custom mapping software called OORT which uses chains of graph nodes
to make flexible maps that can handle loop closures.  This lets Kuri improve
old maps as more data becomes available.  This activity walks the user through
creating a map using kuri_mapping.launch and visualizing the map using rviz

### Mapping with kuri_mapping.launch

Start with Kuri off the dock.  Then start the Kuri software in mapping mode
```
roslaunch kuri_edu kuri_mapping.launch
```

Kuri's chest light should start blinking, and should start with the head down
and eyes closed.

In another terminal, start the keyboard teleop node:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

To start mapping, place kuri on the dock.  When mapping starts, Kuri's eyes
will open and Kuri's head will point forward.  You can now build up the map
by driving Kuri with keyboard teleop.

Once the map contains about 20 square meters of free space, Kuri will begin
to smile

To finish mapping, drive Kuri back onto the dock with keyboard teleop.  
When mapping is complete, Kuri's eyes will close and Kuri's head will go 
down.  The newly created map will be saved in the user home directory under
```~/oort_maps/<uuid>```  At this point it is safe to stop kuri_mapping with
ctrl+c

### Visualizing Mapping Data
Using another computer which can reach Kuri over the network, you can 
watch Kuri create a map with rviz.  Start by sourcing an EDU binary

```
source /opt/edu/setup.bash
```

Next, set up your ROS_MASTER_URI environment variable to point to kuri
```
export ROS_MASTER_URI=http://kuri_hostname:11311
```

You may need to turn off Kuri's firewall.  On kuri, run

```
iptables -F
```

Double-check that you can reach kuri by doing

```
rostopic list
```

and verifying that you see a list of topics

Finally, launch rviz

```
rviz
```

You may need to add displays to rviz.  In the lower-left corner, select 'add'

  - Under "By display type" add "RobotModel" to add Kuri to the visualization
  - Under "By topic" add `/map` to add the map data to the visualization
