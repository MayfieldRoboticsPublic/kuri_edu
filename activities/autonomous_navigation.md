## Using Autonomous Navigation

This activity requires Kuri to have a map.  Create the map by following the
[making a map](./make_a_map.md) activity first.

Once you have map, you will need to tell kuri_edu where to find it.  Create
a simlink to the map you want to use.  kuri_edu.launch will follow this
simlink to find the map

```
cd ~
ln -s oort_maps/<uuid of map>/map.map_capnp
```

To remove the map simlink:

```
rm ~/map.map_capnp
```

### Sending goals with rviz

On the robot, start kuri_edu

```
roslaunch kuri_edu kuri_edu.launch
```

On a different computer with rviz installed, connect to Kuri
```
export ROS_MASTER_URI=http://<kuri-hostname>:11311
rviz
```

You may need to add displays to rviz.  In the lower-left corner, select 'add'

  - Under "By display type" add "RobotModel" to add Kuri to the visualization
  - Under "By topic" add `/map` to add the map data to the visualization
  - Under "By topic" add '/global_plan' to show the path that Kuri is attempting to follow

Now send a nav goal to Kuri by clicking "2D Nav Goal" at the top of rviz.  Click on the map
at the location you want kuri to drive to.  Click and drag to set the position and the orientation
