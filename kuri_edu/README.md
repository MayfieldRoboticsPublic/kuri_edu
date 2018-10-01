# Kuri EDU
This is the kuri EDU ROS package.

## Tests
To run tests, you must first create a [catkin workspace](../activities/README.md#Setting-up-a-catkin-workspace)
with a source check-out of kuri_edu.  Once the workspace is built,

```
source devel/setup.bash
catkin_make run_tests -j1
catkin_test_results
```

will run all of the nosetests and ROS tests in the kuri_edu package.  -j1 is
required because only one gazebo simulation can run at a time, so many of the ROS
tests cannot be run in parallel.  During development, it may be desirable to run
individual tests.

### Running unit tests with nose

To run all of the unit tests with nosetests:
```
cd kuri_edu/kuri_edu/test
nosetests -v .
```

To run one file:
```
nosetests -v test_kuri_edu_launch.py
```

To run an individual test in a file:
```
nosetests -v test_light_animation.py:TestLightAnimation.test_get_framerate
```

### Running ROS tests
ROS tests are round in the `kuri_edu/kuri_edu/rostest` directory

```
rostest test_head_controller.launch
```

Output from the test node itself is not sent to the screen.  To debug a failing test,
you can get more information by running

```
rostest -t test_head_controller.launch
```
