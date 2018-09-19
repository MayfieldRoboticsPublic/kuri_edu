## Make Kuri display a different animation when charging

This activity modifies kuri_edu/src/kuri_edu/chest_light_controller.py to
make kuri display a different animation when charging on the dock

### Background

The default behavior of the chest LED is to display a color to indicate the
charge level of the battery, and to display a simple pattern so the user can
tell the difference between kuri with software running and kuri without
software running.

The chest_light_controller delegates exactly which pixels to output to a
PulseAnimation object by default.  This animation contains four frames
that make it look like the chest light is growing and then shrinking.

```
    LED indeces are as follows (if looking at the robot straight on):
             [12]       [13]
        [11]                 [14]
                [4]   [5]
             [3]   [0]   [6]
                [2]   [1]
        [10]                 [7]
              [9]       [8]
```

### Activity

Create a new animation that will play when Kuri is on the dock charging.
Perhaps an animation that looks like the chest light is filling up from
bottom to top over and over again.  Some tips for making a new animation:

  - Create a new file kuri_edu/src/kuri_edu/charging_animation.py that contains
a ChargingAnimation class
  - Add the ChargingAnimation to kuri_edu/src/kuri_edu/__init__.py so it can be
imported by tests
  - Make sure the ChargingAnimation inherits from LightAnimation and provides
its own `next` method to emit frames.  The implementation will probably be
very similar to the PulseAnimation

To play the ChargingAnimation when Kuri is on the dock, the ChestLedController
class in kuri_edu/src/kuri_edu/chest_light_controller.py needs to be updated.
The _power_cb method can change self._anim to a ChargingAnimation when Kuri
is on the dock, and change it back to a PulseAnimation when Kuri is off the
dock
