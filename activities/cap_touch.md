## Make Kuri display which capacitive electrodes are active

This activity modifies kuri_edu/src/kuri_edu/cap_touch_chest_light.py to
make kuri display a different animation when charging on the dock

### Background

The default behavior of the chest LED is to illuminate white in the approximate
location where the capacitive touch sensor is active.

```
    LED indices are as follows (if looking at the robot straight on):
             [12]       [13]
        [11]                 [14]
                [4]   [5]
             [3]   [0]   [6]
                [2]   [1]
        [10]                 [7]
              [9]       [8]
```

```
    Electrode indices are as follows (if looking at the top of the robot's
    head):

                [5]     [2]
            [6]             [1]
                    [3]
            [7]             [0]
                    [4]
            -- FRONT OF ROBOT --
```

### Activity

Modify the mappings to change the LED behavior -- either remap the activated
LEDs, or change their colors. Some tips for modifying the mappings:

  - The center 7 LEDs are not partitioned, so they will naturally add to each
other. The default mapping has them only light up halfway to compensate for the
additional brightness.
  - The sample class is very simple. More complex interactions are possible,
but will require additional structures to function. Challenge yourself:
    - Make each electrode also affect adjacent LEDs, and blend the colors of
    overlapping influences
    - Have the color or brightness change with how long an electrode has
    been active
