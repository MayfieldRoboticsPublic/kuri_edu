## Make Kuri smile when a face is seen

This activity modifies kuri_edu/src/kuri_edu/head_controller.py to make kuri
smile when a face is detected

### Background

The default behavior of the head controller is to open Kuri's eyes and tilt
Kuri's head up.  It also enables the vision_bridge node face detection and
connects the HeadController._face_cb method to the vision/results ROS topic.  
Take a look at the HeadController.run method to see how it controls the head
and eyes using self._head_client.  The signature of the eyes_to and 
pan_and_tilt methods can be found below:

```
HeadClient.eyes_to
    Moves the robot's eye lids to the specified location in the duration
    specified
    
    :param radians: The eye position.  Expected to be between
    HeadClient.EYES_HAPPY and HeadClient.EYES_CLOSED
    
    :param duration: The amount of time to take to get the eyes to
    the specified location.

HeadClient.pan_and_tilt
    :param pan: The pan - expected to be between HeadClient.PAN_LEFT
    and HeadClient.PAN_RIGHT

    :param tilt: The tilt - expected to be between HeadClient.TILT_UP
    and HeadClient.TILT_DOWN

    :param duration: The amount of time to take to get the head to
    the specified location.

Constants:
    PAN_LEFT
    PAN_NEUTRAL
    PAN_RIGHT

    TILT_UP
    TILT_NEUTRAL
    TILT_DOWN

    EYES_OPEN
    EYES_NEUTRAL
    EYES_CLOSED
    EYES_HAPPY
    EYES_SUPER_SAD
    EYES_CLOSED_BLINK
```

By default, this ROS topic publishes messages at 6hz, regardless of whether
or not faces are seen.  You need to inspect the faces.faces array to see if
any faces are detected

### Activity 1

Using the Head Client eyes_to method, make Kuri smile whenever faces are seen
in the HeadController._face_cb method.

Potential Pitfalls:
  - If Kuri smiles too much, it can look creepy.  Consider recording the time
that smiling started and limiting the duration of smiles.  See
http://wiki.ros.org/rospy/Overview/Time for an overview of time in ROS

  - If a face moves quickly in and out of frame, Kuri's eyelids may chatter.  
Consider delaying when a smile starts or a smile ends until Kuri has seen N
frames of faces or no faces, instead of just one
