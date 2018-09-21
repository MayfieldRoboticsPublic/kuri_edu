## Disable automatic back-up when Kuri is on the dock

This activity modifies kuri_edu/src/kuri_edu/safety_controller.py to disable
the back-up reaction when Kuri is on the dock

### Background

Stopping the robot from moving after a bump event is handled by low level
firmware.  When a bump is detected, the firmware stops the wheels until
the safety controller node overrides the lock-out, backs the robot up,
re-enables safety lock-outs, and clears the safety condition.

```
    # Override the safety status and back the robot up
    self._safety_client.safety_override(current_status)
    self._back_up(rate)
    self._safety_client.safety_override(self.UNHANDLED_EVENTS)
    self._safety_client.safety_clear(current_status)

```

You can verify that the safety controller is working by tapping the bump shell
when the kuri_edu software is running.  Kuri will back up about an inch
automatically

### Activity

Modify the safety controller to supress the back-up when kuri is on the dock.

Some tips:
  - Create a new file kuri_edu/src/kuri_edu/dock_monitor.py that contains a
DockMonitor class.  This class can subscribe to the /mobile_base/power topic 
and keep track of the latest value of `dock_present`
  - Add the DockMOnitor class tokuri_edu/src/kuri_edu/\_\_init\_\_.py so it
can be imported by tests
  - Instantiate the DockMonitor in the SafetyController `__init__` method  
  - Shut down the DockMonitor in the SafetyController `shutdown` method
  - The SafetyController still needs to clear the safety status even when 
Kuri is on the dock.  You only need to suppress the back-up part

It is also possible to integration test this behavior.  There is a test in
kuri_edu/rostest/test_safety_controller.py that uses a simulated Kuri to test
the back-up behavior.  Another test could be added to check that the safety
reaction is suppressed when kuri is on the dock.  The simulation provides
a ROS service called /sim_interface/dock which can change the simulated
dock state of the robot

```
    import rospy
    import gizmo_hw_sim.srv

    dock_srv = rospy.ServiceProxy(
        "/sim_interface/dock",
        gizmo_hw_sim.srv.Dock
    )

    dock_srv(True, True)  # Simulate Kuri on the dock and Kuri charging
```

A new ROS test would look something like this:

```
def test_04_no_back_up_on_dock(self):
    self.dock_srv(True, True)
    rospy.sleep(0.5)  # Give time for 'docked' to propegate.  Simplified

    initial_pos = self.get_gizmo_location()
    status_mon = SafetyStatusMonitor()

    self.kick_srv(False, True, False)  # center-bump

    status_mon.assertSafetyConditionSeen()
    status_mon.assertSafetyConditionCleared()

    final_pos = self.get_gizmo_location()

    # Make sure Kuri stays put:
    self.assertAlmostEqual(initial_pos[0], final_pos[0], places=2)
    self.assertAlmostEqual(initial_pos[1], final_pos[1], places=2)
```
