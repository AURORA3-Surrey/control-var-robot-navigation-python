# control-var-robot-navigation-python
Scripts for robot motion control, ROS 2 communication, and navigation.

## Scripts

- `vicon_yaw_calibration.py` drives a short straight line and saves `vicon_yaw_offset.json`. Run it once a day before using Vicon.
- `vicon_grid.py` runs on the grid using the Vicon pose. It won't start without a yaw calibration from the same day. `--help` lists the flags to skip that.
- `test_grid.py` runs on the same grid using wheel odometry.
- `vicon_to_odom.py` republishes the Vicon pose as `nav_msgs/Odometry`.
- `run_grid_loop.sh` loops `vicon_grid.py` for demos.
- `emergencystop.py` sends zero velocity and turns the motors off.
- `vicon/connection_check.py` checks the Vicon connection.

## Structure

```
vicon_grid.py ─┐
test_grid.py  ─┴─► grid_motion.py ─┐   planner, control_tick, GridMotionNode, run_grid()
vicon_to_odom.py ──────────────────┤
vicon_yaw_calibration.py ──────────┴─► nav_utils.py   angle maths, yaw-offset file, TwistSender (accel limit), shutdown helpers
```

`vicon_grid.py` and `test_grid.py` only set where the pose comes from and their default args. The rest is in `grid_motion.py`.

## Robot-side code

Copy everything (the scripts import each other).

`vicon_yaw_offset.json` gets read and written on the platform.