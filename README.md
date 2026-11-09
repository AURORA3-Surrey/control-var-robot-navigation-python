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
test_grid.py  ─┴─ grid_motion.py   planner (build_grid_plan), GridMotionNode, run_grid()
                       └─ grid_commands.py (Wait, Turn, Drive, MoveTo, ReturnHomeDiff)

vicon_to_odom.py ────────┐
vicon_yaw_calibration.py ┴─ nav_utils.py  angle maths, yaw-offset file,
                                          TwistSender (accel limit), shutdown helpers
```

`vicon_grid.py` and `test_grid.py` set where the pose comes from and their default args. `vicon_grid.py` uses the Vicon pose with the yaw offset and a filtered velocity, and `test_grid.py` uses wheel odometry. Motion commands are contained in `grid_motion.py`.

To add a new motion, add a `Command` subclass in `grid_commands.py` with a `step(node, now)` method.

## Robot-side code

Copy everything (the scripts import each other).

`vicon_yaw_offset.json` gets read and written on the platform.