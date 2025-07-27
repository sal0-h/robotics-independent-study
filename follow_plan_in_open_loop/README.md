# Follow Plan in Open-Loop

This package contains scripts to make the robot follow a predefined plan of movements. The control is "open-loop" in the sense that the robot executes a sequence of straight-line movements and rotations without real-time feedback to correct its trajectory during each step. It completes one motion before starting the next.

## Scripts

- **`square.py`**: This script makes the robot move in a square pattern. The plan is defined as a list of tuples, where each tuple contains a distance to travel straight and an angle to turn. It uses the standard `/odom` topic for pose estimation.

- **`follow_odom.py`**: A more general script that follows a plan consisting of a series of (x, y) waypoints. The robot turns to face the next waypoint and then moves straight towards it. This script also relies on the standard `/odom` topic for its pose information.

- **`follow_kutta.py`**: This script is similar to `follow_odom.py` as it follows a list of (x, y) waypoints. However, it uses a custom pose estimation method based on Runge-Kutta integration of wheel encoder and IMU data instead of the standard `/odom` topic. It also calculates and plots the Mean Squared Error between its custom estimate and the ground truth odometry.

## How to Use

1.  Launch your robot simulation in Gazebo.
2.  Choose one of the scripts to run.
3.  You can modify the `plan` variable within each script to change the robot's trajectory.
    - For `square.py`, the plan is `[(distance, angle), ...]`.
    - For `follow_odom.py` and `follow_kutta.py`, the plan is `[(x, y), ...]`.
4.  Execute the chosen script. The robot will begin to follow the defined plan.
